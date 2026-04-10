import math
import threading
import time
import traceback
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
import pinocchio as pin

ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

HAND_JOINTS = [
    'FFJ4', 'FFJ3', 'FFJ2', 'FFJ1',     
    'MFJ4', 'MFJ3', 'MFJ2', 'MFJ1',      
    'RFJ4', 'RFJ3', 'RFJ2', 'RFJ1',      
    'LFJ5', 'LFJ4', 'LFJ3', 'LFJ2', 'LFJ1',  
    'THJ5', 'THJ4', 'THJ3', 'THJ2', 'THJ1',
]

TOOL0_TO_GRASP = np.array([0.266, 0.0, 0.0], dtype=float)

class URKinematics:
    def __init__(self, urdf_xml, logger=None):
        full = pin.buildModelFromXML(urdf_xml)
        
        arm_set = set(ARM_JOINTS)
        lock_ids = [
            jid for jid in range(1, full.njoints)
            if full.names[jid] not in arm_set
        ]
        q_ref = pin.neutral(full)
        self.model = pin.buildReducedModel(full, lock_ids, q_ref)

        if not self.model.existFrame('tool0'):
            raise RuntimeError("tool0 frame missing from URDF model")

        tool0_id = self.model.getFrameId('tool0')
        tf = self.model.frames[tool0_id]
        grasp_local = pin.SE3(np.eye(3), TOOL0_TO_GRASP)
        grasp_frame = pin.Frame(
            'grasp_point',
            tf.parentJoint,
            tool0_id,
            tf.placement * grasp_local,
            pin.FrameType.OP_FRAME,
        )
        self.grasp_id = self.model.addFrame(grasp_frame)
        self.data = self.model.createData()

        self._q_idx = np.zeros(6, dtype=int)
        for i, name in enumerate(ARM_JOINTS):
            jid = self.model.getJointId(name)
            self._q_idx[i] = self.model.joints[jid].idx_q

        if self.model.nq != 6:
            raise RuntimeError(
                f"Reduced model has nq={self.model.nq}, expected 6")

        if logger is not None:
            logger.info(
                f'[URKinematics] reduced model ready: '
                f'nq={self.model.nq}, njoints={self.model.njoints}, '
                f'grasp_frame_id={self.grasp_id}')

    def _pack(self, q_arm):
        q = np.zeros(self.model.nq, dtype=float)
        for i in range(6):
            q[self._q_idx[i]] = q_arm[i]
        return q

    def _unpack(self, q):
        return np.array([q[self._q_idx[i]] for i in range(6)], dtype=float)

    def grasp_pose(self, q_arm):
        q = self._pack(q_arm)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        M = self.data.oMf[self.grasp_id]
        return M.translation.copy(), M.rotation.copy()

    def solve_ik(self, q_arm_init, p_des, R_des,
                 tol=1e-3, max_iter=300,
                 damping=0.08, step_limit=0.20):
        q = self._pack(q_arm_init)
        oMdes = pin.SE3(np.asarray(R_des, dtype=float),
                        np.asarray(p_des, dtype=float))
        I6 = np.eye(6)
        for _ in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            oMf = self.data.oMf[self.grasp_id]
            dMf = oMf.actInv(oMdes)
            err = pin.log6(dMf).vector
            if np.linalg.norm(err) < tol:
                break
            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.grasp_id, pin.LOCAL)
            A = J @ J.T + (damping ** 2) * I6
            dq = J.T @ np.linalg.solve(A, err)
            n = np.linalg.norm(dq)
            if n > step_limit:
                dq *= step_limit / n
            q = pin.integrate(self.model, q, dq)
        return self._unpack(q)

def _grasp_rotation():
    x = np.array([0.0, 1.0, 0.0], dtype=float)
    y = np.array([1.0, 0.0, 0.0], dtype=float)
    z = np.cross(x, y)
    z /= np.linalg.norm(z)
    return np.column_stack((x, y, z))


def _lerp_path(p0, p1, n_steps):
    return [p0 + (p1 - p0) * t
            for t in np.linspace(0.0, 1.0, n_steps + 1)[1:]]

class ImpedanceInsertionNode(Node):
    BASE_WORLD     = np.array([0.0, -0.55, 1.0], dtype=float)
    SQ_PEG_WORLD   = np.array([ 0.48, -0.15, 1.02], dtype=float)
    RND_PEG_WORLD  = np.array([-0.43, -0.17, 1.02], dtype=float)
    SQ_HOLE_WORLD  = np.array([ 0.49,  0.05, 1.02], dtype=float)
    RND_HOLE_WORLD = np.array([-0.41,  0.05, 1.02], dtype=float)

    APPROACH_Z         = 0.25  
    PREGRASP_X_OFFSET  = 0.10  
    GRASP_HEIGHT_SQ    = 0.050 
    GRASP_HEIGHT_RND   = 0.050 
    LIFT_Z             = 0.10   
    RETRACT_Z          = 0.18   

    INSERT_DT          = 0.08  
    INSERT_SPEED       = 0.03   
    K_XY               = 0.40   
    D_XY               = 0.05   
    K_Z                = 0.25   
    D_Z                = 0.03   
    SPIRAL_RMAX        = 0.004  
    SPIRAL_GROWTH      = 0.0002 
    SPIRAL_OMEGA       = 0.50   

    HAND_OPEN = [
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0,
        0.20, 0.20, 0.0, 0.0, 0.0,
    ]

    HAND_PRE_GRASP = [
        0.0,  0.35, 0.25, 0.15,                  
        0.0,  0.35, 0.25, 0.15,                   
        0.0,  0.35, 0.25, 0.15,                 
        0.0, 0.10, 0.30, 0.20, 0.10,              
        1.00, 0.50, 0.15, 0.10, 0.05,          
    ]

    HAND_POWER_SQ = [
        0.0,  1.50, 1.30, 0.90,                 
        0.0,  1.50, 1.35, 0.95,             
        0.0,  1.50, 1.30, 0.90,               
        0.0, 0.40, 1.30, 1.05, 0.75,          
        0.20, 0.20, 0.0, 0.0, 0.0,             
    ]

    HAND_POWER_RND = [
        0.0,  1.50, 1.30, 0.90,                   
        0.0,  1.50, 1.35, 0.95,                    
        0.0,  1.50, 1.30, 0.90,            
        0.0, 0.40, 1.30, 1.05, 0.75,   
        0.20, 0.20, 0.0, 0.0, 0.0,      
    ]

    def __init__(self):
        super().__init__('impedance_insertion')

        self.joint_pos = None
        self._joint_map = None
        self.kin = None
        self._urdf_loaded = threading.Event()
        self.R_GRASP = _grasp_rotation()

        latch_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            String, '/robot_description', self._urdf_cb, latch_qos)
        self.create_subscription(
            JointState, '/joint_states', self._js_cb, 20)

        self._arm_ac = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        self._hand_ac = ActionClient(
            self, FollowJointTrajectory,
            '/hand_controller/follow_joint_trajectory')

        self.get_logger().info('Waiting for arm action server ...')
        self._arm_ac.wait_for_server()
        self.get_logger().info('Waiting for hand action server ...')
        self._hand_ac.wait_for_server()
        self.get_logger().info('Controllers ready -- starting demo thread.')

        threading.Thread(target=self._run_demo, daemon=True).start()

    def _urdf_cb(self, msg):
        if self.kin is not None:
            return
        try:
            self.kin = URKinematics(msg.data, self.get_logger())
            self.get_logger().info('Pinocchio kinematics loaded from /robot_description')
            self._urdf_loaded.set()
        except Exception as exc:
            self.get_logger().error(f'Pinocchio init failed: {exc}')
            self.get_logger().error(traceback.format_exc())

    def _js_cb(self, msg):
        if self._joint_map is None:
            self._joint_map = {n: i for i, n in enumerate(msg.name)}
        idx = [self._joint_map[n] for n in ARM_JOINTS if n in self._joint_map]
        if len(idx) == 6:
            self.joint_pos = np.array(
                [msg.position[i] for i in idx], dtype=float)

    def _send_traj(self, ac, names, positions_list, times):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        for q, t in zip(positions_list, times):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            pt.velocities = [0.0] * len(names)
            sec = int(t)
            pt.time_from_start = Duration(
                sec=sec, nanosec=int((t - sec) * 1e9))
            goal.trajectory.points.append(pt)

        future = ac.send_goal_async(goal)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().warn('Goal-send timeout')
            return False

        gh = future.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn('Trajectory rejected')
            return False

        res_future = gh.get_result_async()
        deadline = time.monotonic() + max(times[-1], 1.0) + 15.0
        while not res_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return res_future.done()

    def _arm(self, q, dur=2.0):
        return self._send_traj(self._arm_ac, ARM_JOINTS, [q], [dur])

    def _arm_multi(self, waypoints):
        if not waypoints:
            return True
        qs, ts = zip(*waypoints)
        return self._send_traj(self._arm_ac, ARM_JOINTS, qs, ts)

    def _hand(self, qh, dur=1.0):
        return self._send_traj(self._hand_ac, HAND_JOINTS, [qh], [dur])

    def _wait_ready(self):
        while rclpy.ok() and (self.joint_pos is None or self.kin is None):
            time.sleep(0.1)

    def _q(self):
        return (np.array(self.joint_pos, dtype=float)
                if self.joint_pos is not None
                else np.zeros(6, dtype=float))

    def _w2b(self, world_pt):
        return np.asarray(world_pt, dtype=float) - self.BASE_WORLD

    def _goto(self, q_seed, p_target, dur=1.5):
        q_cmd = self.kin.solve_ik(q_seed, p_target, self.R_GRASP)
        self._arm(q_cmd, dur)
        return q_cmd

    def _linear_move(self, q_start, p_target, total_time=1.5, steps=10):
        q = np.array(q_start, dtype=float)
        p0, _ = self.kin.grasp_pose(q)
        waypoints = []
        dt = total_time / max(steps, 1)
        t = 0.0
        for p in _lerp_path(p0, np.asarray(p_target, dtype=float), steps):
            q = self.kin.solve_ik(q, p, self.R_GRASP, max_iter=160)
            t += dt
            waypoints.append((q.copy(), t))
        self._arm_multi(waypoints)
        return q

    def _pick(self, peg_world, grasp_h, hand_close_pose, label):

        pb = self._w2b(peg_world)
        q  = self._q()

        grasp_pt     = pb + np.array([0.0, 0.0, grasp_h])
        pre_grasp_pt = grasp_pt + np.array([self.PREGRASP_X_OFFSET, 0.0, 0.0])
        above_pt     = pre_grasp_pt + np.array([0.0, 0.0, self.APPROACH_Z])
        lifted_pt    = grasp_pt + np.array([0.0, 0.0, self.LIFT_Z])

        self.get_logger().info(f'[{label}] Pre-shaping hand')
        self._hand(self.HAND_OPEN, 0.8)
        time.sleep(0.3)
        self._hand(self.HAND_PRE_GRASP, 0.8)
        time.sleep(0.5)

        self.get_logger().info(f'[{label}] Moving above peg')
        q = self._goto(q, above_pt, dur=2.5)
        time.sleep(0.3)

        self.get_logger().info(f'[{label}] Lowering to pre-grasp')
        q = self._linear_move(q, pre_grasp_pt, total_time=1.2, steps=8)
        time.sleep(0.2)

        self.get_logger().info(f'[{label}] Sliding to grasp position')
        q = self._linear_move(q, grasp_pt, total_time=0.8, steps=6)
        time.sleep(0.3)

        self.get_logger().info(f'[{label}] Closing hand -- power grasp')
        mid = [0.5 * (a + b) for a, b in zip(self.HAND_PRE_GRASP, hand_close_pose)]
        self._hand(mid, 0.6)
        time.sleep(0.4)
        self._hand(hand_close_pose, 0.8)
        time.sleep(0.6)

        self.get_logger().info(f'[{label}] Lifting peg')
        q = self._linear_move(q, lifted_pt, total_time=1.2, steps=8)
        time.sleep(0.3)

        return q

    def _insert(self, q_start, hole_world, depth, label):
        hb = self._w2b(hole_world)
        q = q_start.copy()

        above_hole = hb + np.array([0.0, 0.0, self.APPROACH_Z])
        insert_pt  = hb + np.array([0.0, 0.0, 0.15-depth])
        retract_pt = hb + np.array([0.0, 0.0, self.RETRACT_Z])

        self.get_logger().info(f'[{label}] Moving above hole')
        q = self._linear_move(q, above_hole, total_time=2.0, steps=12)
        time.sleep(0.3)

        self.get_logger().info(f'[{label}] Pushing peg straight down')
        q = self._linear_move(q, insert_pt, total_time=2.0, steps=15)
        time.sleep(0.3)

        self.get_logger().info(f'[{label}] Releasing peg')
        self._hand(self.HAND_OPEN, 0.5)
        time.sleep(0.5)

        self.get_logger().info(f'[{label}] Retracting')
        q = self._linear_move(q, retract_pt, total_time=1.2, steps=8)
        return q

    def _impedance_insert(self, q_start, hole_b, depth, label):
        q  = np.array(q_start, dtype=float)
        dt = self.INSERT_DT
        p_prev, _ = self.kin.grasp_pose(q)
        z_top     = p_prev[2]
        z_bottom  = hole_b[2] - depth
        z_des     = z_top
        theta     = 0.0
        t_acc     = 0.2
        waypoints = []
        self.get_logger().info(
            f'  z_top={z_top:.4f}  z_bottom={z_bottom:.4f}  depth={depth:.3f}')

        while z_des > z_bottom and rclpy.ok():
            p_now, _ = self.kin.grasp_pose(q)
            vel = (p_now - p_prev) / max(dt, 1e-6)

            progress = np.clip(
                (z_top - z_des) / max(z_top - z_bottom, 1e-6), 0.0, 1.0)
            r = min(self.SPIRAL_RMAX,
                    self.SPIRAL_GROWTH * (1.0 + 30.0 * progress))
            theta += self.SPIRAL_OMEGA
            spiral_xy = np.array([r * math.cos(theta), r * math.sin(theta)])

            z_des = max(z_bottom, z_des - self.INSERT_SPEED * dt)

            xy_nom = hole_b[:2] + spiral_xy
            e_xy   = p_now[:2] - xy_nom
            c_xy   = -self.K_XY * e_xy - self.D_XY * vel[:2]

            e_z = p_now[2] - z_des
            c_z = -self.K_Z * e_z - self.D_Z * vel[2]

            p_cmd = np.array([
                xy_nom[0] + c_xy[0],
                xy_nom[1] + c_xy[1],
                z_des     + c_z,
            ], dtype=float)

            q = self.kin.solve_ik(
                q, p_cmd, self.R_GRASP,
                max_iter=100, damping=0.10, step_limit=0.12)
            t_acc += dt
            waypoints.append((q.copy(), t_acc))
            p_prev = p_now.copy()

        t_acc += 0.5
        waypoints.append((q.copy(), t_acc))

        self.get_logger().info(
            f'  Insertion done -- {len(waypoints)} waypoints, {t_acc:.1f} s')
        self._arm_multi(waypoints)
        return q

    def _run_demo(self):
        self._wait_ready()
        if not rclpy.ok():
            return

        q_home = np.array([
            math.pi / 2,    
           -math.pi / 2,  
            math.pi / 2,      
           -math.pi / 2,   
           -math.pi / 2,   
            0.0,   
        ], dtype=float)

        self.get_logger().info('Going to home pose')
        self._arm(q_home, 2.0)
        self._hand(self.HAND_OPEN, 1.0)
        time.sleep(1.0)

        try:
            self.get_logger().info('====== SQUARE PEG ======')

            q = self._pick(
                peg_world=self.SQ_PEG_WORLD,
                grasp_h=self.GRASP_HEIGHT_SQ,
                hand_close_pose=self.HAND_POWER_SQ,
                label='SQ',
            )
            q = self._insert(
                q_start=q,
                hole_world=self.SQ_HOLE_WORLD,
                depth=0.015,
                label='SQ',
            )

            self.get_logger().info('Returning to home')
            self._arm(q_home, 1.5)
            time.sleep(1.0)

            self.get_logger().info('====== ROUND PEG ======')

            q = self._pick(
                peg_world=self.RND_PEG_WORLD,
                grasp_h=self.GRASP_HEIGHT_RND,
                hand_close_pose=self.HAND_POWER_RND,
                label='RND',
            )
            q = self._insert(
                q_start=q,
                hole_world=self.RND_HOLE_WORLD,
                depth=0.015,
                label='RND',
            )

            self.get_logger().info('Returning to home')
            self._arm(q_home, 2.0)
            self.get_logger().info('====== DEMO COMPLETE ======')

        except Exception as exc:
            self.get_logger().error(f'Demo failed: {exc}')
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceInsertionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
