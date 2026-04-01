"""
ROS 2 Jazzy / Gazebo Harmonic
UR5e + Shadow Hand peg-in-hole demo with:
- corrected world->base transform
- full pose IK for grasping
- staged thumb-index pinch
- Cartesian compliance / admittance-style insertion
"""

import math
import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration


# ───────────────────────────── UR5e kinematics ───────────────────────────── #

UR5E_DH = np.array([
    [0.0,      0.1625,   math.pi / 2],
    [-0.425,   0.0,      0.0],
    [-0.3922,  0.0,      0.0],
    [0.0,      0.1333,   math.pi / 2],
    [0.0,      0.0997,  -math.pi / 2],
    [0.0,      0.0996,   0.0],
], dtype=float)

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

# Calibrated grasp point relative to tool0.
# This is the point we try to place on the peg / hole centerline.
TOOL0_TO_PINCH = np.array([0.09, 0.0, 0.0], dtype=float)


def wrap_to_pi(q):
    return (q + math.pi) % (2.0 * math.pi) - math.pi


def _dh(a, d, alpha, theta):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0, sa,       ca,      d],
        [0.0, 0.0,      0.0,     1.0],
    ], dtype=float)


def fk(q):
    T = np.eye(4, dtype=float)
    for i in range(6):
        T = T @ _dh(UR5E_DH[i, 0], UR5E_DH[i, 1], UR5E_DH[i, 2], q[i])
    return T


def tool_pose(q):
    T = fk(q)
    R = T[:3, :3]
    p = T[:3, 3] + R @ TOOL0_TO_PINCH
    return p, R


def rotation_error(R, R_des):
    return 0.5 * (
        np.cross(R[:, 0], R_des[:, 0]) +
        np.cross(R[:, 1], R_des[:, 1]) +
        np.cross(R[:, 2], R_des[:, 2])
    )


def jacobian6(q, eps=1e-6):
    p0, R0 = tool_pose(q)
    J = np.zeros((6, 6), dtype=float)

    for i in range(6):
        dq = np.array(q, dtype=float)
        dq[i] += eps
        p1, R1 = tool_pose(dq)
        J[:3, i] = (p1 - p0) / eps
        J[3:, i] = rotation_error(R0, R1) / eps

    return J


def solve_ik_pose(
    q0,
    p_des,
    R_des,
    tol_pos=1e-3,
    tol_ori=6e-2,
    max_iter=250,
    damping=0.08,
    step_limit=0.20,
):
    q = np.array(q0, dtype=float)

    for _ in range(max_iter):
        p, R = tool_pose(q)
        e_pos = p_des - p
        e_ori = rotation_error(R, R_des)

        if np.linalg.norm(e_pos) < tol_pos and np.linalg.norm(e_ori) < tol_ori:
            break

        e = np.concatenate([e_pos, e_ori])
        J = jacobian6(q)
        A = J @ J.T + (damping ** 2) * np.eye(6)
        dq = J.T @ np.linalg.solve(A, e)

        n = np.linalg.norm(dq)
        if n > step_limit:
            dq *= step_limit / n

        q += dq
        q = wrap_to_pi(q)

    return q


def linear_path(p0, p1, steps):
    return [p0 * (1.0 - a) + p1 * a for a in np.linspace(0.0, 1.0, steps + 1)[1:]]


def make_grasp_rotation():
    # tool0 x-axis points downward
    x = np.array([0.0, 1.0, 0.0], dtype=float)
    # tool0 y-axis points along world +x
    y = np.array([1.0, 0.0, 0.0], dtype=float)
    # tool0 z-axis chosen to keep a right-handed frame
    z = np.cross(x, y)
    z /= np.linalg.norm(z)
    return np.column_stack((x, y, z))


# ───────────────────────────── Demo node ───────────────────────────── #

class ImpedanceInsertionNode(Node):

    BASE_WORLD = np.array([0.0, -0.4, 1.0], dtype=float)

    SQ_PEG_WORLD   = np.array([ 0.10, -0.10, 1.05], dtype=float)
    RND_PEG_WORLD  = np.array([-0.15, -0.10, 1.04], dtype=float)
    SQ_HOLE_WORLD  = np.array([ 0.10,  0.10, 1.05], dtype=float)
    RND_HOLE_WORLD = np.array([-0.15,  0.10, 1.05], dtype=float)

    APPROACH_Z = 0.16
    PREGRASP_Y_OFFSET = -0.06
    FINAL_GRASP_Y_OFFSET = -0.008
    LIFT_DELTA_Z = 0.10
    RETRACT_Z = 0.18

    SQUARE_GRASP_HEIGHT = 0.080
    ROUND_GRASP_HEIGHT = 0.078

    INSERT_DT = 0.08
    INSERT_SPEED = 0.010
    INSERT_XY_K = 0.35
    INSERT_XY_D = 0.04
    INSERT_Z_K = 0.20
    INSERT_Z_D = 0.02
    SEARCH_RADIUS_MAX = 0.004
    SEARCH_GROWTH = 0.00025
    SEARCH_OMEGA = 0.45

    HAND_OPEN = [
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0,
        0.2, 0.2, 0.0, 0.0, 0.0,
    ]

    HAND_PRE_GRASP = [
        0.0, 0.35, 0.25, 0.15,   # FF
        0.0, 0.35, 0.25, 0.15,   # MF
        0.0, 0.35, 0.25, 0.15,   # RF
        0.15, 0.10, 0.30, 0.20, 0.10,  # LF helps wrap
        1.05, 0.55, 0.20, 0.15, 0.10,  # thumb opposed and opened
    ]
    
    HAND_CLOSED_SQ = [
        0.0, 1.10, 0.90, 0.55,   # FF
        0.0, 1.15, 0.95, 0.60,   # MF
        0.0, 1.10, 0.90, 0.55,   # RF
        0.25, 0.20, 0.95, 0.75, 0.45,  # LF
        1.20, 0.95, 0.60, 0.45, 0.35,  # thumb wraps inward
    ]

    HAND_CLOSED_RND = [
        0.0, 1.00, 0.80, 0.50,
        0.0, 1.05, 0.85, 0.55,
        0.0, 1.00, 0.80, 0.50,
        0.20, 0.15, 0.85, 0.65, 0.40,
        1.15, 0.90, 0.55, 0.40, 0.30,
    ]

    def __init__(self):
        super().__init__('impedance_insertion')

        self.joint_pos = None
        self._joint_map = None
        self.R_GRASP = make_grasp_rotation()

        self.create_subscription(JointState, '/joint_states', self._js_cb, 20)

        self._arm_ac = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        self._hand_ac = ActionClient(
            self,
            FollowJointTrajectory,
            '/hand_controller/follow_joint_trajectory'
        )

        self.get_logger().info('Waiting for arm trajectory server...')
        self._arm_ac.wait_for_server()
        self.get_logger().info('Waiting for hand trajectory server...')
        self._hand_ac.wait_for_server()
        self.get_logger().info('Controllers ready.')

        self._demo_thread = threading.Thread(target=self._run_demo, daemon=True)
        self._demo_thread.start()

    # ───────────────────────── ROS helpers ───────────────────────── #

    def _js_cb(self, msg):
        if self._joint_map is None:
            self._joint_map = {n: i for i, n in enumerate(msg.name)}

        idx = [self._joint_map[n] for n in ARM_JOINTS if n in self._joint_map]
        if len(idx) == 6:
            self.joint_pos = np.array([msg.position[i] for i in idx], dtype=float)

    def _send_traj(self, ac, names, positions_list, times):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)

        for q, t in zip(positions_list, times):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            pt.velocities = [0.0] * len(names)

            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            goal.trajectory.points.append(pt)

        future = ac.send_goal_async(goal)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        if not future.done():
            self.get_logger().warn('Goal send timeout.')
            return False

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Trajectory goal rejected.')
            return False

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + max(times[-1], 1.0) + 10.0
        while not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        return result_future.done()

    def _send_arm(self, q, dur=2.0):
        return self._send_traj(self._arm_ac, ARM_JOINTS, [q], [dur])

    def _send_arm_multi(self, waypoints):
        if not waypoints:
            return True
        qs, ts = zip(*waypoints)
        return self._send_traj(self._arm_ac, ARM_JOINTS, qs, ts)

    def _send_hand(self, qh, dur=1.0):
        return self._send_traj(self._hand_ac, HAND_JOINTS, [qh], [dur])

    def _wait_joint_state(self):
        while self.joint_pos is None and rclpy.ok():
            time.sleep(0.1)

    def _current_q(self):
        if self.joint_pos is None:
            return np.zeros(6, dtype=float)
        return np.array(self.joint_pos, dtype=float)

    def _w2b(self, world_pt):
        # Robot was spawned with translation only, so world->base is just subtraction.
        return np.asarray(world_pt, dtype=float) - self.BASE_WORLD

    # ───────────────────────── Motion primitives ───────────────────────── #

    def _solve_pose(self, q_seed, p_des, R_des=None, dur=1.5):
        if R_des is None:
            R_des = self.R_GRASP
        q_cmd = solve_ik_pose(q_seed, p_des, R_des)
        self._send_arm(q_cmd, dur)
        return q_cmd

    def _move_linear_pose(self, q_start, p_target, R_des=None, total_time=1.2, steps=8):
        if R_des is None:
            R_des = self.R_GRASP

        q = np.array(q_start, dtype=float)
        p0, _ = tool_pose(q)
        pts = linear_path(p0, p_target, steps)

        waypoints = []
        t = 0.0
        dt = total_time / max(steps, 1)

        for p in pts:
            q = solve_ik_pose(q, p, R_des, max_iter=140)
            t += dt
            waypoints.append((np.array(q), t))

        self._send_arm_multi(waypoints)
        return q

    def _close_grasp(self):
        self._send_hand(self.HAND_SOFT_PINCH, 0.8)
        time.sleep(0.3)
        self._send_hand(self.HAND_FIRM_PINCH, 1.0)
        time.sleep(0.4)

    # ───────────────────────── Task logic ───────────────────────── #

    def _pick_and_insert(
        self,
        peg_world,
        hole_world,
        grasp_height,
        insertion_depth,
        name='peg'
    ):
        peg_b = self._w2b(peg_world)
        hole_b = self._w2b(hole_world)

        q = self._current_q()

        grasp_center = peg_b + np.array([0.0, 0.0, grasp_height], dtype=float)
        pregrasp = grasp_center + np.array([0.0, self.PREGRASP_Y_OFFSET, 0.0], dtype=float)
        final_grasp = grasp_center + np.array([0.0, self.FINAL_GRASP_Y_OFFSET, 0.0], dtype=float)
        above_pregrasp = pregrasp + np.array([0.0, 0.0, self.APPROACH_Z], dtype=float)
        lifted = final_grasp + np.array([0.0, 0.0, self.LIFT_DELTA_Z], dtype=float)

        hole_top = hole_b + np.array([0.0, 0.0, self.APPROACH_Z], dtype=float)

        self.get_logger().info(f'Starting {name} peg pick.')
        self._send_hand(self.HAND_OPEN, 0.8)
        self._send_hand(self.HAND_PREGRASP, 0.8)

        q = self._solve_pose(q, above_pregrasp, dur=2.0)
        q = self._move_linear_pose(q, pregrasp, total_time=1.0, steps=7)
        q = self._move_linear_pose(q, final_grasp, total_time=0.9, steps=7)

        self._close_grasp()

        q = self._move_linear_pose(q, lifted, total_time=1.1, steps=7)
        self.get_logger().info(f'{name} peg lifted.')

        q = self._move_linear_pose(q, hole_top, total_time=2.0, steps=12)
        q = self._compliance_insert(q, hole_b, insertion_depth)

        self._send_hand(self.HAND_OPEN, 0.8)
        retract = hole_b + np.array([0.0, 0.0, self.RETRACT_Z], dtype=float)
        q = self._move_linear_pose(q, retract, total_time=1.1, steps=8)

        return q

    def _compliance_insert(self, q_start, hole_b, depth):
        q = np.array(q_start, dtype=float)
        dt = self.INSERT_DT
        p_prev, _ = tool_pose(q)

        z_top = hole_b[2] + self.APPROACH_Z
        z_bottom = hole_b[2] - depth

        z_des = z_top
        theta = 0.0
        time_from_start = 0.0
        waypoints = []

        while z_des > z_bottom:
            p_now, _ = tool_pose(q)
            v = (p_now - p_prev) / dt

            progress = max(0.0, min(1.0, (z_top - z_des) / max(z_top - z_bottom, 1e-6)))
            r = min(self.SEARCH_RADIUS_MAX, self.SEARCH_GROWTH * (1.0 + 35.0 * progress))
            theta += self.SEARCH_OMEGA

            search_xy = np.array([
                r * math.cos(theta),
                r * math.sin(theta),
            ], dtype=float)

            z_des = max(z_bottom, z_des - self.INSERT_SPEED * dt)

            e_xy = p_now[:2] - (hole_b[:2] + search_xy)
            e_z = p_now[2] - z_des

            corr_xy = -self.INSERT_XY_K * e_xy - self.INSERT_XY_D * v[:2]
            corr_z = -self.INSERT_Z_K * e_z - self.INSERT_Z_D * v[2]

            p_cmd = np.array([
                hole_b[0] + search_xy[0] + corr_xy[0],
                hole_b[1] + search_xy[1] + corr_xy[1],
                z_des + corr_z,
            ], dtype=float)

            q = solve_ik_pose(q, p_cmd, self.R_GRASP, max_iter=90, damping=0.10, step_limit=0.12)
            time_from_start += dt
            waypoints.append((np.array(q), time_from_start))
            p_prev = p_now.copy()

        time_from_start += 0.4
        waypoints.append((np.array(q), time_from_start))
        self._send_arm_multi(waypoints)
        return q

    # ───────────────────────── Demo sequence ───────────────────────── #

    def _run_demo(self):
        self._wait_joint_state()
        if not rclpy.ok():
            return

        q_home = np.array([
            math.pi / 2.0,
            -math.pi / 2.0,
            math.pi / 2.0,
            -math.pi / 2.0,
            -math.pi / 2.0,
            0.0,
        ], dtype=float)

        self._send_arm(q_home, 2.5)
        self._send_hand(self.HAND_OPEN, 1.0)
        time.sleep(0.5)

        try:
            q = self._pick_and_insert(
                peg_world=self.SQ_PEG_WORLD,
                hole_world=self.SQ_HOLE_WORLD,
                grasp_height=self.SQUARE_GRASP_HEIGHT,
                insertion_depth=0.070,
                name='square'
            )

            self._send_arm(q_home, 2.0)
            time.sleep(0.8)

            q = self._pick_and_insert(
                peg_world=self.RND_PEG_WORLD,
                hole_world=self.RND_HOLE_WORLD,
                grasp_height=self.ROUND_GRASP_HEIGHT,
                insertion_depth=0.055,
                name='round'
            )

            self._send_arm(q_home, 2.0)

        except Exception as exc:
            self.get_logger().error(f'Demo failed: {exc}')


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