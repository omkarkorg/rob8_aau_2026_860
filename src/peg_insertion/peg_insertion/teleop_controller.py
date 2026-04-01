#!/usr/bin/env python3
"""
Interactive keyboard teleop for UR5e arm + Shadow Hand.

ARM mode  (press 'a'):
  1-6        select joint
  UP / DOWN  move ±step rad
  0          go to home pose

HAND mode (press 'h'):
  f/m/r/l/t  select finger  (index/middle/ring/little/thumb)
  1-5        select joint within the active finger
  UP / DOWN  move ±step rad
  o          open all fingers
  c          close all fingers (power grasp)
  g          grasp pose (pinch-ready)

Both modes:
  [ / ]      decrease / increase step size
  q          quit
"""

import curses
import threading

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ── Arm ───────────────────────────────────────────────────────────────────────

ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]
ARM_LABELS = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
ARM_HOME   = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
ARM_LIMITS = [
    (-6.28,  6.28),
    (-6.28,  6.28),
    (-3.14,  3.14),
    (-6.28,  6.28),
    (-6.28,  6.28),
    (-6.28,  6.28),
]

# ── Hand ──────────────────────────────────────────────────────────────────────

FINGER_ORDER = ['FF', 'MF', 'RF', 'LF', 'TH']
FINGER_NAMES = {
    'FF': 'INDEX',
    'MF': 'MIDDLE',
    'RF': 'RING',
    'LF': 'LITTLE',
    'TH': 'THUMB',
}
FINGER_KEY = {'f': 0, 'm': 1, 'r': 2, 'l': 3, 't': 4}

FINGER_JOINTS = {
    'FF': {
        'joints': ['FFJ4', 'FFJ3', 'FFJ2', 'FFJ1'],
        'limits': [(-0.436, 0.436), (0.0, 1.571), (0.0, 1.571), (0.0, 1.571)],
        'labels': ['J4(spread)', 'J3(MCP)', 'J2(PIP)', 'J1(DIP)'],
    },
    'MF': {
        'joints': ['MFJ4', 'MFJ3', 'MFJ2', 'MFJ1'],
        'limits': [(-0.436, 0.436), (0.0, 1.571), (0.0, 1.571), (0.0, 1.571)],
        'labels': ['J4(spread)', 'J3(MCP)', 'J2(PIP)', 'J1(DIP)'],
    },
    'RF': {
        'joints': ['RFJ4', 'RFJ3', 'RFJ2', 'RFJ1'],
        'limits': [(-0.436, 0.436), (0.0, 1.571), (0.0, 1.571), (0.0, 1.571)],
        'labels': ['J4(spread)', 'J3(MCP)', 'J2(PIP)', 'J1(DIP)'],
    },
    'LF': {
        'joints': ['LFJ5', 'LFJ4', 'LFJ3', 'LFJ2', 'LFJ1'],
        'limits': [(0.0, 0.698), (-0.436, 0.436), (0.0, 1.571), (0.0, 1.571), (0.0, 1.571)],
        'labels': ['J5(meta)', 'J4(spread)', 'J3(MCP)', 'J2(PIP)', 'J1(DIP)'],
    },
    'TH': {
        'joints': ['THJ5', 'THJ4', 'THJ3', 'THJ2', 'THJ1'],
        'limits': [(-1.047, 1.047), (0.0, 1.309), (-0.262, 0.262), (-0.524, 0.524), (0.0, 1.571)],
        'labels': ['J5(rot)', 'J4(abduct)', 'J3(flex1)', 'J2(flex2)', 'J1(tip)'],
    },
}

ALL_HAND_JOINTS: list[str] = []
for _fd in FINGER_JOINTS.values():
    ALL_HAND_JOINTS.extend(_fd['joints'])


# ── ROS 2 node ────────────────────────────────────────────────────────────────

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_controller')

        self.arm_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.hand_pub = self.create_publisher(
            JointTrajectory, '/hand_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        # Mutable state (written by curses thread, read by timer)
        self.lock = threading.Lock()
        self.arm_targets  = list(ARM_HOME)
        self.hand_targets = {j: 0.0 for j in ALL_HAND_JOINTS}
        self.current_pos: dict[str, float] = {}

        self.create_timer(0.05, self._publish)   # 20 Hz publish

    def _js_cb(self, msg: JointState):
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                self.current_pos[name] = pos

    def _publish(self):
        now_ns = 150_000_000   # 0.15 s move time

        with self.lock:
            arm_tgt  = list(self.arm_targets)
            hand_tgt = [self.hand_targets[j] for j in ALL_HAND_JOINTS]

        arm_msg = JointTrajectory()
        arm_msg.joint_names = ARM_JOINTS
        arm_pt = JointTrajectoryPoint()
        arm_pt.positions = arm_tgt
        arm_pt.time_from_start = Duration(sec=0, nanosec=now_ns)
        arm_msg.points = [arm_pt]
        self.arm_pub.publish(arm_msg)

        hand_msg = JointTrajectory()
        hand_msg.joint_names = ALL_HAND_JOINTS
        hand_pt = JointTrajectoryPoint()
        hand_pt.positions = hand_tgt
        hand_pt.time_from_start = Duration(sec=0, nanosec=now_ns)
        hand_msg.points = [hand_pt]
        self.hand_pub.publish(hand_msg)


# ── Curses UI ─────────────────────────────────────────────────────────────────

def _clamp(val, lo, hi):
    return max(lo, min(hi, val))


def _put(stdscr, row, col, text, attr=0):
    try:
        stdscr.addstr(row, col, text, attr)
    except curses.error:
        pass


def run_ui(stdscr, node: TeleopNode):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(80)

    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_CYAN,   -1)   # headers
    curses.init_pair(2, curses.COLOR_YELLOW, -1)   # active section
    curses.init_pair(3, curses.COLOR_GREEN,  -1)   # selected item
    curses.init_pair(4, curses.COLOR_WHITE,  -1)   # normal

    C_HDR  = curses.color_pair(1) | curses.A_BOLD
    C_ACT  = curses.color_pair(2)
    C_SEL  = curses.color_pair(3) | curses.A_BOLD
    C_NRM  = curses.color_pair(4)

    mode       = 'ARM'
    arm_idx    = 0        # 0-5 selected arm joint
    finger_idx = 0        # 0-4 selected finger
    joint_idx  = 0        # joint within selected finger
    step       = 0.05     # rad per keypress

    while True:
        key = stdscr.getch()

        # ── Global keys ───────────────────────────────────────────────────────
        if key == ord('q'):
            break
        elif key == ord('a'):
            mode = 'ARM'
        elif key == ord('h'):
            mode = 'HAND'
        elif key == ord('['):
            step = max(0.005, round(step - 0.005, 3))
        elif key == ord(']'):
            step = min(0.5, round(step + 0.005, 3))

        # ── ARM mode keys ─────────────────────────────────────────────────────
        if mode == 'ARM':
            for i in range(6):
                if key == ord(str(i + 1)):
                    arm_idx = i

            with node.lock:
                if key == curses.KEY_UP:
                    lo, hi = ARM_LIMITS[arm_idx]
                    node.arm_targets[arm_idx] = _clamp(
                        node.arm_targets[arm_idx] + step, lo, hi)
                elif key == curses.KEY_DOWN:
                    lo, hi = ARM_LIMITS[arm_idx]
                    node.arm_targets[arm_idx] = _clamp(
                        node.arm_targets[arm_idx] - step, lo, hi)
                elif key == ord('0'):
                    node.arm_targets = list(ARM_HOME)

        # ── HAND mode keys ────────────────────────────────────────────────────
        elif mode == 'HAND':
            if key != -1 and chr(key) in FINGER_KEY:
                finger_idx = FINGER_KEY[chr(key)]
                joint_idx  = 0

            cur_finger = FINGER_ORDER[finger_idx]
            cur_data   = FINGER_JOINTS[cur_finger]
            n_joints   = len(cur_data['joints'])

            for i in range(n_joints):
                if key == ord(str(i + 1)):
                    joint_idx = i

            cur_joint = cur_data['joints'][joint_idx]
            lo, hi    = cur_data['limits'][joint_idx]

            with node.lock:
                if key == curses.KEY_UP:
                    node.hand_targets[cur_joint] = _clamp(
                        node.hand_targets[cur_joint] + step, lo, hi)
                elif key == curses.KEY_DOWN:
                    node.hand_targets[cur_joint] = _clamp(
                        node.hand_targets[cur_joint] - step, lo, hi)
                elif key == ord('o'):   # open all
                    for j in ALL_HAND_JOINTS:
                        node.hand_targets[j] = 0.0
                elif key == ord('c'):   # close (power grasp)
                    for fname in FINGER_ORDER:
                        fd = FINGER_JOINTS[fname]
                        for ji, jn in enumerate(fd['joints']):
                            jlo, jhi = fd['limits'][ji]
                            # spread joints → neutral, flex joints → ~80 % max
                            if jlo < 0:
                                node.hand_targets[jn] = 0.0
                            else:
                                node.hand_targets[jn] = jhi * 0.8
                elif key == ord('g'):   # grasp (pinch) pose
                    for fname in ['FF', 'MF', 'RF', 'LF']:
                        fd = FINGER_JOINTS[fname]
                        for ji, jn in enumerate(fd['joints']):
                            jlo, jhi = fd['limits'][ji]
                            node.hand_targets[jn] = 0.0 if jlo < 0 else jhi * 0.7
                    node.hand_targets['THJ5'] =  0.8
                    node.hand_targets['THJ4'] =  1.0
                    node.hand_targets['THJ3'] =  0.0
                    node.hand_targets['THJ2'] =  0.0
                    node.hand_targets['THJ1'] =  0.9

        # ── Snapshot state for rendering (no lock held during draw) ───────────
        with node.lock:
            arm_tgts   = list(node.arm_targets)
            hand_tgts  = dict(node.hand_targets)
            cur_pos    = dict(node.current_pos)

        # ── Draw ──────────────────────────────────────────────────────────────
        stdscr.erase()
        row = 0

        _put(stdscr, row, 0, '═' * 68, C_HDR); row += 1
        _put(stdscr, row, 0, '  UR5e + Shadow Hand  Teleop Controller', C_HDR); row += 1
        _put(stdscr, row, 0, '═' * 68, C_HDR); row += 1

        arm_attr  = C_SEL if mode == 'ARM'  else C_NRM
        hand_attr = C_SEL if mode == 'HAND' else C_NRM
        _put(stdscr, row, 0,  '  Mode: ')
        _put(stdscr, row, 8,  '[ARM]',  arm_attr)
        _put(stdscr, row, 14, '  /  ')
        _put(stdscr, row, 19, '[HAND]', hand_attr)
        _put(stdscr, row, 26, '    a=ARM  h=HAND  q=quit  [/]=step')
        row += 2

        # ── ARM section ───────────────────────────────────────────────────────
        sec_attr = C_ACT if mode == 'ARM' else C_NRM
        _put(stdscr, row, 0,
             '── ARM  (1-6 select joint │ UP/DOWN move │ 0=home) ──────────',
             sec_attr); row += 1

        for i, lbl in enumerate(ARM_LABELS):
            cur = cur_pos.get(ARM_JOINTS[i], arm_tgts[i])
            tgt = arm_tgts[i]
            marker = '►' if (mode == 'ARM' and i == arm_idx) else ' '
            attr   = C_SEL if (mode == 'ARM' and i == arm_idx) else C_NRM
            _put(stdscr, row, 0,
                 f'  {marker}[{i+1}] {lbl:<15}  cur:{cur:+7.3f}  tgt:{tgt:+7.3f}',
                 attr)
            row += 1

        _put(stdscr, row, 0,
             f'     step: {step:.3f} rad  │  [ decrease   ] increase', C_ACT)
        row += 2

        # ── HAND section ──────────────────────────────────────────────────────
        sec_attr = C_ACT if mode == 'HAND' else C_NRM
        _put(stdscr, row, 0,
             '── HAND  (f/m/r/l/t finger │ 1-5 joint │ UP/DOWN move) ─────',
             sec_attr); row += 1
        _put(stdscr, row, 0,
             '     o=open  c=close(power)  g=grasp(pinch)', C_ACT); row += 1

        for fi, fname in enumerate(FINGER_ORDER):
            fd = FINGER_JOINTS[fname]
            is_af = (mode == 'HAND' and fi == finger_idx)
            fattr = C_SEL if is_af else C_NRM

            # Finger header line
            key_char = list(FINGER_KEY.keys())[fi]
            hdr = f"  [{key_char}] {FINGER_NAMES[fname]:<7} ({fname}):"
            _put(stdscr, row, 0, hdr, fattr); row += 1

            # One line per joint
            for ji, (jn, lbl) in enumerate(zip(fd['joints'], fd['labels'])):
                tgt = hand_tgts[jn]
                cur = cur_pos.get(jn, tgt)
                is_aj = is_af and ji == joint_idx
                marker = '►' if is_aj else ' '
                jattr  = C_SEL if is_aj else (C_ACT if is_af else C_NRM)
                lo, hi = fd['limits'][ji]
                bar_w  = 12
                frac   = (tgt - lo) / (hi - lo) if hi > lo else 0.0
                filled = int(frac * bar_w)
                bar    = '█' * filled + '░' * (bar_w - filled)
                _put(stdscr, row, 0,
                     f'    {marker}[{ji+1}] {lbl:<12}  cur:{cur:+6.3f}  tgt:{tgt:+6.3f}  [{bar}]',
                     jattr)
                row += 1

            row += 1  # gap between fingers

        stdscr.refresh()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = TeleopNode()

    # Spin ROS 2 in a background thread so curses owns the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        curses.wrapper(run_ui, node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
