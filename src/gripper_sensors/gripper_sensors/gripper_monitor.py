#!/usr/bin/env python3
"""Read-only view of the eight motors and the palm TOF.

Subscribes and nothing else -- no commands, no services, no controller
borrowing -- so it runs alongside dxl_debug or grasp_pilot without competing
with them for the command interfaces.

    ros2 run gripper_sensors gripper_monitor
"""

import curses
import threading
import time

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range

# Motor 1..8, the order the mechanism is documented and wired in.
JOINTS = [
    ('C_2_Joint', 'thumb  prox'),
    ('C_3_Joint', 'thumb  dist'),
    ('A_2_Joint', 'index  prox'),
    ('A_3_Joint', 'index  dist'),
    ('B_2_Joint', 'middle prox'),
    ('B_3_Joint', 'middle dist'),
    ('A_1_Joint', 'index  spread'),
    ('B_1_Joint', 'middle spread'),
]
STATUS_NAME = {0: 'IDLE', 1: 'TRACKING', 2: 'HOLDING', 3: 'BLOCKED'}
STALE_AFTER = 1.0        # s without a message before a source is called stale
FLEXION = 6

try:
    from gripper_msgs.msg import GripperState
except ImportError:                                          # pragma: no cover
    GripperState = None


class Monitor(Node):

    def __init__(self):
        super().__init__('gripper_monitor')
        self.joints = {}
        self.dyn = {}
        self.range = None
        self.grip = None
        self.seen = {}

        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.create_subscription(DynamicJointState, '/dynamic_joint_states',
                                 self._on_dyn, 10)
        self.create_subscription(Range, '/tof/range', self._on_range, 10)
        if GripperState is not None:
            self.create_subscription(GripperState, '/gripper_controller/state',
                                     self._on_grip, 10)

    def _mark(self, key):
        self.seen[key] = time.time()

    def age(self, key):
        t = self.seen.get(key)
        return None if t is None else time.time() - t

    def _on_joints(self, msg):
        for i, name in enumerate(msg.name):
            self.joints[name] = (
                msg.position[i] if i < len(msg.position) else float('nan'),
                msg.velocity[i] if i < len(msg.velocity) else float('nan'))
        self._mark('joints')

    def _on_dyn(self, msg):
        for name, iv in zip(msg.joint_names, msg.interface_values):
            self.dyn[name] = dict(zip(iv.interface_names, iv.values))
        self._mark('dyn')

    def _on_range(self, msg):
        self.range = msg.range
        self._mark('tof')

    def _on_grip(self, msg):
        self.grip = msg
        self._mark('grip')


def addnstr(win, y, x, text, n, attr=0):
    try:
        win.addnstr(y, x, text, n, attr)
    except curses.error:
        pass


def source(mon, key, label):
    age = mon.age(key)
    if age is None:
        return f'{label}=--'
    return f'{label}={"stale" if age > STALE_AFTER else "ok"}'


def draw(stdscr, mon):
    stdscr.erase()
    _, width = stdscr.getmaxyx()
    addnstr(stdscr, 0, 0, ' gripper monitor '.ljust(width), width, curses.A_REVERSE)

    row = 2
    addnstr(stdscr, row, 0,
            f'{"#":>2} {"joint":<12}{"":<14}{"position":>10}{"velocity":>9}'
            f'{"current":>9}{"temp":>6}{"volt":>6}{"err":>5}{"con":>5}', width)
    row += 1

    # Present Current keeps its sign: it is the torque direction, which a
    # stalled joint still reports when velocity has gone to zero. The stall
    # check and the ceiling both compare the magnitude.
    nan = float('nan')
    for i, (name, role) in enumerate(JOINTS):
        pos, vel = mon.joints.get(name, (nan, nan))
        extra = mon.dyn.get(name, {})
        err = extra.get('error', nan)
        con = ''
        if mon.grip is not None and i < FLEXION and mon.grip.stalled[i]:
            con = 'C'
        attr = curses.A_BOLD if (err == err and err) else 0
        addnstr(stdscr, row, 0,
                f'{i + 1:>2} {name:<12}{role:<14}{pos:>10.4f}{vel:>9.3f}'
                f'{extra.get("current", nan):>9.0f}'
                f'{extra.get("temperature", nan):>6.0f}'
                f'{extra.get("voltage", nan):>6.1f}'
                f'{err:>5.0f}{con:>5}', width, attr)
        row += 1

    row += 1
    r = mon.range
    shown = '--' if r is None or r != r or r > 90.0 else f'{r:.3f} m'
    addnstr(stdscr, row, 0, f'  tof   {shown}', width)
    row += 1

    g = mon.grip
    if g is None:
        addnstr(stdscr, row, 0, '  servo  no /gripper_controller/state', width)
    else:
        contacts = sum(g.stalled[:FLEXION])
        addnstr(stdscr, row, 0,
                f'  servo  {STATUS_NAME.get(g.status, "?"):<9}'
                f'reach {g.reach:5.3f}   contacts {contacts}/{FLEXION}'
                + ('   goal stale' if g.goal_stale else ''), width)
    row += 2

    addnstr(stdscr, row, 0,
            '  ' + '   '.join([source(mon, 'joints', 'joint_states'),
                               source(mon, 'dyn', 'dynamic'),
                               source(mon, 'tof', 'tof'),
                               source(mon, 'grip', 'servo')]), width)
    row += 1
    addnstr(stdscr, row, 0, '  read only, safe to run beside dxl_debug or '
                            'grasp_pilot   q quit', width)
    stdscr.refresh()


def run(stdscr, mon):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)
    while True:
        draw(stdscr, mon)
        if stdscr.getch() in (ord('q'), 27):
            return


def main():
    rclpy.init()
    mon = Monitor()
    executor = SingleThreadedExecutor()
    executor.add_node(mon)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        curses.wrapper(run, mon)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        mon.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    main()
