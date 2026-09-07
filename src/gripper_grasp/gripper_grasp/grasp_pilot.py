#!/usr/bin/env python3
"""Pick a grasp by eye, let the sensors pick the moment.

You choose which way the fingers should be arranged; the TOF decides when the
object is in the right place and the hand closes on it. Contact is read from the
per-joint stall flags, not from the range: past about 11% closure the TOF is
looking at the fingers rather than the scene, so the two never overlap.

    ros2 run gripper_grasp grasp_pilot
    ros2 run gripper_grasp grasp_pilot --ros-args -p trigger_centre:=0.06
"""

import curses
import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from gripper_msgs.msg import GripperGoal, GripperState
from sensor_msgs.msg import Image, Range

from gripper_grasp.alignment import CentreAlignment
from gripper_grasp.grasps import BY_KEY, GRASPS, JOINTS

IDLE, PRESHAPE, ARMED, CLOSING, HOLDING = 'IDLE', 'PRESHAPE', 'ARMED', 'CLOSING', 'HOLDING'
STATUS_NAME = {0: 'IDLE', 1: 'TRACKING', 2: 'HOLDING', 3: 'BLOCKED'}
SERVO_HOLDING = 2
FLEXION = range(6)


class GraspPilot(Node):

    def __init__(self):
        super().__init__('grasp_pilot')
        p = self.declare_parameter
        self._centre = float(p('trigger_centre', 0.05).value)
        self._window = float(p('trigger_window', 0.03).value)
        self._hold_samples = int(p('trigger_samples', 8).value)
        self._stall_needed = int(p('stall_joints', 3).value)
        self._approach_current = float(p('approach_current', 200.0).value)
        self._grip_current = float(p('grip_current', 350.0).value)
        self._close_seconds = float(p('close_seconds', 4.0).value)
        # One motor velocity unit is 0.024 rad/s; below that never trips.
        self._settle_speed = float(p('settle_speed', 0.05).value)

        self._align = CentreAlignment()
        self._align_enabled = True
        self._align_score = 0.0
        self._align_ok = False

        self._range = None
        self._state = None
        self._in_window = 0

        self.grasp = GRASPS[0]
        self.phase = PRESHAPE
        self.closure = 0.0
        self.message = ''

        self._goal = self.create_publisher(GripperGoal, '/gripper_controller/goal', 10)
        self.create_subscription(Range, '/tof/range', self._on_range, 10)
        self.create_subscription(Image, '/palm_camera/image_raw', self._on_image, 1)
        self.create_subscription(GripperState, '/gripper_controller/state',
                                 self._on_state, 10)

        self._period = 0.05
        self.create_timer(self._period, self._tick)

    # ---- subscriptions -------------------------------------------------

    def _on_range(self, msg):
        self._range = msg.range

    def _on_image(self, msg):
        self._align_ok, self._align_score = self._align.check(msg)

    def _on_state(self, msg):
        self._state = msg

    # ---- helpers -------------------------------------------------------

    @property
    def range_ok(self):
        r = self._range
        return (r is not None and r == r and r < 90.0
                and abs(r - self._centre) <= self._window)

    @property
    def aligned(self):
        return self._align_ok or not self._align_enabled

    def settled(self):
        """Reached the preshape: the servo says HOLDING and nothing is moving."""
        s = self._state
        if s is None:
            return False
        return (s.status == SERVO_HOLDING and
                max(abs(v) for v in s.velocity) < self._settle_speed)

    def stalled_count(self):
        s = self._state
        return 0 if s is None else sum(1 for i in FLEXION if s.stalled[i])

    def _publish(self, closure, current):
        goal = GripperGoal()
        pose = self.grasp.pose(closure)
        for i in range(JOINTS):
            goal.position[i] = pose[i]
            goal.current_limit[i] = current
        goal.max_velocity = 0.3
        goal.max_acceleration = 1.5
        self._goal.publish(goal)

    # ---- state machine -------------------------------------------------

    def _tick(self):
        # Publish every cycle whatever the phase: the servo's watchdog holds
        # position after half a second of silence, and a hand that keeps
        # re-deciding is easier to reason about than one that goes quiet.
        if self.phase in (PRESHAPE, ARMED):
            self._publish(0.0, self._approach_current)
            if self.phase == ARMED:
                self._check_trigger()
            return

        if self.phase == CLOSING:
            self.closure = min(1.0, self.closure + self._period / self._close_seconds)
            self._publish(self.closure, self._grip_current)
            self._check_contact()
            return

        self._publish(self.closure, self._grip_current)

    def _check_trigger(self):
        self._in_window = self._in_window + 1 if self.range_ok else 0
        if self._in_window >= self._hold_samples and self.aligned:
            self.phase = CLOSING
            self.closure = 0.0
            self._in_window = 0
            self.message = f'triggered at {self._range:.3f} m'

    def _check_contact(self):
        stalled = self.stalled_count()
        blocked = self._state is not None and self._state.status == 3
        if stalled >= self._stall_needed:
            self.phase = HOLDING
            self.message = f'holding, {stalled} joints in contact'
        elif blocked:
            # The reach table should have kept us clear of this.
            self.phase = HOLDING
            self.message = 'stopped: self-collision limit, check the reach table'
        elif self.closure >= 1.0 and self.settled():
            self.phase = HOLDING
            self.message = f'closed fully, {stalled} joints in contact'

    # ---- commands ------------------------------------------------------

    def select(self, key):
        if key in BY_KEY and self.phase in (IDLE, PRESHAPE, ARMED):
            self.grasp = BY_KEY[key]
            self.phase = PRESHAPE
            self.message = f'{self.grasp.name}: {self.grasp.note}'

    def toggle_arm(self):
        if self.phase == ARMED:
            self.phase = PRESHAPE
            self._in_window = 0
            self.message = 'disarmed'
        elif self.phase == PRESHAPE:
            if not self.settled():
                self.message = 'still moving into the preshape'
                return
            self.phase = ARMED
            self.message = 'armed, waiting for the object'

    def release(self):
        self.phase = PRESHAPE
        self.closure = 0.0
        self._in_window = 0
        self.message = 'released'

    def toggle_alignment(self):
        self._align_enabled = not self._align_enabled
        self.message = f'alignment gate {"on" if self._align_enabled else "off"}'

    def calibrate(self):
        """Take the current reading as the middle of the trigger window.

        Beats deriving it: palm_tof's pose in the URDF is a guess, and the
        measured range disagrees with it by more than a factor of two.
        """
        r = self._range
        if r is None or r != r or r > 90.0:
            self.message = 'no valid range to calibrate against'
            return
        self._centre = r
        self.message = f'trigger centre set to {r:.3f} m'


def addnstr(win, y, x, text, n, attr=0):
    try:
        win.addnstr(y, x, text, n, attr)
    except curses.error:
        pass


def draw(stdscr, pilot):
    stdscr.erase()
    _, width = stdscr.getmaxyx()
    addnstr(stdscr, 0, 0, ' grasp pilot '.ljust(width), width, curses.A_REVERSE)

    row = 2
    for g in GRASPS:
        mark = '>' if g is pilot.grasp else ' '
        addnstr(stdscr, row, 0,
                f' {mark} {g.key}  {g.name:<8} spread {g.index_spread:+.0f}/'
                f'{g.middle_spread:+.0f}   closes to {g.closure_limit * 100:.0f}%'
                f'   {g.note}', width,
                curses.A_BOLD if g is pilot.grasp else 0)
        row += 1

    row += 1
    r = pilot._range
    shown = '--' if r is None or r != r or r > 90.0 else f'{r:.3f} m'
    gate = 'on' if pilot._align_enabled else 'off'
    addnstr(stdscr, row, 0,
            f'  phase {pilot.phase:<9} closure {pilot.closure * 100:5.1f}%'
            f'   range {shown:>8}'
            f' {"IN" if pilot.range_ok else "  "} window '
            f'{pilot._centre - pilot._window:.3f}-{pilot._centre + pilot._window:.3f}',
            width)
    row += 1
    addnstr(stdscr, row, 0,
            f'  alignment {gate:<3} score {pilot._align_score:4.2f} '
            f'{"ok" if pilot._align_ok else "no"}'
            f'   contacts {pilot.stalled_count()}/{len(FLEXION)}'
            f' need {pilot._stall_needed}', width)
    row += 2

    s = pilot._state
    if s is not None:
        names = ['C_2', 'C_3', 'A_2', 'A_3', 'B_2', 'B_3', 'A_1', 'B_1']
        addnstr(stdscr, row, 0, '  ' + ''.join(f'{n:>8}' for n in names), width)
        row += 1
        addnstr(stdscr, row, 0, '  ' + ''.join(f'{v:>8.3f}' for v in s.position),
                width)
        row += 1
        addnstr(stdscr, row, 0,
                '  ' + ''.join(f'{("C" if v else "-"):>8}' for v in s.stalled), width)
        row += 1
        addnstr(stdscr, row, 0,
                f'  servo {STATUS_NAME.get(s.status, "?")}'
                f'   reach {s.reach:.2f}'
                + ('   goal stale' if s.goal_stale else ''), width)
        row += 1

    row += 1
    addnstr(stdscr, row, 0,
            '  1/2/3 grasp   space arm   r release   a alignment gate   '
            'c calibrate   q quit', width)
    if pilot.message:
        addnstr(stdscr, row + 1, 0, '  ' + pilot.message, width, curses.A_BOLD)
    stdscr.refresh()


def run(stdscr, pilot):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)
    while True:
        draw(stdscr, pilot)
        key = stdscr.getch()
        if key in (ord('q'), 27):
            return
        if key in (ord('1'), ord('2'), ord('3')):
            pilot.select(chr(key))
        elif key == ord(' '):
            pilot.toggle_arm()
        elif key == ord('r'):
            pilot.release()
        elif key == ord('a'):
            pilot.toggle_alignment()
        elif key == ord('c'):
            pilot.calibrate()


def main():
    rclpy.init()
    pilot = GraspPilot()
    executor = SingleThreadedExecutor()
    executor.add_node(pilot)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        curses.wrapper(run, pilot)
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        executor.shutdown()
        pilot.destroy_node()
        spin.join(timeout=2.0)
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
