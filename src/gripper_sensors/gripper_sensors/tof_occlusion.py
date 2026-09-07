#!/usr/bin/env python3
"""Close the gripper slowly and record when the fingers start blocking the TOF.

A single-zone TOF reports one distance for its whole cone, dominated by the
strongest return, and a finger a few centimetres away beats anything further
out. Point the gripper at something at least a metre away, run this, and the
closure at which the reading collapses is where the sensor stops describing the
scene and starts describing the hand.

    ros2 run gripper_sensors tof_occlusion
    ros2 run gripper_sensors tof_occlusion --ros-args -p max_closure:=0.30
"""

import sys

import rclpy
from rclpy.node import Node

from gripper_msgs.msg import GripperGoal, GripperState
from sensor_msgs.msg import Range

# Flexion limits from the URDF, in the message's joint order. Zero is fully
# open and negative closes, so a closure fraction scales these.
FLEXION_LOWER = [-0.2217, -2.0944, -0.4992, -2.0944, -0.4992, -2.0944]


class TofOcclusion(Node):

    def __init__(self):
        super().__init__('tof_occlusion')
        self._spread = (
            float(self.declare_parameter('index_spread', 0.9076).value),
            float(self.declare_parameter('middle_spread', -0.9076).value),
        )
        self._max_closure = float(self.declare_parameter('max_closure', 0.35).value)
        self._seconds = float(self.declare_parameter('seconds', 25.0).value)
        self._current = float(self.declare_parameter('current_limit', 200.0).value)
        self._settle = float(self.declare_parameter('settle', 4.0).value)

        self._goal = self.create_publisher(GripperGoal, '/gripper_controller/goal', 10)
        self.create_subscription(Range, '/tof/range', self._on_range, 10)
        self.create_subscription(GripperState, '/gripper_controller/state',
                                 self._on_state, 10)

        self._range = None
        self._state = None
        self._rows = []
        self._closure = 0.0
        self._phase = 'spread'
        self._elapsed = 0.0
        self._period = 0.05
        self.create_timer(self._period, self._tick)

    def _on_range(self, msg):
        self._range = msg.range

    def _on_state(self, msg):
        self._state = msg

    def _publish(self, closure):
        goal = GripperGoal()
        for i in range(6):
            goal.position[i] = closure * FLEXION_LOWER[i]
        goal.position[6], goal.position[7] = self._spread
        for i in range(8):
            goal.current_limit[i] = self._current
        goal.max_velocity = 0.12
        goal.max_acceleration = 0.6
        self._goal.publish(goal)

    def _tick(self):
        self._elapsed += self._period

        if self._phase == 'spread':
            self._publish(0.0)
            if self._elapsed >= self._settle:
                self._phase = 'ramp'
                self._elapsed = 0.0
                print(f'{"closure":>8}{"thumb distal":>14}{"tof (m)":>10}'
                      f'{"status":>8}{"reach":>7}')
            return

        if self._phase == 'ramp':
            self._closure = min(self._max_closure,
                                self._max_closure * self._elapsed / self._seconds)
            self._publish(self._closure)
            if self._state is not None:
                self._rows.append((self._closure,
                                   self._state.position[1],
                                   self._range,
                                   self._state.status,
                                   self._state.reach))
                if len(self._rows) % 20 == 0:
                    self._report(self._rows[-1])
            if self._elapsed >= self._seconds + 3.0:
                self._phase = 'done'
            return

        self._publish(0.0)   # reopen, and keep the goal fresh while doing it

    def _report(self, row):
        closure, distal, distance, status, reach = row
        shown = 'inf' if distance is None or distance != distance or distance > 90 \
            else f'{distance:.3f}'
        print(f'{closure * 100:7.1f}%{distal:14.3f}{shown:>10}{status:8}{reach:7.2f}')

    def done(self):
        return self._phase == 'done'

    def summarise(self):
        print()
        if not self._rows:
            print('No samples. Is the stack running?')
            return

        finite = lambda r: r is not None and r == r and r < 90.0
        near = lambda r: finite(r) and r < 0.25

        # Occlusion looks different depending on what is out there. Aimed past
        # the sensor's range every sample starts as +inf and a finger makes it
        # finite; aimed at a wall the reading steps down instead. Watch for a
        # sustained close reading either way -- one sample is noise, a run of
        # them is a finger.
        run = 0
        for closure, _, distance, _, _ in self._rows:
            run = run + 1 if near(distance) else 0
            if run >= 10:
                print(f'The TOF starts reading the fingers at about '
                      f'{closure * 100:.0f}% closure.')
                print('Past that its distance describes the hand, not the scene.')
                break
        else:
            print(f'Nothing close appeared up to '
                  f'{self._rows[-1][0] * 100:.0f}% closure.')
            if not any(finite(r) for _, _, r, _, _ in self._rows):
                print('Every sample was +inf, so there was nothing in range to '
                      'occlude. Aim it at something a metre or two away.')

        blocked = [c for c, _, _, s, _ in self._rows if s == 3]
        if blocked:
            print(f'Self-collision limiting took over at '
                  f'{min(blocked) * 100:.0f}% closure.')


def main():
    rclpy.init()
    node = TofOcclusion()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        node.summarise()
        # Leave it open rather than parked wherever the ramp ended.
        for _ in range(60):
            rclpy.spin_once(node, timeout_sec=0.05)
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
