#!/usr/bin/env python3
"""Interactive check for the motors, through board_bridge.

Reads /joint_states and /dynamic_joint_states, publishes to the bridge's commands
topic, and toggles torque through its set_torque service. Requires a running stack.

    ros2 run dynamixel_tools dxl_debug
    ros2 run dynamixel_tools dxl_debug --check
    ros2 run dynamixel_tools dxl_debug --step 0.01
"""

import argparse
import curses
import sys
import time

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState
from dynamixel_msgs.srv import SetTorque
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class DxlDebug(Node):

    def __init__(self, args):
        super().__init__('dxl_debug')
        self._dxl_svc = {}
        self._state = None
        self._dyn = None

        self._controller = args.controller or self._discover_controller()
        if self._controller is None:
            print('No */commands topic found. Is the stack running?', file=sys.stderr)
            sys.exit(1)

        self._component = args.component or self._discover_component()

        ns = self._controller.rsplit('/', 1)[0] if '/' in self._controller else ''
        prefix = f'/{ns}' if ns else ''
        states = args.joint_states or f'{prefix}/joint_states'

        self.create_subscription(JointState, states, self._on_state, 10)
        self.create_subscription(
            DynamicJointState, f'{prefix}/dynamic_joint_states', self._on_dyn, 10)
        self._pub = self.create_publisher(
            Float64MultiArray, f'/{self._controller}/commands', 10)

        self._order = self._controller_joints()

    def _discover_controller(self):
        def probe():
            return sorted(
                name.strip('/').rsplit('/', 1)[0]
                for name, types in self.get_topic_names_and_types()
                if name.endswith('/commands')
                and 'std_msgs/msg/Float64MultiArray' in types)

        found = self._settle(probe)
        if found and len(found) > 1:
            print(f'multiple controllers found: {found}\n'
                  f'using {found[0]}, pass --controller to pick another',
                  file=sys.stderr)
        return found[0] if found else None

    def _settle(self, probe, settle=1.0, timeout=5.0):
        """Let discovery fill in before deciding.

        Returning on the first hit races the graph: with two stacks up, whichever
        one is seen first wins and the other is silently ignored.
        """
        deadline = time.time() + timeout
        quiet = time.time() + settle
        found = []
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            found = probe()
            if found and time.time() >= quiet:
                break
        return found

    def _discover_component(self):
        found = self._settle(lambda: sorted(
            name.rsplit('/', 1)[0]
            for name, types in self.get_service_names_and_types()
            if name.endswith('/set_torque')
            and 'dynamixel_msgs/srv/SetTorque' in types))
        return found[0] if found else None

    def _controller_joints(self):
        """Command array order comes from the controller's own joints parameter."""
        client = self.create_client(GetParameters, f'/{self._controller}/get_parameters')
        if not client.wait_for_service(timeout_sec=5.0):
            print(f'cannot reach /{self._controller}/get_parameters', file=sys.stderr)
            sys.exit(1)
        future = client.call_async(GetParameters.Request(names=['joints']))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return list(future.result().values[0].string_array_value)

    def _on_state(self, msg):
        self._state = dict(zip(msg.name, msg.position))

    def _on_dyn(self, msg):
        out = {}
        for name, iface in zip(msg.joint_names, msg.interface_values):
            out[name] = dict(zip(iface.interface_names, iface.values))
        self._dyn = out

    def wait_for_state(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            ready = self._state and all(j in self._state for j in self._order)
            if ready and self._dyn is not None:
                return True
        return bool(self._state)

    def spin(self):
        rclpy.spin_once(self, timeout_sec=0.0)

    def positions(self):
        return [self._state.get(j, 0.0) for j in self._order]

    def send(self, targets):
        self._pub.publish(Float64MultiArray(data=list(targets)))

    def _call(self, service, srv_type, request, timeout=3.0):
        if service not in self._dxl_svc:
            self._dxl_svc[service] = self.create_client(
                srv_type, f'{self._component}/{service}')
        client = self._dxl_svc[service]
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def set_torque(self, on):
        return self._call('set_torque', SetTorque, SetTorque.Request(id=0, enable=on))

    @property
    def all_names(self):
        """Commandable joints in the controller's own order, then the rest.

        The controller's joints parameter is written in whatever order the
        hardware is wired -- motor 1..n -- so following it puts the table in the
        order you think about the robot. joint_states is alphabetical, which is
        not that order.
        """
        if not self._state:
            return []
        ordered = [j for j in self._order if j in self._state]
        return ordered + sorted(set(self._state) - set(ordered))

    @property
    def order(self):
        return self._order

    @property
    def state(self):
        return self._state

    @property
    def dyn(self):
        return self._dyn

    @property
    def component(self):
        return self._component

    @property
    def controller(self):
        return self._controller


def format_table(dxl, targets, selected=None):
    lines = [f'{"":2}{"joint":<18}{"position":>10}{"target":>10}'
             f'{"temp":>7}{"volt":>7}{"err":>5}']
    state = dxl.state or {}
    for name in dxl.all_names:
        cmd_index = dxl.order.index(name) if name in dxl.order else None
        mark = '>' if cmd_index is not None and cmd_index == selected else ' '
        pos = state.get(name, float('nan'))
        tgt = targets[cmd_index] if (targets and cmd_index is not None) else float('nan')
        extra = (dxl.dyn or {}).get(name, {})
        lines.append(
            f'{mark:2}{name:<18}{pos:>10.4f}{tgt:>10.4f}'
            f'{extra.get("temperature", float("nan")):>7.0f}'
            f'{extra.get("voltage", float("nan")):>7.1f}'
            f'{extra.get("error", float("nan")):>5.0f}')
    return lines


def run_tui(stdscr, dxl, args):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)

    targets = dxl.positions()
    start_pose = list(targets)
    selected = 0
    step = args.step
    torque_on = True
    message = ''

    while True:
        dxl.spin()
        stdscr.erase()
        row = 0
        for line in format_table(dxl, targets, selected):
            stdscr.addnstr(row, 0, line, curses.COLS - 1)
            row += 1
        row += 1
        stdscr.addnstr(row, 0, f'controller={dxl.controller}  component={dxl.component}',
                       curses.COLS - 1)
        row += 1
        stdscr.addnstr(row, 0,
                       f'step={step:.3f} rad   torque={"ON" if torque_on else "OFF"}',
                       curses.COLS - 1)
        row += 1
        stdscr.addnstr(row, 0,
                       'up/down or 1-9 select   left/right jog   [ ] step   '
                       'h home   t torque   q quit', curses.COLS - 1)
        if message:
            row += 1
            stdscr.addnstr(row, 0, message, curses.COLS - 1)
        stdscr.refresh()

        c = stdscr.getch()
        if c == -1:
            continue
        if c in (ord('q'), 27):
            return
        elif c in (curses.KEY_UP, ord('p')):
            selected = (selected - 1) % len(dxl.order)
        elif c in (curses.KEY_DOWN, ord('n')):
            selected = (selected + 1) % len(dxl.order)
        elif ord('1') <= c <= ord('0') + min(9, len(dxl.order)):
            selected = c - ord('1')
        elif c == ord('['):
            step = max(args.step_min, step / 2)
        elif c == ord(']'):
            step = min(args.step_max, step * 2)
        elif c == ord('t'):
            if torque_on:
                res = dxl.set_torque(False)
                message = res.message if res else 'set_torque unavailable'
                torque_on = False
            else:
                targets = dxl.positions()
                dxl.send(targets)
                res = dxl.set_torque(True)
                message = res.message if res else 'set_torque unavailable'
                torque_on = True
        elif c in (curses.KEY_LEFT, curses.KEY_RIGHT, ord('h')):
            if not torque_on:
                message = 'torque is off'
                continue
            if c == ord('h'):
                targets = list(start_pose)
                message = 'returning to start pose'
            else:
                delta = step if c == curses.KEY_RIGHT else -step
                targets[selected] += delta
                message = f'{dxl.order[selected]} -> {targets[selected]:.4f}'
            dxl.send(targets)


def build_parser():
    p = argparse.ArgumentParser(prog='dxl_debug', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('-c', '--controller', default=None,
                   help='node offering the commands topic (auto-detected if omitted)')
    p.add_argument('--component', default=None,
                   help='namespace of the motor services (auto-detected if omitted)')
    p.add_argument('--joint-states', default=None,
                   help='joint states topic (derived from the controller if omitted)')
    p.add_argument('--check', action='store_true',
                   help='print one state table and exit')

    g = p.add_argument_group('motion tuning')
    g.add_argument('--step', type=float, default=0.02,
                   help='initial jog step in rad (default: %(default)s)')
    g.add_argument('--step-min', type=float, default=0.005,
                   help='lower bound for [ (default: %(default)s)')
    g.add_argument('--step-max', type=float, default=0.05,
                   help='upper bound for ] (default: %(default)s)')
    return p


def main():
    args = build_parser().parse_args()
    rclpy.init()
    dxl = DxlDebug(args)
    try:
        if not dxl.wait_for_state():
            print('no /joint_states received', file=sys.stderr)
            sys.exit(1)
        if args.check:
            for line in format_table(dxl, dxl.positions()):
                print(line)
        else:
            curses.wrapper(run_tui, dxl, args)
    finally:
        dxl.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
