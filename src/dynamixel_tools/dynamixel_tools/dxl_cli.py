#!/usr/bin/env python3
"""Command-line front end for the dynamixel_hardware services.

Requires a running ros2_control stack -- it talks to the plugin's services,
it does not open the serial port itself. For rescue work while the stack is
stopped, use the offline tool instead.

    ros2 run dynamixel_tools dxl_cli dump 11
    ros2 run dynamixel_tools dxl_cli torque all off
    ros2 run dynamixel_tools dxl_cli reboot 13
    ros2 run dynamixel_tools dxl_cli read 11 146 1
    ros2 run dynamixel_tools dxl_cli write 11 led 1
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from dynamixel_msgs.srv import ReadRegister, Reboot, SetTorque, SetZero, WriteRegister

from dynamixel_tools.registers import BY_NAME, REGISTERS

SERVICE_TYPES = {
    'set_torque': SetTorque,
    'reboot': Reboot,
    'set_zero': SetZero,
    'read_register': ReadRegister,
    'write_register': WriteRegister,
}


class DxlCli(Node):

    def __init__(self, component):
        super().__init__('dxl_cli')
        self.component = component or self._discover_component()
        if self.component is None:
            self.get_logger().error(
                'No dynamixel_hardware services found. Is the ros2_control stack running?')
            sys.exit(1)
        self._svc = {}

    def _discover_component(self):
        """Find the hardware component namespace by looking for set_torque."""
        deadline = time.time() + 5.0
        quiet = time.time() + 1.0
        found = []
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            found = sorted(
                name.rsplit('/', 1)[0]
                for name, types in self.get_service_names_and_types()
                if name.endswith('/set_torque')
                and 'dynamixel_msgs/srv/SetTorque' in types)
            if found and time.time() >= quiet:
                break
        if found and len(found) > 1:
            print(f'multiple components found: {found}\n'
                  f'using {found[0]}, pass --component to pick another',
                  file=sys.stderr)
        return found[0] if found else None

    def call(self, service, request, timeout=5.0):
        if service not in self._svc:
            self._svc[service] = self.create_client(
                SERVICE_TYPES[service], f'{self.component}/{service}')
        client = self._svc[service]
        if not client.wait_for_service(timeout_sec=timeout):
            print(f'service {self.component}/{service} unavailable', file=sys.stderr)
            sys.exit(1)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            print('service call timed out', file=sys.stderr)
            sys.exit(1)
        return future.result()


def parse_id(text):
    return 0 if text in ('all', '0') else int(text)


def parse_onoff(text):
    if text in ('on', '1', 'true'):
        return True
    if text in ('off', '0', 'false'):
        return False
    raise argparse.ArgumentTypeError(f'expected on/off, got {text}')


def resolve_target(text):
    """Accept either a raw address or a friendly register name."""
    if text in BY_NAME:
        return BY_NAME[text]
    return int(text), None


def cmd_dump(cli, args):
    """Read every known register of one motor -- the READ_REG table dump."""
    print(f'{"name":<24}{"addr":>5}{"size":>5}{"value":>12}  note')
    print('-' * 70)
    for name, addr, size, _signed, note in REGISTERS:
        req = ReadRegister.Request(id=args.id, address=addr, size=size)
        res = cli.call('read_register', req)
        value = res.value if res.success else 'ERR'
        print(f'{name:<24}{addr:>5}{size:>5}{value:>12}  {note}')


def cmd_read(cli, args):
    addr, default_size = resolve_target(args.target)
    size = args.size if args.size else (default_size or 1)
    res = cli.call('read_register', ReadRegister.Request(id=args.id, address=addr, size=size))
    if res.success:
        print(res.value)
    else:
        print(res.message, file=sys.stderr)
        sys.exit(1)


def cmd_write(cli, args):
    addr, default_size = resolve_target(args.target)
    size = args.size if args.size else (default_size or 1)
    res = cli.call(
        'write_register',
        WriteRegister.Request(id=args.id, address=addr, size=size, value=args.value))
    print('OK' if res.success else res.message)
    sys.exit(0 if res.success else 1)


def cmd_torque(cli, args):
    res = cli.call('set_torque', SetTorque.Request(id=args.id, enable=args.state))
    print(res.message if res.success else res.message, file=sys.stderr if not res.success else sys.stdout)
    sys.exit(0 if res.success else 1)


def cmd_reboot(cli, args):
    res = cli.call('reboot', Reboot.Request(id=args.id))
    print(res.message if res.success else res.message)
    sys.exit(0 if res.success else 1)


def cmd_zero(cli, args):
    res = cli.call('set_zero', SetZero.Request(id=args.id))
    print(res.message if res.success else res.message)
    sys.exit(0 if res.success else 1)


def build_parser():
    p = argparse.ArgumentParser(prog='dxl_cli', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('-c', '--component', default=None,
                   help='hardware component namespace (auto-detected if omitted)')
    sub = p.add_subparsers(dest='cmd', required=True)

    s = sub.add_parser('dump', help='print the full register table of one motor')
    s.add_argument('id', type=int)
    s.set_defaults(func=cmd_dump)

    s = sub.add_parser('read', help='read a register (address or name)')
    s.add_argument('id', type=int)
    s.add_argument('target')
    s.add_argument('size', nargs='?', type=int, default=None)
    s.set_defaults(func=cmd_read)

    s = sub.add_parser('write', help='write a register (address or name)')
    s.add_argument('id', type=parse_id)
    s.add_argument('target')
    s.add_argument('value', type=int)
    s.add_argument('size', nargs='?', type=int, default=None)
    s.set_defaults(func=cmd_write)

    s = sub.add_parser('torque', help='enable/disable torque')
    s.add_argument('id', type=parse_id)
    s.add_argument('state', type=parse_onoff)
    s.set_defaults(func=cmd_torque)

    s = sub.add_parser('reboot', help='clear hardware error')
    s.add_argument('id', type=parse_id)
    s.set_defaults(func=cmd_reboot)

    s = sub.add_parser('zero', help='write current position as the new zero')
    s.add_argument('id', type=parse_id)
    s.set_defaults(func=cmd_zero)

    return p


def main():
    args = build_parser().parse_args()
    rclpy.init()
    cli = DxlCli(args.component)
    try:
        args.func(cli, args)
    finally:
        cli.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
