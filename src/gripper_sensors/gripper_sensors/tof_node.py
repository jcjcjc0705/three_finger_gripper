#!/usr/bin/env python3
"""Palm TOF driver. The sensor's MCU streams ASCII over USB CDC:

    D,<distance_mm>,<range_status>\r\n      single point, 30 Hz

Publishes sensor_msgs/Range, and the raw distance separately because Range
reports an invalid reading as +inf -- useful for the decision layer, useless
for watching the sensor come alive during bring-up.

    ros2 run gripper_sensors tof_node --ros-args -p port:=/dev/tof
"""

import os
import termios

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from std_msgs.msg import Float32

VALID_STATUS = 0


class TofNode(Node):

    def __init__(self):
        super().__init__('tof_node')
        port = self.declare_parameter('port', '/dev/tof').value
        self._frame = self.declare_parameter('frame_id', 'palm_tof').value
        self._fov = float(self.declare_parameter('field_of_view', 0.44).value)
        self._min = float(self.declare_parameter('min_range', 0.02).value)
        self._max = float(self.declare_parameter('max_range', 4.0).value)

        self._range = self.create_publisher(Range, 'tof/range', 10)
        self._raw = self.create_publisher(Float32, 'tof/raw', 10)

        self._fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self._set_raw()
        self._buf = b''
        self._bad = 0
        self.create_timer(0.005, self._poll)
        self.get_logger().info(f'reading {port}')

    def _set_raw(self):
        a = termios.tcgetattr(self._fd)
        a[0] = a[1] = a[3] = 0                      # iflag, oflag, lflag
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B115200               # ignored on USB CDC
        a[6][termios.VMIN] = 0
        a[6][termios.VTIME] = 0
        termios.tcsetattr(self._fd, termios.TCSANOW, a)

    def _poll(self):
        try:
            chunk = os.read(self._fd, 512)
        except BlockingIOError:
            return
        if not chunk:
            return
        self._buf += chunk
        # Keep only the last complete line: at 30 Hz a backlog is stale, and the
        # decision layer wants the newest reading, not every reading.
        *lines, self._buf = self._buf.split(b'\n')
        for line in lines[-1:]:
            self._emit(line.strip())

    def _emit(self, line):
        parts = line.decode('ascii', 'replace').split(',')
        if len(parts) != 3 or parts[0] != 'D':
            self._bad += 1
            if self._bad % 100 == 1:
                self.get_logger().warn(f'unparsed frame: {line!r}')
            return
        try:
            millimetres, status = int(parts[1]), int(parts[2])
        except ValueError:
            self._bad += 1
            return

        metres = millimetres / 1000.0
        self._raw.publish(Float32(data=metres))

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = self._fov
        msg.min_range = self._min
        msg.max_range = self._max
        msg.range = metres if status == VALID_STATUS else float('inf')
        self._range.publish(msg)

    def destroy_node(self):
        if getattr(self, '_fd', None) is not None:
            os.close(self._fd)
            self._fd = None
        super().destroy_node()


def main():
    rclpy.init()
    node = TofNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
