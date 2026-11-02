#!/usr/bin/env python3
"""Palm camera from the board, streamed by its uros_node.

Publishes palm_camera/image_raw/compressed (JPEG).

    ros2 run gripper_sensors palm_stream --ros-args -p host:=192.168.0.3
"""

import socket
import struct
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

HEADER = struct.Struct('<4sBBHIQI')
MAGIC = b'PSS1'
TYPE_JPEG = 1


class Rate:

    def __init__(self):
        self.count = 0
        self.bytes = 0
        self.lost = 0
        self.last_seq = None

    def add(self, seq, size):
        if self.last_seq is not None and seq > self.last_seq + 1:
            self.lost += seq - self.last_seq - 1
        self.last_seq = seq
        self.count += 1
        self.bytes += size

    def take(self):
        out = (self.count, self.bytes, self.lost)
        self.count = self.bytes = self.lost = 0
        return out


class PalmStream(Node):

    def __init__(self):
        super().__init__('palm_stream')
        self._host = self.declare_parameter('host', '192.168.0.3').value
        self._port = int(self.declare_parameter('port', 9000).value)
        self._camera_frame = self.declare_parameter('camera_frame_id', 'palm_camera').value
        report = float(self.declare_parameter('report_period', 5.0).value)

        self._image = self.create_publisher(
            CompressedImage, 'palm_camera/image_raw/compressed', 1)

        self._lock = threading.Lock()
        self._cam = Rate()
        self._last_report = time.monotonic()
        self._running = True
        self._sock = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.create_timer(report, self._report)

    def _run(self):
        while self._running and rclpy.ok():
            try:
                sock = socket.create_connection((self._host, self._port), timeout=2.0)
            except OSError as e:
                self.get_logger().warn(
                    f'cannot reach {self._host}:{self._port} ({e}), retrying',
                    throttle_duration_sec=10.0)
                time.sleep(1.0)
                continue
            sock.settimeout(3.0)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._sock = sock
            self.get_logger().info(f'connected to {self._host}:{self._port}')
            try:
                self._receive(sock)
            except (OSError, ValueError) as e:
                if self._running:
                    self.get_logger().warn(f'stream lost: {e}')
            finally:
                sock.close()
                self._sock = None

    def _read_exact(self, sock, n):
        buf = bytearray(n)
        view = memoryview(buf)
        got = 0
        while got < n:
            k = sock.recv_into(view[got:], n - got)
            if k == 0:
                raise ValueError('board closed the connection')
            got += k
        return buf

    def _receive(self, sock):
        while self._running:
            magic, kind, _, _, seq, _stamp_ns, length = HEADER.unpack(
                self._read_exact(sock, HEADER.size))
            if magic != MAGIC:
                raise ValueError(f'bad magic {bytes(magic)!r}')
            payload = self._read_exact(sock, length)
            if kind == TYPE_JPEG:
                self._on_jpeg(seq, payload)

    def _on_jpeg(self, seq, payload):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._camera_frame
        msg.format = 'jpeg'
        msg.data = bytes(payload[4:])
        self._image.publish(msg)
        with self._lock:
            self._cam.add(seq, len(payload))

    def _report(self):
        now = time.monotonic()
        dt = now - self._last_report
        self._last_report = now
        with self._lock:
            frames, nbytes, lost = self._cam.take()
        if self._sock is None:
            return
        self.get_logger().info(
            f'camera {frames / dt:5.1f} fps ({nbytes / max(frames, 1) / 1024:4.1f} KB/frame, '
            f'{nbytes * 8 / dt / 1e6:5.2f} Mbit/s, lost {lost})')

    def destroy_node(self):
        self._running = False
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = PalmStream()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
