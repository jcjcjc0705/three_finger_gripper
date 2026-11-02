#!/usr/bin/env python3
"""Lets dxl_cli and dxl_debug work on the gripper, with uros_node running on its board.

They expect five services, /joint_states, /dynamic_joint_states and a commands topic.
micro-ROS has room for two services and has no dynamixel_msgs, so the board takes text
on a topic instead (firmware_ns02501/src/motor_debug.hpp) and this node translates. The
board does the work and decides what is allowed; nothing is decided here.

    ros2 run dynamixel_tools board_bridge
    ros2 run dynamixel_tools dxl_cli dump 1
    ros2 run dynamixel_tools dxl_debug
"""

import re
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.msg import DynamicJointState, InterfaceValue
from dynamixel_msgs.srv import ReadRegister, Reboot, SetTorque, SetZero, WriteRegister
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

# The board's joint order: gripper_joints.hpp, and the order of its motor_diag.
JOINTS = ['C_2_Joint', 'C_3_Joint', 'A_2_Joint', 'A_3_Joint',
          'B_2_Joint', 'B_3_Joint', 'A_1_Joint', 'B_1_Joint']
DIAG_FIELDS = ['temperature', 'voltage', 'error', 'torque']

# A reboot answers once the motor is back, which the board gives three seconds.
REPLY_TIMEOUT_S = 4.0


class BoardBridge(Node):

    def __init__(self):
        super().__init__('gripper_board')
        self.declare_parameter('joints', JOINTS)

        self._lock = threading.Lock()
        self._seq = 0
        self._waiting = {}
        self._fault = 'none'

        group = ReentrantCallbackGroup()
        self._requests = self.create_publisher(String, '/gripper/motor_cmd', 10)
        self._joint_states = self.create_publisher(JointState, '/joint_states', 10)
        self._dynamic = self.create_publisher(DynamicJointState, '/dynamic_joint_states', 10)

        self.create_subscription(
            String, '/gripper/motor_reply', self._on_reply, 10, callback_group=group)
        self.create_subscription(
            JointState, '/gripper/joint_states', self._joint_states.publish, 10,
            callback_group=group)
        self.create_subscription(
            Float32MultiArray, '/gripper/motor_diag', self._on_diag, 10, callback_group=group)
        self.create_subscription(
            String, '/gripper/status', self._on_status, 10, callback_group=group)
        self.create_subscription(
            Float64MultiArray, '~/commands', self._on_commands, 10, callback_group=group)

        self.create_service(SetTorque, '~/set_torque', self._set_torque, callback_group=group)
        self.create_service(Reboot, '~/reboot', self._reboot, callback_group=group)
        self.create_service(SetZero, '~/set_zero', self._set_zero, callback_group=group)
        self.create_service(
            ReadRegister, '~/read_register', self._read_register, callback_group=group)
        self.create_service(
            WriteRegister, '~/write_register', self._write_register, callback_group=group)

    def _ask(self, text):
        """Sends one request and waits for its answer: (ok, value, message)."""
        event = threading.Event()
        answer = {}
        with self._lock:
            self._seq += 1
            seq = self._seq
            self._waiting[seq] = (event, answer)
        self._requests.publish(String(data=f'{seq} {text}'))
        got = event.wait(REPLY_TIMEOUT_S)
        with self._lock:
            self._waiting.pop(seq, None)
        if not got:
            return False, 0, f'no answer from the board in {REPLY_TIMEOUT_S:.0f} s'
        return answer['ok'], answer['value'], answer['message']

    def _on_reply(self, msg):
        parts = msg.data.split(' ', 3)
        if len(parts) < 3:
            self.get_logger().warning(f'unreadable reply: {msg.data}')
            return
        with self._lock:
            waiting = self._waiting.get(int(parts[0]))
        if waiting is None:
            return
        event, answer = waiting
        answer.update(ok=parts[1] == 'ok', value=int(parts[2]),
                      message=parts[3] if len(parts) > 3 else '')
        event.set()

    def _on_status(self, msg):
        match = re.search(r'\bfault=(\S+)', msg.data)
        if match is None or match.group(1) == self._fault:
            return
        self._fault = match.group(1)
        if self._fault == 'self_collision':
            self.get_logger().error(
                'Self-collision pose: every motor has lost torque. Move the fingers apart '
                'by hand.')
        else:
            self.get_logger().info(
                'Self-collision pose cleared. Turn torque back on: dxl_cli torque all on')

    def _on_diag(self, msg):
        fields = len(DIAG_FIELDS)
        if len(msg.data) != len(JOINTS) * fields:
            self.get_logger().warning(f'motor_diag has {len(msg.data)} values')
            return
        out = DynamicJointState(joint_names=JOINTS)
        out.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(JOINTS)):
            row = msg.data[i * fields:(i + 1) * fields]
            out.interface_values.append(
                InterfaceValue(interface_names=DIAG_FIELDS, values=[float(v) for v in row]))
        self._dynamic.publish(out)

    def _on_commands(self, msg):
        if len(msg.data) != len(JOINTS):
            self.get_logger().warning(f'commands has {len(msg.data)} values')
            return
        targets = ' '.join(f'{v:.4f}' for v in msg.data)
        self._requests.publish(String(data=f'0 jog {targets}'))

    def _answer(self, res, text):
        res.success, _, res.message = self._ask(text)
        return res

    def _set_torque(self, req, res):
        return self._answer(res, f'torque {req.id} {1 if req.enable else 0}')

    def _reboot(self, req, res):
        return self._answer(res, f'reboot {req.id}')

    def _set_zero(self, req, res):
        return self._answer(res, f'zero {req.id}')

    def _read_register(self, req, res):
        res.success, res.value, res.message = self._ask(
            f'read {req.id} {req.address} {req.size}')
        return res

    def _write_register(self, req, res):
        return self._answer(res, f'write {req.id} {req.address} {req.size} {req.value}')


def main():
    rclpy.init()
    node = BoardBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
