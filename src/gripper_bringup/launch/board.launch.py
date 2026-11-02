#!/usr/bin/env python3
"""The gripper's state on the PC.

/joint_states, the 3D model's /tf, and what dxl_cli and dxl_debug talk to. It needs
the micro-ROS Agent to hear from the board: docker/scripts/agent.sh.

    ros2 launch gripper_bringup board.launch.py
"""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution([
                    FindPackageShare('three_finger_gripper_description'),
                    'urdf', 'three_finger_gripper.urdf.xacro']),
            ]),
            value_type=str,
        )
    }

    return LaunchDescription([
        # Its default of 20 Hz would drop every other state from the board's 25 Hz.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description, {'publish_frequency': 30.0}], output='both'),
        # Republishes the board's joint states as /joint_states for the node above.
        Node(package='dynamixel_tools', executable='board_bridge', output='both'),
    ])
