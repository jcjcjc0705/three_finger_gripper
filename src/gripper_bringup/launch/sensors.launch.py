#!/usr/bin/env python3
"""Palm camera from the board, and the Foxglove bridge.

The TOF has no node here: uros_node publishes /tof/range over micro-ROS.

    ros2 launch gripper_bringup sensors.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('board_host', default_value='192.168.0.3'),
        DeclareLaunchArgument('board_port', default_value='9000'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
    ]

    nodes = [
        Node(
            package='gripper_sensors', executable='palm_stream',
            name='palm_stream', output='both',
            parameters=[{
                'host': ParameterValue(LaunchConfiguration('board_host'), value_type=str),
                'port': ParameterValue(LaunchConfiguration('board_port'), value_type=int),
            }],
        ),
        # The board sends JPEG. This is the same camera decoded, for anything wanting raw images.
        Node(
            package='image_transport', executable='republish',
            name='palm_camera_decompress', output='both',
            parameters=[{'in_transport': 'compressed', 'out_transport': 'raw'}],
            remappings=[
                ('in/compressed', '/palm_camera/image_raw/compressed'),
                ('out', '/palm_camera/image_raw'),
            ],
        ),
        Node(
            package='foxglove_bridge', executable='foxglove_bridge',
            name='foxglove_bridge', output='both',
            condition=IfCondition(LaunchConfiguration('foxglove')),
            parameters=[{
                'port': ParameterValue(
                    LaunchConfiguration('foxglove_port'), value_type=int),
                # Lets the 3D panel resolve the package:// mesh paths in the URDF.
                'asset_uri_allowlist': ['^package://(?!.*\\.\\.).*'],
            }],
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)
