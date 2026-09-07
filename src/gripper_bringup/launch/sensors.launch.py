#!/usr/bin/env python3
"""Palm camera, TOF and the Foxglove bridge.

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
        DeclareLaunchArgument('camera_device', default_value='/dev/palm_cam'),
        DeclareLaunchArgument('tof_port', default_value='/dev/tof'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('tof', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
    ]

    nodes = [
        Node(
            package='v4l2_camera', executable='v4l2_camera_node',
            name='palm_camera', namespace='palm_camera', output='both',
            condition=IfCondition(LaunchConfiguration('camera')),
            parameters=[{
                'video_device': ParameterValue(
                    LaunchConfiguration('camera_device'), value_type=str),
                'camera_frame_id': 'palm_camera',
            }],
        ),
        Node(
            package='gripper_sensors', executable='tof_node',
            name='palm_tof', output='both',
            condition=IfCondition(LaunchConfiguration('tof')),
            parameters=[{
                'port': ParameterValue(LaunchConfiguration('tof_port'), value_type=str),
                'frame_id': 'palm_tof',
            }],
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
