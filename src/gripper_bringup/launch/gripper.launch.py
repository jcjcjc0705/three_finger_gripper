#!/usr/bin/env python3
"""ros2_control stack for the gripper.

    ros2 launch gripper_bringup gripper.launch.py
    ros2 launch gripper_bringup gripper.launch.py use_mock_hardware:=true rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('port_name', default_value='/dev/gripper'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'controller', default_value='gripper_controller',
            description='Which one starts active. The other is loaded inactive, '
                        'so switching needs no restart: '
                        'ros2 control switch_controllers '
                        '--deactivate A --activate B'),
    ]

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution([
                    FindPackageShare('three_finger_gripper_description'),
                    'urdf', 'three_finger_gripper.urdf.xacro']),
                ' prefix:=', LaunchConfiguration('prefix'),
                ' use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'),
                ' port_name:=', LaunchConfiguration('port_name'),
            ]),
            value_type=str,
        )
    }

    controllers = PathJoinSubstitution(
        [FindPackageShare('gripper_bringup'), 'config', 'gripper_controllers.yaml'])

    control_node = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[controllers], output='both')

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[robot_description], output='both')

    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'], output='screen')

    controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[LaunchConfiguration('controller')], output='screen')

    # The other one is loaded but left inactive. A command interface can only be
    # claimed by one active controller, so they can never both run -- but having
    # both configured makes swapping a switch instead of a restart. That matters
    # because dxl_debug and dxl_cli only speak to position_controller.
    other_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[PythonExpression([
            "'position_controller' if '", LaunchConfiguration('controller'),
            "' == 'gripper_controller' else 'gripper_controller'"]),
            '--inactive'],
        output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('three_finger_gripper_description'),
            'rviz', 'view_robot.rviz'])])

    return LaunchDescription(declared_arguments + [
        control_node,
        robot_state_publisher,
        jsb_spawner,
        # Only claim the command interfaces once the state broadcaster is up, so
        # a failure to talk to the bus shows up before anything is commanded.
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner,
                                           on_exit=[controller_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=controller_spawner,
                                           on_exit=[other_spawner])),
        rviz,
    ])
