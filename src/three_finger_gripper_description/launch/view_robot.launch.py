from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'prefix', default_value='',
            description='Joint name prefix, for mounting more than one',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Whether to start RViz',
        ),
    ]

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution([
                    FindPackageShare('three_finger_gripper_description'),
                    'urdf',
                    'three_finger_gripper.urdf.xacro',
                ]),
                ' prefix:=', LaunchConfiguration('prefix'),
                ' use_mock_hardware:=true',
            ]),
            value_type=str,
        )
    }

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('three_finger_gripper_description'), 'rviz', 'view_robot.rviz']
    )

    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description], output='both'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             output='both'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
             condition=IfCondition(LaunchConfiguration('rviz')), output='log'),
    ]

    return LaunchDescription(declared_arguments + nodes)
