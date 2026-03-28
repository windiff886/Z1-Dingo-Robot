"""ROS2 simulation base control chain aligned with ROS1 dingo_control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('enabled', default_value='true'),
    ]

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('unidingo_description'),
            'config',
            'twist_mux.yaml',
        ])],
        remappings=[
            ('cmd_vel_out', 'mobile_base_controller/cmd_vel_unstamped'),
        ],
        condition=IfCondition(LaunchConfiguration('enabled')),
    )

    ekf_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('unidingo_description'),
            'config',
            'robot_localization.yaml',
        ])],
        condition=IfCondition(LaunchConfiguration('enabled')),
    )

    return LaunchDescription(
        declared_args + [
            twist_mux,
            ekf_localization,
        ]
    )
