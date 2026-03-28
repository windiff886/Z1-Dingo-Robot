"""Shared MoveIt2 include for Uni-Dingo ROS2 bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('moveit_package', default_value='z1_moveit_config'),
    ]

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(LaunchConfiguration('moveit_package')),
            'launch',
            'move_group.launch.py',
        ])),
    )

    return LaunchDescription(declared_args + [move_group])
