"""Backward-compatible display entrypoint for the Uni-Dingo robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('has_gripper', default_value='true'),
        DeclareLaunchArgument('pan_tilt', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('unidingo_description'), 'rviz', 'dual_arm.rviz'
            ]),
        ),
    ]

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('unidingo_description'),
            'launch',
            'dual_arm_display.launch.py',
        ])),
        launch_arguments={
            'has_gripper': LaunchConfiguration('has_gripper'),
            'pan_tilt': LaunchConfiguration('pan_tilt'),
            'gui': LaunchConfiguration('gui'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
    )

    return LaunchDescription(declared_args + [display_launch])
