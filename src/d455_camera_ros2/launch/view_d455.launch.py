"""Bring up the D455 camera and a standalone visualization entrypoint."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('camera_name', default_value='d455'),
        DeclareLaunchArgument('camera_namespace', default_value='d455'),
        DeclareLaunchArgument('camera_serial', default_value=''),
        DeclareLaunchArgument('color_fps', default_value='30'),
        DeclareLaunchArgument('depth_fps', default_value='30'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('d455_camera'), 'rviz', 'view_d455.rviz'
            ]),
        ),
    ]

    camera_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('d455_camera'), 'launch', 'd455_camera.launch.py',
        ])),
        launch_arguments={
            'camera_serial': LaunchConfiguration('camera_serial'),
            'color_fps': LaunchConfiguration('color_fps'),
            'depth_fps': LaunchConfiguration('depth_fps'),
        }.items(),
    )

    camera_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('unidingo_description'), 'launch', 'view_d455_only.launch.py',
        ])),
        launch_arguments={
            'gui': 'false',
            'use_rviz': 'false',
        }.items(),
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(declared_args + [camera_bringup, camera_model, rviz2])
