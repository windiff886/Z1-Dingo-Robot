"""Model-visualization entrypoint for the Uni-Dingo robot (ROS2)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('has_gripper', default_value='true'),
        DeclareLaunchArgument('head', default_value='true'),
        DeclareLaunchArgument('perception', default_value='true'),
    ]

    include_pan_tilt = PythonExpression([
        "'true' if '", LaunchConfiguration('head'), "' == 'true' or '",
        LaunchConfiguration('perception'), "' == 'true' else 'false'",
    ])

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('unidingo_description'),
            'launch',
            'dual_arm_display.launch.py',
        ])),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'has_gripper': LaunchConfiguration('has_gripper'),
            'pan_tilt': include_pan_tilt,
        }.items(),
    )

    return LaunchDescription(declared_args + [display_launch])
