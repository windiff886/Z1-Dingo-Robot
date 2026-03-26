"""Launch keyboard teleop for Uni-Dingo robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='',
        description='Robot mode: simulation or real')

    teleop_node = Node(
        package='whole_body_example',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        prefix='xterm -e',
        parameters=[{'mode': LaunchConfiguration('mode')}],
    )

    return LaunchDescription([
        mode_arg,
        teleop_node,
    ])
