"""ROS2 simulation teleop chain aligned with ROS1 dingo_control teleop."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='joystick'),
        DeclareLaunchArgument('joystick', default_value='true'),
    ]

    joystick_enabled = IfCondition(PythonExpression([
        "'true' if '", LaunchConfiguration('enabled'), "' == 'true' and '",
        LaunchConfiguration('joystick'), "' == 'true' and '",
        LaunchConfiguration('teleop'), "' != 'keyboard' else 'false'",
    ]))

    keyboard_enabled = IfCondition(PythonExpression([
        "'true' if '", LaunchConfiguration('enabled'), "' == 'true' and '",
        LaunchConfiguration('teleop'), "' == 'keyboard' else 'false'",
    ]))

    joy_node = Node(
        package='joy',
        executable='joy_node',
        namespace='bluetooth_teleop',
        name='joy_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('unidingo_description'),
            'config',
            'teleop_omni.yaml',
        ])],
        condition=joystick_enabled,
    )

    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        namespace='bluetooth_teleop',
        name='teleop_twist_joy',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('unidingo_description'),
            'config',
            'teleop_omni.yaml',
        ])],
        condition=joystick_enabled,
    )

    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        namespace='keyboard_teleop',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        condition=keyboard_enabled,
    )

    return LaunchDescription(
        declared_args + [
            joy_node,
            teleop_twist_joy,
            keyboard_teleop,
        ]
    )
