"""Launch file for a single Z1 arm with ros2_control."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_args = [
        DeclareLaunchArgument('controller_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('sdk_port', default_value='0'),
        DeclareLaunchArgument('controller_port', default_value='8871'),
        DeclareLaunchArgument('has_gripper', default_value='true'),
    ]

    # Generate robot description via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('z1_description'), 'urdf', 'z1_standalone.urdf.xacro'
        ]),
        ' controller_ip:=', LaunchConfiguration('controller_ip'),
        ' sdk_port:=', LaunchConfiguration('sdk_port'),
        ' controller_port:=', LaunchConfiguration('controller_port'),
        ' has_gripper:=', LaunchConfiguration('has_gripper'),
    ])

    robot_description = {'robot_description': robot_description_content}

    controller_config = PathJoinSubstitution([
        FindPackageShare('z1_description'), 'config', 'z1_controllers.yaml'
    ])

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )

    # Chain spawners: wait for joint_state_broadcaster before arm controller
    delay_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    return LaunchDescription(
        declared_args + [
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster_spawner,
            delay_arm,
            delay_gripper,
        ]
    )
