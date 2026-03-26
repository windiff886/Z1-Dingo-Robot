"""Launch file for dual Z1 arms with ros2_control."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('has_gripper', default_value='true'),
        DeclareLaunchArgument('left_controller_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('left_sdk_port', default_value='0'),
        DeclareLaunchArgument('left_controller_port', default_value='8871'),
        DeclareLaunchArgument('right_controller_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('right_sdk_port', default_value='0'),
        DeclareLaunchArgument('right_controller_port', default_value='8872'),
    ]

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('z1_description'), 'urdf', 'z1_dual_standalone.urdf.xacro'
        ]),
        ' has_gripper:=', LaunchConfiguration('has_gripper'),
        ' left_controller_ip:=', LaunchConfiguration('left_controller_ip'),
        ' left_sdk_port:=', LaunchConfiguration('left_sdk_port'),
        ' left_controller_port:=', LaunchConfiguration('left_controller_port'),
        ' right_controller_ip:=', LaunchConfiguration('right_controller_ip'),
        ' right_sdk_port:=', LaunchConfiguration('right_sdk_port'),
        ' right_controller_port:=', LaunchConfiguration('right_controller_port'),
    ])

    robot_description = {'robot_description': robot_description_content}

    controller_config = PathJoinSubstitution([
        FindPackageShare('z1_description'), 'config', 'z1_dual_controllers.yaml'
    ])

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

    left_arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller'],
        output='screen',
    )

    left_gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_gripper_controller'],
        output='screen',
    )

    right_arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller'],
        output='screen',
    )

    right_gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_gripper_controller'],
        output='screen',
    )

    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                left_arm_spawner,
                left_gripper_spawner,
                right_arm_spawner,
                right_gripper_spawner,
            ],
        )
    )

    return LaunchDescription(
        declared_args + [
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster_spawner,
            delay_controllers,
        ]
    )
