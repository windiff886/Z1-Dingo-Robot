"""Bring up the dual-arm hardware-oriented model and controllers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('has_gripper', default_value='true'),
        DeclareLaunchArgument('pan_tilt', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('unidingo_description'), 'rviz', 'dual_arm.rviz'
            ]),
        ),
        DeclareLaunchArgument('left_arm_ip', default_value='192.168.123.110'),
        DeclareLaunchArgument('left_port_to_mcu', default_value='8881'),
        DeclareLaunchArgument('left_port_to_sdk', default_value='8871'),
        DeclareLaunchArgument('left_own_port', default_value='8872'),
        DeclareLaunchArgument('right_arm_ip', default_value='192.168.123.111'),
        DeclareLaunchArgument('right_port_to_mcu', default_value='8882'),
        DeclareLaunchArgument('right_port_to_sdk', default_value='8873'),
        DeclareLaunchArgument('right_own_port', default_value='8874'),
    ]

    robot_description = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('unidingo_description'), 'urdf', 'uni_dingo_dual_arm.urdf.xacro',
        ]),
        ' has_gripper:=', LaunchConfiguration('has_gripper'),
        ' pan_tilt:=', LaunchConfiguration('pan_tilt'),
        ' left_arm_ip:=', LaunchConfiguration('left_arm_ip'),
        ' left_port_to_mcu:=', LaunchConfiguration('left_port_to_mcu'),
        ' left_port_to_sdk:=', LaunchConfiguration('left_port_to_sdk'),
        ' left_own_port:=', LaunchConfiguration('left_own_port'),
        ' right_arm_ip:=', LaunchConfiguration('right_arm_ip'),
        ' right_port_to_mcu:=', LaunchConfiguration('right_port_to_mcu'),
        ' right_port_to_sdk:=', LaunchConfiguration('right_port_to_sdk'),
        ' right_own_port:=', LaunchConfiguration('right_own_port'),
    ]), value_type=str)

    controller_config = PathJoinSubstitution([
        FindPackageShare('z1_description'), 'config', 'z1_dual_controllers.yaml',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description}, controller_config],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    left_arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    right_arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    left_gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('has_gripper')),
    )

    right_gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('has_gripper')),
    )

    delay_arm_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[left_arm_spawner, right_arm_spawner],
        )
    )

    delay_gripper_spawners = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=left_arm_spawner,
                on_exit=[left_gripper_spawner],
            ),
            condition=IfCondition(LaunchConfiguration('has_gripper')),
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=right_arm_spawner,
                on_exit=[right_gripper_spawner],
            ),
            condition=IfCondition(LaunchConfiguration('has_gripper')),
        ),
    ]

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(
        declared_args + [
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster,
            delay_arm_spawners,
            *delay_gripper_spawners,
            rviz2,
        ]
    )
