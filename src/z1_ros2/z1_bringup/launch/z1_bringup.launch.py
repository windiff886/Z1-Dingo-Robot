"""
Launch file for a single Z1 arm.

启动顺序（与 ROS1 real_arm.launch 对齐）:
  1. z1_ctrl  — FSM 节点，连接 MCU，监听 Z1System 的 UDP 命令
  2. robot_state_publisher
  3. ros2_control_node  — 内含 Z1System（UnitreeArm SDK 客户端）
  4. joint_state_broadcaster spawner
  5. arm_controller / gripper_controller spawner（等待 broadcaster 就绪）
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── 参数声明（与 ROS1 real_ctrl.launch / real_arm.launch 完全一致）──────
    declared_args = [
        # z1_ctrl_ros2 参数（对应 ROS1 real_ctrl.launch）
        DeclareLaunchArgument('arm_ip',       default_value='192.168.123.110'),
        DeclareLaunchArgument('port_to_mcu',  default_value='8881'),
        DeclareLaunchArgument('port_to_sdk',  default_value='8871'),
        DeclareLaunchArgument('own_port',     default_value='8872'),
        # 硬件选项
        DeclareLaunchArgument('has_gripper',  default_value='true'),
        DeclareLaunchArgument('communication', default_value='UDP'),
    ]

    # ── 1. z1_ctrl 节点（FSM，对应 ROS1 的 z1_ctrl 进程）────────────────────
    z1_ctrl = Node(
        package='z1_controller_ros2',
        executable='z1_ctrl',
        name='z1_ctrl',
        output='screen',
        parameters=[{
            'udp.mcu_ip':      LaunchConfiguration('arm_ip'),
            'udp.port_to_mcu': LaunchConfiguration('port_to_mcu'),
            'udp.port_to_sdk': LaunchConfiguration('port_to_sdk'),
            'communication':   LaunchConfiguration('communication'),
            'collision.open':  False,
            'collision.limitT': 10.0,
            'cmdCache':        False,
        }],
    )

    # ── 2. robot_state_publisher ──────────────────────────────────────────────
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('z1_description'), 'urdf', 'z1_standalone.urdf.xacro'
        ]),
        ' port_to_sdk:=', LaunchConfiguration('port_to_sdk'),
        ' own_port:=',    LaunchConfiguration('own_port'),
        ' has_gripper:=', LaunchConfiguration('has_gripper'),
    ])

    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ── 3. ros2_control_node（Z1System SDK 客户端，连接 z1_ctrl）────────────
    controller_config = PathJoinSubstitution([
        FindPackageShare('z1_description'), 'config', 'z1_controllers.yaml'
    ])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
    )

    # ── 4 & 5. 控制器 spawner（与 ROS1 controller_spawner 对齐）─────────────
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

    # broadcaster 就绪后再启动轨迹控制器
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

    # ros2_control_node 在 z1_ctrl 启动后延迟 2 秒再启动，
    # 确保 z1_ctrl 已经与 MCU 建立连接
    delayed_ros2_control = TimerAction(
        period=2.0,
        actions=[
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster_spawner,
            delay_arm,
            delay_gripper,
        ]
    )

    return LaunchDescription(
        declared_args + [
            z1_ctrl,
            delayed_ros2_control,
        ]
    )
