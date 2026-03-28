"""
Launch file for dual Z1 arms.

启动顺序（与 ROS1 real_arm.launch 双臂模式对齐）:
  1. left_z1_ctrl  — 左臂 FSM 节点（.110:8881, sdk_port=8871）
  2. right_z1_ctrl — 右臂 FSM 节点（.111:8882, sdk_port=8873）
  3. robot_state_publisher
  4. ros2_control_node  — 内含左右两个 Z1System
  5. joint_state_broadcaster spawner
  6. left/right arm_controller / gripper_controller spawner
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, RegisterEventHandler, TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── 参数声明（与 ROS1 real_arm.launch 双臂参数完全一致）─────────────────
    declared_args = [
        DeclareLaunchArgument('left_arm',            default_value='true'),
        DeclareLaunchArgument('right_arm',           default_value='true'),
        DeclareLaunchArgument('use_robot_state_publisher', default_value='true'),
        DeclareLaunchArgument('has_gripper',          default_value='true'),
        DeclareLaunchArgument('communication',         default_value='UDP'),
        # 左臂（对应 ROS1 left_arm namespace 下的 real_ctrl.launch 参数）
        DeclareLaunchArgument('left_arm_ip',          default_value='192.168.123.110'),
        DeclareLaunchArgument('left_port_to_mcu',     default_value='8881'),
        DeclareLaunchArgument('left_port_to_sdk',     default_value='8871'),
        DeclareLaunchArgument('left_own_port',        default_value='8872'),
        # 右臂
        DeclareLaunchArgument('right_arm_ip',         default_value='192.168.123.111'),
        DeclareLaunchArgument('right_port_to_mcu',    default_value='8882'),
        DeclareLaunchArgument('right_port_to_sdk',    default_value='8873'),
        DeclareLaunchArgument('right_own_port',       default_value='8874'),
    ]

    any_arm_enabled = PythonExpression([
        "'", LaunchConfiguration('left_arm'), "' == 'true' or '",
        LaunchConfiguration('right_arm'), "' == 'true'",
    ])
    left_gripper_enabled = PythonExpression([
        "'", LaunchConfiguration('left_arm'), "' == 'true' and '",
        LaunchConfiguration('has_gripper'), "' == 'true'",
    ])
    right_gripper_enabled = PythonExpression([
        "'", LaunchConfiguration('right_arm'), "' == 'true' and '",
        LaunchConfiguration('has_gripper'), "' == 'true'",
    ])
    use_robot_state_publisher = PythonExpression([
        "'", LaunchConfiguration('use_robot_state_publisher'), "' == 'true' and '",
        LaunchConfiguration('left_arm'), "' == 'true' or '",
        LaunchConfiguration('use_robot_state_publisher'), "' == 'true' and '",
        LaunchConfiguration('right_arm'), "' == 'true'",
    ])

    # ── 1 & 2. 两个 z1_ctrl 节点（对应 ROS1 的两个 z1_ctrl 进程）────────────
    left_z1_ctrl = Node(
        package='z1_controller_ros2',
        executable='z1_ctrl',
        name='left_z1_ctrl',
        output='screen',
        parameters=[{
            'udp.mcu_ip':      LaunchConfiguration('left_arm_ip'),
            'udp.port_to_mcu': LaunchConfiguration('left_port_to_mcu'),
            'udp.port_to_sdk': LaunchConfiguration('left_port_to_sdk'),
            'communication':   LaunchConfiguration('communication'),
            'collision.open':  False,
            'collision.limitT': 10.0,
            'cmdCache':        False,
        }],
        condition=IfCondition(LaunchConfiguration('left_arm')),
    )

    right_z1_ctrl = Node(
        package='z1_controller_ros2',
        executable='z1_ctrl',
        name='right_z1_ctrl',
        output='screen',
        parameters=[{
            'udp.mcu_ip':      LaunchConfiguration('right_arm_ip'),
            'udp.port_to_mcu': LaunchConfiguration('right_port_to_mcu'),
            'udp.port_to_sdk': LaunchConfiguration('right_port_to_sdk'),
            'communication':   LaunchConfiguration('communication'),
            'collision.open':  False,
            'collision.limitT': 10.0,
            'cmdCache':        False,
        }],
        condition=IfCondition(LaunchConfiguration('right_arm')),
    )

    # ── 3. robot_state_publisher ──────────────────────────────────────────────
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('z1_description'), 'urdf', 'z1_dual_standalone.urdf.xacro'
        ]),
        ' left_arm:=',        LaunchConfiguration('left_arm'),
        ' right_arm:=',       LaunchConfiguration('right_arm'),
        ' has_gripper:=',       LaunchConfiguration('has_gripper'),
        ' left_port_to_sdk:=',  LaunchConfiguration('left_port_to_sdk'),
        ' left_own_port:=',     LaunchConfiguration('left_own_port'),
        ' right_port_to_sdk:=', LaunchConfiguration('right_port_to_sdk'),
        ' right_own_port:=',    LaunchConfiguration('right_own_port'),
    ])

    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        condition=IfCondition(use_robot_state_publisher),
    )

    # ── 4. ros2_control_node（含左右两个 Z1System）──────────────────────────
    controller_config = PathJoinSubstitution([
        FindPackageShare('z1_description'), 'config', 'z1_dual_controllers.yaml'
    ])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
        condition=IfCondition(any_arm_enabled),
    )

    # ── 5 & 6. 控制器 spawner（与 ROS1 双臂 controller_spawner 对齐）────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
        condition=IfCondition(any_arm_enabled),
    )

    left_arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm/left_z1_joint_traj_controller'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('left_arm')),
    )
    left_gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm/left_gripper_controller'],
        output='screen',
        condition=IfCondition(left_gripper_enabled),
    )
    right_arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm/right_z1_joint_traj_controller'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('right_arm')),
    )
    right_gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm/right_gripper_controller'],
        output='screen',
        condition=IfCondition(right_gripper_enabled),
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

    # 两个 z1_ctrl 都启动后，延迟 2 秒让其完成与 MCU 的握手
    delayed_ros2_control = TimerAction(
        period=2.0,
        actions=[
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster_spawner,
            delay_controllers,
        ],
        condition=IfCondition(any_arm_enabled),
    )

    return LaunchDescription(
        declared_args + [
            left_z1_ctrl,
            right_z1_ctrl,
            delayed_ros2_control,
        ]
    )
