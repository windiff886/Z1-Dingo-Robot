"""
Full system bringup for the Uni-Dingo robot (ROS2).

Launches:
  - Robot state publisher (unified URDF)
  - ros2_control (dual Z1 arms + joint state broadcaster)
  - Pan-tilt driver (real mode only)
  - D455 camera (real mode only)
  - Robot state aggregator node

Usage:
  # Simulation (URDF + controllers only)
  ros2 launch uni_dingo_api bringup.launch.py mode:=simulation

  # Real hardware
  ros2 launch uni_dingo_api bringup.launch.py mode:=real
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='simulation',
        description='Robot mode: simulation or real')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz2')

    left_controller_ip_arg = DeclareLaunchArgument(
        'left_controller_ip', default_value='192.168.123.110',
        description='Left Z1 arm controller IP')

    right_controller_ip_arg = DeclareLaunchArgument(
        'right_controller_ip', default_value='192.168.123.111',
        description='Right Z1 arm controller IP')

    mode = LaunchConfiguration('mode')
    use_rviz = LaunchConfiguration('use_rviz')
    is_real = PythonExpression(["'", mode, "' == 'real'"])

    # --- Robot Description ---
    xacro_file = PathJoinSubstitution([
        FindPackageShare('unidingo_description'), 'urdf',
        'uni_dingo_dual_arm.urdf.xacro',
    ])

    robot_description = Command([
        'xacro ', xacro_file,
        ' left_controller_ip:=', LaunchConfiguration('left_controller_ip'),
        ' right_controller_ip:=', LaunchConfiguration('right_controller_ip'),
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # --- ros2_control (dual arm controllers) ---
    controller_config = PathJoinSubstitution([
        FindPackageShare('z1_description'), 'config',
        'z1_dual_controllers.yaml',
    ])

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_config],
        output='screen',
        condition=IfCondition(is_real),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(is_real),
    )

    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(is_real),
    )

    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(is_real),
    )

    left_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(is_real),
    )

    right_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(is_real),
    )

    # Chain spawners: JSB -> arm controllers -> gripper controllers
    delay_left_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_arm_controller_spawner],
        ),
        condition=IfCondition(is_real),
    )

    delay_right_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[right_arm_controller_spawner],
        ),
        condition=IfCondition(is_real),
    )

    delay_left_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_controller_spawner,
            on_exit=[left_gripper_controller_spawner],
        ),
        condition=IfCondition(is_real),
    )

    delay_right_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[right_gripper_controller_spawner],
        ),
        condition=IfCondition(is_real),
    )

    # --- Pan-Tilt Driver (real mode) ---
    pan_tilt_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('pan_tilt_bringup'), 'launch',
                'pan_tilt_bringup.launch.py',
            ])
        ),
        condition=IfCondition(is_real),
    )

    # --- D455 Camera (real mode) ---
    d455_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('d455_camera_bringup'), 'launch',
                'd455_camera.launch.py',
            ])
        ),
        condition=IfCondition(is_real),
    )

    # --- Robot state aggregator ---
    robot_state_aggregator = Node(
        package='uni_dingo_api',
        executable='robot_state_publisher_node',
        name='uni_dingo_state_publisher',
        output='screen',
    )

    # --- Simulation joint state publisher (for testing without hardware) ---
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'simulation'"])),
    )

    # --- RViz2 ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        # Arguments
        mode_arg,
        use_rviz_arg,
        left_controller_ip_arg,
        right_controller_ip_arg,
        # Robot description
        robot_state_publisher,
        # ros2_control (real mode)
        ros2_control_node,
        joint_state_broadcaster_spawner,
        delay_left_arm,
        delay_right_arm,
        delay_left_gripper,
        delay_right_gripper,
        # Peripherals (real mode)
        pan_tilt_bringup,
        d455_camera,
        # State aggregator
        robot_state_aggregator,
        # Simulation helpers
        joint_state_publisher_gui,
        # Visualization
        rviz_node,
    ])
