"""Real-hardware bringup for the Uni-Dingo robot (ROS2)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('moveit', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('base', default_value='true'),
        DeclareLaunchArgument('left_arm', default_value='true'),
        DeclareLaunchArgument('right_arm', default_value='true'),
        DeclareLaunchArgument('head', default_value='true'),
        DeclareLaunchArgument('perception', default_value='true'),
        DeclareLaunchArgument('state_publisher', default_value='true'),
        DeclareLaunchArgument('has_gripper', default_value='true'),
        DeclareLaunchArgument('left_arm_ip', default_value='192.168.123.110'),
        DeclareLaunchArgument('left_port_to_mcu', default_value='8881'),
        DeclareLaunchArgument('left_port_to_sdk', default_value='8871'),
        DeclareLaunchArgument('left_own_port', default_value='8872'),
        DeclareLaunchArgument('right_arm_ip', default_value='192.168.123.111'),
        DeclareLaunchArgument('right_port_to_mcu', default_value='8882'),
        DeclareLaunchArgument('right_port_to_sdk', default_value='8873'),
        DeclareLaunchArgument('right_own_port', default_value='8874'),
        DeclareLaunchArgument('pan_tilt_port', default_value='/dev/pan_tilt'),
        DeclareLaunchArgument('camera_serial', default_value=''),
        DeclareLaunchArgument('color_fps', default_value='30'),
        DeclareLaunchArgument('depth_fps', default_value='30'),
        DeclareLaunchArgument('moveit_package', default_value='z1_dual_moveit_config'),
    ]

    any_arm_enabled = PythonExpression([
        "'", LaunchConfiguration('left_arm'), "' == 'true' or '",
        LaunchConfiguration('right_arm'), "' == 'true'",
    ])
    moveit_with_rviz = PythonExpression([
        "'", LaunchConfiguration('moveit'), "' == 'true' and '",
        LaunchConfiguration('rviz'), "' == 'true'",
    ])
    rviz_without_moveit = PythonExpression([
        "'", LaunchConfiguration('rviz'), "' == 'true' and '",
        LaunchConfiguration('moveit'), "' != 'true'",
    ])

    robot_description = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('unidingo_description'), 'urdf', 'uni_dingo_dual_arm.urdf.xacro',
        ]),
        ' has_gripper:=', LaunchConfiguration('has_gripper'),
        ' pan_tilt:=', LaunchConfiguration('head'),
        ' left_arm:=', LaunchConfiguration('left_arm'),
        ' right_arm:=', LaunchConfiguration('right_arm'),
        ' left_arm_ip:=', LaunchConfiguration('left_arm_ip'),
        ' left_port_to_mcu:=', LaunchConfiguration('left_port_to_mcu'),
        ' left_port_to_sdk:=', LaunchConfiguration('left_port_to_sdk'),
        ' left_own_port:=', LaunchConfiguration('left_own_port'),
        ' right_arm_ip:=', LaunchConfiguration('right_arm_ip'),
        ' right_port_to_mcu:=', LaunchConfiguration('right_port_to_mcu'),
        ' right_port_to_sdk:=', LaunchConfiguration('right_port_to_sdk'),
        ' right_own_port:=', LaunchConfiguration('right_own_port'),
    ]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('z1_bringup'),
            'launch',
            'z1_dual_bringup.launch.py',
        ])),
        launch_arguments={
            'use_robot_state_publisher': 'false',
            'left_arm': LaunchConfiguration('left_arm'),
            'right_arm': LaunchConfiguration('right_arm'),
            'has_gripper': LaunchConfiguration('has_gripper'),
            'left_arm_ip': LaunchConfiguration('left_arm_ip'),
            'left_port_to_mcu': LaunchConfiguration('left_port_to_mcu'),
            'left_port_to_sdk': LaunchConfiguration('left_port_to_sdk'),
            'left_own_port': LaunchConfiguration('left_own_port'),
            'right_arm_ip': LaunchConfiguration('right_arm_ip'),
            'right_port_to_mcu': LaunchConfiguration('right_port_to_mcu'),
            'right_port_to_sdk': LaunchConfiguration('right_port_to_sdk'),
            'right_own_port': LaunchConfiguration('right_own_port'),
        }.items(),
        condition=IfCondition(any_arm_enabled),
    )

    pan_tilt_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('pan_tilt_bringup'),
            'launch',
            'pan_tilt_bringup.launch.py',
        ])),
        launch_arguments={
            'pan_tilt_port': LaunchConfiguration('pan_tilt_port'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('head')),
    )

    d455_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('d455_camera'),
            'launch',
            'd455_camera.launch.py',
        ])),
        launch_arguments={
            'camera_serial': LaunchConfiguration('camera_serial'),
            'color_fps': LaunchConfiguration('color_fps'),
            'depth_fps': LaunchConfiguration('depth_fps'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('perception')),
    )

    state_aggregator = Node(
        package='uni_dingo_api',
        executable='robot_state_publisher_node',
        name='uni_dingo_state_publisher',
        output='screen',
        parameters=[{'mode': 'real', 'publish_rate': 10.0}],
        condition=IfCondition(LaunchConfiguration('state_publisher')),
    )

    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('uni_dingo_api'),
            'launch',
            'includes',
            'base.launch.py',
        ])),
        condition=IfCondition(LaunchConfiguration('base')),
    )

    moveit_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('uni_dingo_api'),
            'launch',
            'includes',
            'moveit.launch.py',
        ])),
        launch_arguments={
            'moveit_package': LaunchConfiguration('moveit_package'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('moveit')),
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(LaunchConfiguration('moveit_package')),
            'launch',
            'moveit_rviz.launch.py',
        ])),
        condition=IfCondition(moveit_with_rviz),
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('unidingo_description'),
            'rviz',
            'dual_arm.rviz',
        ])],
        condition=IfCondition(rviz_without_moveit),
    )

    return LaunchDescription(
        declared_args + [
            base_bringup,
            robot_state_publisher,
            arm_bringup,
            pan_tilt_bringup,
            d455_camera,
            state_aggregator,
            moveit_bringup,
            moveit_rviz,
            rviz2,
        ]
    )
