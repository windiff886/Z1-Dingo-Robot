"""Simulation-mode bringup for the Uni-Dingo robot (ROS2)."""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_default_world() -> str:
    try:
        return str(Path(get_package_share_directory('dingo_gazebo')) / 'worlds' / 'dingo_race.world')
    except PackageNotFoundError:
        current_file = Path(__file__).resolve()
        for parent in current_file.parents:
            candidate = parent / 'src' / 'dingo' / 'dingo_gazebo' / 'worlds' / 'dingo_race.world'
            if candidate.exists():
                return str(candidate)

    raise RuntimeError('无法定位 dingo_race.world，请确认 dingo_gazebo 已存在于工作区。')


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'world',
            default_value=_resolve_default_world(),
        ),
        DeclareLaunchArgument('paused', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='none'),
        DeclareLaunchArgument('joystick', default_value='true'),
        DeclareLaunchArgument('moveit', default_value='true'),
        DeclareLaunchArgument('auto_safe_pose', default_value='true'),
        DeclareLaunchArgument('base', default_value='true'),
        DeclareLaunchArgument('left_arm', default_value='true'),
        DeclareLaunchArgument('right_arm', default_value='true'),
        DeclareLaunchArgument('head', default_value='true'),
        DeclareLaunchArgument('perception', default_value='true'),
        DeclareLaunchArgument('state_publisher', default_value='true'),
        DeclareLaunchArgument('has_gripper', default_value='true'),
        DeclareLaunchArgument('moveit_package', default_value='z1_moveit_config'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.1'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('unidingo_description'),
            'launch',
            'dual_arm_gazebo_moveit.launch.py',
        ])),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'world': LaunchConfiguration('world'),
            'paused': LaunchConfiguration('paused'),
            'teleop': LaunchConfiguration('teleop'),
            'joystick': LaunchConfiguration('joystick'),
            'moveit': LaunchConfiguration('moveit'),
            'auto_safe_pose': LaunchConfiguration('auto_safe_pose'),
            'base': LaunchConfiguration('base'),
            'left_arm': LaunchConfiguration('left_arm'),
            'right_arm': LaunchConfiguration('right_arm'),
            'head': LaunchConfiguration('head'),
            'perception': LaunchConfiguration('perception'),
            'has_gripper': LaunchConfiguration('has_gripper'),
            'moveit_package': LaunchConfiguration('moveit_package'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    state_aggregator = Node(
        package='uni_dingo_api',
        executable='robot_state_publisher_node',
        name='uni_dingo_state_publisher',
        output='screen',
        parameters=[{'mode': 'simulation', 'publish_rate': 10.0}],
        condition=IfCondition(LaunchConfiguration('state_publisher')),
    )

    return LaunchDescription(
        declared_args + [
            gazebo_world,
            state_aggregator,
        ]
    )
