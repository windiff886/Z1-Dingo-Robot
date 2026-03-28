"""Spawn the full Uni-Dingo robot model into Gazebo."""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=_resolve_default_world()),
        DeclareLaunchArgument('paused', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.1'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('has_gripper', default_value='true'),
        DeclareLaunchArgument('pan_tilt', default_value='true'),
    ]

    robot_description = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('unidingo_description'), 'urdf', 'uni_dingo_dual_arm.urdf.xacro',
        ]),
        ' sim_gazebo:=true',
        ' has_gripper:=', LaunchConfiguration('has_gripper'),
        ' pan_tilt:=', LaunchConfiguration('pan_tilt'),
    ]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py',
        ])),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'pause': LaunchConfiguration('paused'),
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py',
        ])),
        launch_arguments={
            'gui_required': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'uni_dingo_dual_arm',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    return LaunchDescription(
        declared_args + [
            gazebo,
            gazebo_client,
            robot_state_publisher,
            spawn_entity,
        ]
    )
