"""Unified bringup entrypoint for the Uni-Dingo robot (ROS2)."""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
        DeclareLaunchArgument(
            'mode',
            default_value='simulation',
            description="Operating mode: 'simulation' or 'real'",
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch Gazebo GUI',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz visualization',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=_resolve_default_world(),
            description='Gazebo world path override',
        ),
        DeclareLaunchArgument(
            'paused',
            default_value='true',
            description='Start Gazebo paused',
        ),
        DeclareLaunchArgument(
            'teleop',
            default_value='none',
            description="Teleop method: 'keyboard', 'joystick', or 'none'",
        ),
        DeclareLaunchArgument(
            'moveit',
            default_value='true',
            description='Whether to launch the MoveIt2 planning stack',
        ),
        DeclareLaunchArgument(
            'auto_safe_pose',
            default_value='true',
            description='Move arms to safe pose on startup',
        ),
        DeclareLaunchArgument(
            'base',
            default_value='true',
            description='Whether to enable the mobile base integration',
        ),
        DeclareLaunchArgument(
            'left_arm',
            default_value='true',
            description='Whether to enable the left arm control path',
        ),
        DeclareLaunchArgument(
            'right_arm',
            default_value='true',
            description='Whether to enable the right arm control path',
        ),
        DeclareLaunchArgument(
            'head',
            default_value='true',
            description='Whether to enable the pan-tilt head',
        ),
        DeclareLaunchArgument(
            'perception',
            default_value='true',
            description='Whether to enable the D455 camera pipeline',
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Spawn x position',
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Spawn y position',
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.1',
            description='Spawn z position',
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Spawn yaw angle',
        ),
    ]

    simulation_launch_arguments = {
        'gui': LaunchConfiguration('gui'),
        'rviz': LaunchConfiguration('rviz'),
        'world': LaunchConfiguration('world'),
        'paused': LaunchConfiguration('paused'),
        'teleop': LaunchConfiguration('teleop'),
        'moveit': LaunchConfiguration('moveit'),
        'auto_safe_pose': LaunchConfiguration('auto_safe_pose'),
        'base': LaunchConfiguration('base'),
        'left_arm': LaunchConfiguration('left_arm'),
        'right_arm': LaunchConfiguration('right_arm'),
        'head': LaunchConfiguration('head'),
        'perception': LaunchConfiguration('perception'),
        'state_publisher': 'true',
        'has_gripper': 'true',
        'moveit_package': 'z1_moveit_config',
        'x': LaunchConfiguration('x'),
        'y': LaunchConfiguration('y'),
        'z': LaunchConfiguration('z'),
        'yaw': LaunchConfiguration('yaw'),
    }
    real_launch_arguments = {
        'rviz': LaunchConfiguration('rviz'),
        'moveit': LaunchConfiguration('moveit'),
        'base': LaunchConfiguration('base'),
        'left_arm': LaunchConfiguration('left_arm'),
        'right_arm': LaunchConfiguration('right_arm'),
        'head': LaunchConfiguration('head'),
        'perception': LaunchConfiguration('perception'),
        'state_publisher': 'true',
        'has_gripper': 'true',
        'moveit_package': 'z1_dual_moveit_config',
    }
    simulation_mode = IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'simulation'"]))
    real_mode = IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'real'"]))

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uni_dingo_api'),
                'launch',
                'simulation.launch.py',
            ])
        ),
        launch_arguments=simulation_launch_arguments.items(),
        condition=simulation_mode,
    )
    real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uni_dingo_api'),
                'launch',
                'real.launch.py',
            ])
        ),
        launch_arguments=real_launch_arguments.items(),
        condition=real_mode,
    )

    return LaunchDescription(
        declared_args + [
            simulation_launch,
            real_launch,
        ]
    )
