"""Gazebo + MoveIt bringup for the full Uni-Dingo dual-arm robot."""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('world', default_value=_resolve_default_world()),
        DeclareLaunchArgument('paused', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='keyboard'),
        DeclareLaunchArgument('joystick', default_value='true'),
        DeclareLaunchArgument('moveit', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('auto_stow', default_value='true'),
        DeclareLaunchArgument('auto_safe_pose', default_value='true'),
        DeclareLaunchArgument('base', default_value='true'),
        DeclareLaunchArgument('left_arm', default_value='true'),
        DeclareLaunchArgument('right_arm', default_value='true'),
        DeclareLaunchArgument('head', default_value='true'),
        DeclareLaunchArgument('perception', default_value='true'),
        DeclareLaunchArgument('has_gripper', default_value='true'),
        DeclareLaunchArgument('moveit_package', default_value='z1_moveit_config'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.1'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    pan_tilt_enabled = PythonExpression([
        "'true' if '", LaunchConfiguration('head'), "' == 'true' or '",
        LaunchConfiguration('perception'), "' == 'true' else 'false'",
    ])

    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('unidingo_description'),
            'launch',
            'dual_arm_world.launch.py',
        ])),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'headless': LaunchConfiguration('headless'),
            'world': LaunchConfiguration('world'),
            'paused': LaunchConfiguration('paused'),
            'has_gripper': LaunchConfiguration('has_gripper'),
            'pan_tilt': pan_tilt_enabled,
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    base_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('unidingo_description'),
            'launch',
            'sim_base_control.launch.py',
        ])),
        launch_arguments={
            'enabled': LaunchConfiguration('base'),
        }.items(),
    )

    teleop_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('unidingo_description'),
            'launch',
            'sim_teleop.launch.py',
        ])),
        launch_arguments={
            'enabled': LaunchConfiguration('base'),
            'teleop': LaunchConfiguration('teleop'),
            'joystick': LaunchConfiguration('joystick'),
        }.items(),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    mobile_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mobile_base_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('base')),
    )

    pan_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pan_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(pan_tilt_enabled),
    )

    tilt_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tilt_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(pan_tilt_enabled),
    )

    delay_controller_spawners = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                mobile_base_controller_spawner,
                pan_controller_spawner,
                tilt_controller_spawner,
            ],
        )
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(LaunchConfiguration('moveit_package')),
            'launch',
            'move_group.launch.py',
        ])),
        condition=IfCondition(LaunchConfiguration('moveit')),
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(LaunchConfiguration('moveit_package')),
            'launch',
            'moveit_rviz.launch.py',
        ])),
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    safe_pose = Node(
        package='unidingo_description',
        executable='set_dual_arm_safe_pose.py',
        name='set_dual_arm_safe_pose',
        output='screen',
        parameters=[{
            'model_name': 'uni_dingo_dual_arm',
            'left_positions': [0.0, 0.0, 0.0, -0.07435, 0.0, 0.0],
            'right_positions': [0.0, 0.0, 0.0, -0.07435, 0.0, 0.0],
            'spawn_model': True,
            'spawn_x': LaunchConfiguration('x'),
            'spawn_y': LaunchConfiguration('y'),
            'spawn_z': LaunchConfiguration('z'),
            'spawn_yaw': LaunchConfiguration('yaw'),
            'robot_description_param': '/robot_description',
            'set_model': PythonExpression([
                "'false' if '", LaunchConfiguration('auto_stow'), "' == 'true' else 'true'",
            ]),
            'stow_only': LaunchConfiguration('auto_stow'),
            'strict': True,
            'retry_interval': 0.5,
            'switch_timeout': 2.0,
            'unpause': PythonExpression([
                "'false' if '", LaunchConfiguration('paused'), "' == 'true' else 'true'",
            ]),
            'unpause_after_set_model': False,
            'pre_unpause_for_controllers': True,
            'pause_on_failure': True,
            'disable_gravity_during_controller_start': True,
            'restore_gravity_after_safe_pose': True,
            'controllers_to_load': [
                'left_arm/left_z1_joint_traj_controller',
                'right_arm/right_z1_joint_traj_controller',
            ],
            'start_controllers': [
                'left_arm/left_z1_joint_traj_controller',
                'right_arm/right_z1_joint_traj_controller',
            ],
        }],
        condition=IfCondition(LaunchConfiguration('auto_safe_pose')),
    )

    return LaunchDescription(
        declared_args + [
            gazebo_world,
            base_control,
            teleop_control,
            joint_state_broadcaster_spawner,
            delay_controller_spawners,
            move_group,
            moveit_rviz,
            safe_pose,
        ]
    )
