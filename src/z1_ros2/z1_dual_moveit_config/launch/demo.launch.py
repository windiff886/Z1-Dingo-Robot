from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('z1_dual_moveit_config')
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'),
            ' ',
            PathJoinSubstitution([package_share, 'config', 'dual_z1.urdf.xacro']),
        ])
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'use_gui': use_gui},
                {'source_list': ['/move_group/fake_controller_joint_states']},
            ],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([package_share, 'launch', 'move_group.launch.py'])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([package_share, 'launch', 'moveit_rviz.launch.py'])
            ),
            condition=IfCondition(use_rviz),
        ),
    ])
