from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    unitree_gripper = LaunchConfiguration('UnitreeGripperYN')
    robot_description = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('z1_description'), 'urdf', 'z1_standalone.urdf.xacro',
        ]),
        ' has_gripper:=', unitree_gripper,
    ]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('UnitreeGripperYN', default_value='true'),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'publish_frequency': 1000.0},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('z1_rviz'), 'rviz', 'model.rviz'
            ])],
        ),
    ])
