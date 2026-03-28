"""Launch the pose visualizer with robot model publishing and RViz."""

from launch import LaunchDescription
from launch.actions import Shutdown
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('unidingo_description'), 'urdf', 'uni_dingo_dual_arm.urdf.xacro',
        ]),
        ' has_gripper:=true',
        ' pan_tilt:=true',
    ]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'publish_frequency': 50.0},
        ],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('whole_body_example'), 'rviz', 'pose_visualizer.rviz'
        ])],
    )

    pose_visualizer = Node(
        package='whole_body_example',
        executable='pose_visualizer',
        output='screen',
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz2,
        pose_visualizer,
    ])
