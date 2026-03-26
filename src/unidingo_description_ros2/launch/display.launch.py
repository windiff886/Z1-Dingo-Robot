"""Launch file for displaying the Uni-Dingo robot in RViz2."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument('has_gripper', default_value='true'),
        DeclareLaunchArgument('pan_tilt', default_value='true'),
    ]

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('unidingo_description'), 'urdf', 'uni_dingo_dual_arm.urdf.xacro'
        ]),
        ' has_gripper:=', LaunchConfiguration('has_gripper'),
        ' pan_tilt:=', LaunchConfiguration('pan_tilt'),
    ])

    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
    )

    return LaunchDescription(
        declared_args + [
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz2,
        ]
    )
