from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('z1_rviz'), 'rviz', 'robot.rviz'
            ])],
        ),
    ])
