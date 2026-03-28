from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pan_tilt_port',
            default_value='/dev/pan_tilt',
            description='Serial port for the pan-tilt device'),

        Node(
            package='pan_tilt_driver',
            executable='pan_tilt_driver_node',
            name='pan_tilt_driver',
            output='screen',
            parameters=[{
                'port_name': LaunchConfiguration('pan_tilt_port'),
                'yaw_joint_name': 'pan_tilt_yaw_joint',
                'pitch_joint_name': 'pan_tilt_pitch_joint',
            }],
        ),
    ])
