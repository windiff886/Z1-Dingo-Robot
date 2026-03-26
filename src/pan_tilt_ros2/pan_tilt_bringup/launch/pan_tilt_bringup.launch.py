from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/pan_tilt',
            description='Serial port for the pan-tilt device'),
        DeclareLaunchArgument(
            'id',
            default_value='1',
            description='Modbus device ID'),

        Node(
            package='pan_tilt_driver',
            executable='pan_tilt_driver_node',
            name='pan_tilt_driver',
            output='screen',
            parameters=[{
                'port_name': LaunchConfiguration('port_name'),
                'yaw_joint_name': 'iqr_pan_tilt_yaw_joint',
                'pitch_joint_name': 'iqr_pan_tilt_pitch_joint',
            }],
        ),
    ])
