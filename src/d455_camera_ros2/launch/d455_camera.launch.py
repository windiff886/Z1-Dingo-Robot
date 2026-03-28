from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_serial', default_value=''),
        DeclareLaunchArgument('color_fps', default_value='30'),
        DeclareLaunchArgument('depth_fps', default_value='30'),
        Node(
            package='d455_camera',
            executable='d455_camera_node',
            name='d455_camera',
            output='screen',
            parameters=[{
                'serial_no': LaunchConfiguration('camera_serial'),
                'enable_color': True,
                'enable_depth': True,
                'color_width': 640,
                'color_height': 480,
                'color_fps': LaunchConfiguration('color_fps'),
                'depth_width': 640,
                'depth_height': 480,
                'depth_fps': LaunchConfiguration('depth_fps'),
                'color_frame_id': 'd455_color_optical_frame',
                'depth_frame_id': 'd455_depth_optical_frame',
                'use_hardware_time': True,
            }],
        ),
    ])
