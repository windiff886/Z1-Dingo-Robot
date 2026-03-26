import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    realsense_dir = get_package_share_directory('realsense2_camera')
    realsense_launch = os.path.join(realsense_dir, 'launch', 'rs_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='d455'),
        DeclareLaunchArgument('camera_namespace', default_value='d455'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_namespace': LaunchConfiguration('camera_namespace'),
                'enable_color': 'true',
                'enable_depth': 'true',
                'rgb_camera.color_profile': '640x480x30',
                'depth_module.depth_profile': '640x480x30',
                'align_depth.enable': 'true',
                'pointcloud.enable': 'true',
            }.items(),
        ),
    ])
