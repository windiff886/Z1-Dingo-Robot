from setuptools import setup

package_name = 'whole_body_example'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/keyboard_teleop.launch.py',
            'launch/pose_visualizer.launch.py',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/pose_visualizer.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Example scripts for Uni-Dingo robot (ROS2)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'keyboard_teleop = whole_body_example.keyboard_teleop:main',
            'pose_visualizer = whole_body_example.pose_visualizer:main',
        ],
    },
)
