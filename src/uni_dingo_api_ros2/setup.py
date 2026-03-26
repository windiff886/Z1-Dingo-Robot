from setuptools import setup, find_packages

package_name = 'uni_dingo_api'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/robot_config.yaml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Unified Python API for the Uni-Dingo robot (ROS2)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'robot_state_publisher_node = uni_dingo_api.robot_state_publisher_node:main',
        ],
    },
)
