import importlib.util
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = REPO_ROOT / 'src' / 'd455_camera_ros2'
LAUNCH_DIR = PACKAGE_ROOT / 'launch'
SRC_DIR = PACKAGE_ROOT / 'src'


class D455LaunchContractTest(unittest.TestCase):
    def test_camera_launch_declares_ros1_style_arguments(self):
        launch_path = LAUNCH_DIR / 'd455_camera.launch.py'
        self.assertTrue(launch_path.exists(), 'd455_camera.launch.py must exist')

        launch_text = launch_path.read_text(encoding='utf-8')
        for arg_name in ['camera_serial', 'color_fps', 'depth_fps']:
            with self.subTest(arg_name=arg_name):
                self.assertIn(f"'{arg_name}'", launch_text)

        for hidden_arg in [
            'serial_no',
            'camera_name',
            'camera_namespace',
            'color_width',
            'color_height',
            'depth_width',
            'depth_height',
            'enable_imu',
        ]:
            with self.subTest(hidden_arg=hidden_arg):
                self.assertNotIn(f"DeclareLaunchArgument('{hidden_arg}'", launch_text)

        self.assertIn("package='d455_camera'", launch_text)
        self.assertIn("executable='d455_camera_node'", launch_text)
        self.assertNotIn('realsense2_camera', launch_text)
        self.assertNotIn('rs_launch.py', launch_text)

    def test_package_contains_in_repo_ros2_camera_source(self):
        required_paths = [
            PACKAGE_ROOT / 'package.xml',
            PACKAGE_ROOT / 'CMakeLists.txt',
            SRC_DIR / 'd455_camera_node.cpp',
        ]

        missing = [str(path.relative_to(REPO_ROOT)) for path in required_paths if not path.exists()]
        self.assertEqual([], missing, f'Missing D455 ROS2 source files: {missing}')

        cmake_text = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
        package_text = (PACKAGE_ROOT / 'package.xml').read_text(encoding='utf-8')

        self.assertIn('project(d455_camera)', cmake_text)
        self.assertIn('add_executable(d455_camera_node', cmake_text)
        self.assertIn('<name>d455_camera</name>', package_text)

    def test_view_d455_launch_exists_and_is_importable(self):
        launch_path = LAUNCH_DIR / 'view_d455.launch.py'
        self.assertTrue(launch_path.exists(), 'view_d455.launch.py must exist')

        spec = importlib.util.spec_from_file_location('view_d455_launch', launch_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertTrue(hasattr(module, 'generate_launch_description'))


if __name__ == '__main__':
    unittest.main()
