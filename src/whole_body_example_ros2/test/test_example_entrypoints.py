import importlib.util
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
WHOLE_BODY_ROOT = REPO_ROOT / 'src' / 'whole_body_example_ros2'
PAN_TILT_DRIVER_ROOT = REPO_ROOT / 'src' / 'pan_tilt_ros2' / 'pan_tilt_driver'


class ExampleEntrypointsContractTest(unittest.TestCase):
    def test_pose_visualizer_resources_exist(self):
        required_paths = [
            WHOLE_BODY_ROOT / 'whole_body_example' / 'pose_visualizer.py',
            WHOLE_BODY_ROOT / 'launch' / 'pose_visualizer.launch.py',
            WHOLE_BODY_ROOT / 'rviz' / 'pose_visualizer.rviz',
        ]

        missing = [str(path.relative_to(REPO_ROOT)) for path in required_paths if not path.exists()]
        self.assertEqual([], missing, f'Missing pose visualizer resources: {missing}')

    def test_pose_visualizer_launch_is_importable(self):
        launch_path = WHOLE_BODY_ROOT / 'launch' / 'pose_visualizer.launch.py'
        self.assertTrue(launch_path.exists(), 'pose_visualizer.launch.py must exist')

        spec = importlib.util.spec_from_file_location('pose_visualizer_launch', launch_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertTrue(hasattr(module, 'generate_launch_description'))

    def test_setup_installs_pose_visualizer_entrypoints(self):
        setup_text = (WHOLE_BODY_ROOT / 'setup.py').read_text(encoding='utf-8')
        self.assertIn('pose_visualizer = whole_body_example.pose_visualizer:main', setup_text)
        self.assertIn('launch/pose_visualizer.launch.py', setup_text)

    def test_pan_tilt_control_script_is_installed(self):
        script_path = PAN_TILT_DRIVER_ROOT / 'scripts' / 'pan_tilt_control.py'
        self.assertTrue(script_path.exists(), 'pan_tilt_control.py must exist')

        cmake_text = (PAN_TILT_DRIVER_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
        self.assertIn('scripts/pan_tilt_control.py', cmake_text)

    def test_keyboard_teleop_launch_keeps_ros1_default_mode(self):
        launch_text = (WHOLE_BODY_ROOT / 'launch' / 'keyboard_teleop.launch.py').read_text(
            encoding='utf-8')

        self.assertIn("DeclareLaunchArgument(", launch_text)
        self.assertIn("'mode'", launch_text)
        self.assertIn("default_value='simulation'", launch_text)

    def test_keyboard_teleop_launch_keeps_required_node_semantics(self):
        launch_text = (WHOLE_BODY_ROOT / 'launch' / 'keyboard_teleop.launch.py').read_text(
            encoding='utf-8')

        self.assertIn('Shutdown()', launch_text)

    def test_pose_visualizer_keeps_ros1_publish_rate_and_required_semantics(self):
        script_text = (WHOLE_BODY_ROOT / 'whole_body_example' / 'pose_visualizer.py').read_text(
            encoding='utf-8')
        launch_text = (WHOLE_BODY_ROOT / 'launch' / 'pose_visualizer.launch.py').read_text(
            encoding='utf-8')

        self.assertIn("declare_parameter('publish_rate', 50.0)", script_text)
        self.assertIn('Shutdown()', launch_text)

    def test_pose_visualizer_launch_does_not_expose_ros2_only_arguments(self):
        launch_text = (WHOLE_BODY_ROOT / 'launch' / 'pose_visualizer.launch.py').read_text(
            encoding='utf-8')

        for hidden_arg in ['has_gripper', 'pan_tilt', 'use_rviz', 'rviz_config']:
            with self.subTest(hidden_arg=hidden_arg):
                self.assertNotIn(f"DeclareLaunchArgument('{hidden_arg}'", launch_text)

    def test_keyboard_teleop_keeps_ros1_style_interactive_text(self):
        script_text = (WHOLE_BODY_ROOT / 'whole_body_example' / 'keyboard_teleop.py').read_text(
            encoding='utf-8')

        self.assertIn('Uni-Dingo 键盘遥控', script_text)
        self.assertIn('等待键盘输入', script_text)
        self.assertIn('退出键盘遥控', script_text)

    def test_pose_visualizer_keeps_ros1_style_interactive_text(self):
        script_text = (WHOLE_BODY_ROOT / 'whole_body_example' / 'pose_visualizer.py').read_text(
            encoding='utf-8')

        self.assertIn('机械臂位姿可视化工具', script_text)
        self.assertIn('输入命令 > ', script_text)
        self.assertIn('退出', script_text)
        self.assertIn("if command == 'help':", script_text)
        self.assertIn('def set_pose(self, pose_name):', script_text)
        self.assertIn('print(f"位姿: {pose_name}")', script_text)


if __name__ == '__main__':
    unittest.main()
