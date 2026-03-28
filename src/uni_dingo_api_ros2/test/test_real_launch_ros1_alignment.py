import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
REAL_LAUNCH_PATH = REPO_ROOT / 'src' / 'uni_dingo_api_ros2' / 'launch' / 'real.launch.py'
D455_LAUNCH_PATH = REPO_ROOT / 'src' / 'd455_camera_ros2' / 'launch' / 'd455_camera.launch.py'
PAN_TILT_LAUNCH_PATH = (
    REPO_ROOT / 'src' / 'pan_tilt_ros2' / 'pan_tilt_bringup' / 'launch' / 'pan_tilt_bringup.launch.py'
)


class RealLaunchRos1AlignmentTest(unittest.TestCase):
    def test_real_launch_keeps_ros1_style_component_toggles(self):
        launch_text = REAL_LAUNCH_PATH.read_text(encoding='utf-8')

        self.assertIn("DeclareLaunchArgument('base', default_value='true')", launch_text)

        for arg_name in [
            'moveit',
            'rviz',
            'base',
            'left_arm',
            'right_arm',
            'head',
            'perception',
            'state_publisher',
            'has_gripper',
            'pan_tilt_port',
            'camera_serial',
            'color_fps',
            'depth_fps',
        ]:
            with self.subTest(arg_name=arg_name):
                self.assertIn(f"'{arg_name}'", launch_text)

        for hidden_arg in ['camera_name', 'camera_namespace']:
            with self.subTest(hidden_arg=hidden_arg):
                self.assertNotIn(f"DeclareLaunchArgument('{hidden_arg}'", launch_text)

    def test_real_launch_references_ros1_equivalent_component_chain(self):
        launch_text = REAL_LAUNCH_PATH.read_text(encoding='utf-8')

        for expected_text in [
            'robot_state_publisher',
            'z1_dual_bringup.launch.py',
            'pan_tilt_bringup.launch.py',
            'd455_camera.launch.py',
            'base.launch.py',
            'moveit.launch.py',
            'moveit_rviz.launch.py',
            'uni_dingo_state_publisher',
        ]:
            with self.subTest(expected_text=expected_text):
                self.assertIn(expected_text, launch_text)

        for removed_text in [
            'ros2_control_node',
            'joint_state_broadcaster',
            'left_arm_spawner',
            'left_gripper_spawner',
            'right_arm_spawner',
            'right_gripper_spawner',
        ]:
            with self.subTest(removed_text=removed_text):
                self.assertNotIn(removed_text, launch_text)

        self.assertTrue(
            'dual_arm.rviz' in launch_text or "executable='rviz2'" in launch_text,
            'real.launch.py 应提供非 MoveIt 模式下的 RViz 入口',
        )

    def test_real_launch_keeps_ros1_head_and_state_publisher_semantics(self):
        launch_text = REAL_LAUNCH_PATH.read_text(encoding='utf-8')

        self.assertIn("' pan_tilt:=', LaunchConfiguration('head')", launch_text)
        self.assertIn("'publish_rate': 10.0", launch_text)

    def test_d455_launch_only_exposes_ros1_style_camera_arguments(self):
        launch_text = D455_LAUNCH_PATH.read_text(encoding='utf-8')

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
        ]:
            with self.subTest(hidden_arg=hidden_arg):
                self.assertNotIn(f"DeclareLaunchArgument('{hidden_arg}'", launch_text)

    def test_pan_tilt_bringup_uses_ros1_style_port_contract(self):
        launch_text = PAN_TILT_LAUNCH_PATH.read_text(encoding='utf-8')

        self.assertIn("'pan_tilt_port'", launch_text)
        self.assertNotIn("DeclareLaunchArgument('port_name'", launch_text)
        self.assertNotIn("DeclareLaunchArgument('id'", launch_text)


if __name__ == '__main__':
    unittest.main()
