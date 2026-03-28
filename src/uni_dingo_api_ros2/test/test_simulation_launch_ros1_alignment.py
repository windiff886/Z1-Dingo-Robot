import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
LAUNCH_PATH = REPO_ROOT / 'src' / 'uni_dingo_api_ros2' / 'launch' / 'simulation.launch.py'


class SimulationLaunchRos1AlignmentTest(unittest.TestCase):
    def test_simulation_launch_routes_through_dual_arm_gazebo_moveit(self):
        self.assertTrue(LAUNCH_PATH.exists(), 'simulation.launch.py must exist')

        launch_text = LAUNCH_PATH.read_text(encoding='utf-8')

        self.assertIn('dual_arm_gazebo_moveit.launch.py', launch_text)
        self.assertNotIn('dual_arm_world.launch.py', launch_text)

        for arg_name in [
            'gui',
            'rviz',
            'world',
            'paused',
            'teleop',
            'joystick',
            'moveit',
            'auto_safe_pose',
            'base',
            'left_arm',
            'right_arm',
            'head',
            'perception',
            'state_publisher',
        ]:
            with self.subTest(arg_name=arg_name):
                self.assertIn(f"'{arg_name}'", launch_text)

    def test_simulation_launch_keeps_ros1_style_control_toggles(self):
        launch_text = LAUNCH_PATH.read_text(encoding='utf-8')

        for arg_name in [
            'gui',
            'rviz',
            'world',
            'paused',
            'teleop',
            'joystick',
            'moveit',
            'auto_safe_pose',
            'base',
            'left_arm',
            'right_arm',
            'head',
            'perception',
        ]:
            with self.subTest(arg_name=arg_name):
                self.assertIn(f"'{arg_name}': LaunchConfiguration('{arg_name}')", launch_text)

    def test_simulation_launch_uses_ros1_world_default_and_state_publish_rate(self):
        launch_text = LAUNCH_PATH.read_text(encoding='utf-8')

        self.assertIn("'dingo_race.world'", launch_text)
        self.assertIn("'publish_rate': 10.0", launch_text)


if __name__ == '__main__':
    unittest.main()
