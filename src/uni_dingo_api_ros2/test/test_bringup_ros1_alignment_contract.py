import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
LAUNCH_PATH = REPO_ROOT / 'src' / 'uni_dingo_api_ros2' / 'launch' / 'bringup.launch.py'


class BringupRos1AlignmentContractTest(unittest.TestCase):
    def test_bringup_restores_mode_routing_and_top_level_args(self):
        self.assertTrue(LAUNCH_PATH.exists(), 'bringup.launch.py must exist')

        launch_text = LAUNCH_PATH.read_text(encoding='utf-8')

        self.assertIn("'mode'", launch_text)
        self.assertIn("default_value='simulation'", launch_text)
        self.assertIn('simulation.launch.py', launch_text)
        self.assertIn('real.launch.py', launch_text)

        for arg_name in [
            'gui',
            'rviz',
            'world',
            'paused',
            'teleop',
            'moveit',
            'auto_safe_pose',
            'base',
            'left_arm',
            'right_arm',
            'head',
            'perception',
        ]:
            with self.subTest(arg_name=arg_name):
                self.assertIn(f"'{arg_name}'", launch_text)

        self.assertIn("'dingo_race.world'", launch_text)
        for hidden_arg in ['state_publisher', 'has_gripper', 'moveit_package']:
            with self.subTest(hidden_arg=hidden_arg):
                self.assertNotIn(f"DeclareLaunchArgument('{hidden_arg}'", launch_text)


if __name__ == '__main__':
    unittest.main()
