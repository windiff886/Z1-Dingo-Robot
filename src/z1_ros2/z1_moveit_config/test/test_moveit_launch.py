import importlib.util
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
PACKAGE_ROOT = REPO_ROOT / 'src' / 'z1_ros2' / 'z1_moveit_config'
CONFIG_DIR = PACKAGE_ROOT / 'config'
LAUNCH_DIR = PACKAGE_ROOT / 'launch'


class MoveItConfigPackageContractTest(unittest.TestCase):
    def test_moveit_config_package_files_exist(self):
        required_paths = [
            PACKAGE_ROOT / 'package.xml',
            PACKAGE_ROOT / 'CMakeLists.txt',
            CONFIG_DIR / 'dual_z1.srdf',
            CONFIG_DIR / 'kinematics.yaml',
            CONFIG_DIR / 'joint_limits.yaml',
            CONFIG_DIR / 'moveit_controllers.yaml',
            LAUNCH_DIR / 'move_group.launch.py',
            LAUNCH_DIR / 'moveit_rviz.launch.py',
        ]

        missing = [str(path.relative_to(REPO_ROOT)) for path in required_paths if not path.exists()]
        self.assertEqual([], missing, f'Missing expected MoveIt2 config files: {missing}')

    def test_srdf_declares_dual_arm_groups(self):
        srdf_path = CONFIG_DIR / 'dual_z1.srdf'
        self.assertTrue(srdf_path.exists(), 'dual_z1.srdf must exist')

        srdf_text = srdf_path.read_text(encoding='utf-8')
        self.assertIn('left_arm_manipulator', srdf_text)
        self.assertIn('right_arm_manipulator', srdf_text)

    def test_move_group_launch_is_importable(self):
        launch_path = LAUNCH_DIR / 'move_group.launch.py'
        self.assertTrue(launch_path.exists(), 'move_group.launch.py must exist')

        spec = importlib.util.spec_from_file_location('z1_move_group_launch', launch_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        self.assertTrue(hasattr(module, 'generate_launch_description'))


if __name__ == '__main__':
    unittest.main()
