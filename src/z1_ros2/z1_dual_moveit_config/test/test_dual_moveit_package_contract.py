import importlib.util
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
PACKAGE_ROOT = REPO_ROOT / 'src' / 'z1_ros2' / 'z1_dual_moveit_config'
CONFIG_DIR = PACKAGE_ROOT / 'config'
LAUNCH_DIR = PACKAGE_ROOT / 'launch'


class DualMoveItConfigPackageContractTest(unittest.TestCase):
    def test_dual_moveit_package_restores_local_assets(self):
        required_paths = [
            PACKAGE_ROOT / 'package.xml',
            PACKAGE_ROOT / 'CMakeLists.txt',
            CONFIG_DIR / 'dual_z1.srdf',
            CONFIG_DIR / 'dual_z1.urdf.xacro',
            CONFIG_DIR / 'joint_limits.yaml',
            CONFIG_DIR / 'kinematics.yaml',
            CONFIG_DIR / 'ompl_planning.yaml',
            CONFIG_DIR / 'ros_controllers.yaml',
            CONFIG_DIR / 'moveit_controllers.yaml',
            LAUNCH_DIR / 'demo.launch.py',
            LAUNCH_DIR / 'move_group.launch.py',
            LAUNCH_DIR / 'moveit_rviz.launch.py',
            LAUNCH_DIR / 'moveit.rviz',
        ]

        missing = [str(path.relative_to(REPO_ROOT)) for path in required_paths if not path.exists()]
        self.assertEqual([], missing, f'Missing expected dual-arm MoveIt assets: {missing}')

    def test_primary_launches_are_self_hosted(self):
        move_group_text = (LAUNCH_DIR / 'move_group.launch.py').read_text(encoding='utf-8')
        moveit_rviz_text = (LAUNCH_DIR / 'moveit_rviz.launch.py').read_text(encoding='utf-8')

        self.assertIn("package_name='z1_dual_moveit_config'", move_group_text)
        self.assertIn("package_name='z1_dual_moveit_config'", moveit_rviz_text)
        self.assertNotIn("FindPackageShare('z1_moveit_config')", move_group_text)
        self.assertNotIn("FindPackageShare('z1_moveit_config')", moveit_rviz_text)

    def test_move_group_launch_is_importable(self):
        launch_path = LAUNCH_DIR / 'move_group.launch.py'
        self.assertTrue(launch_path.exists(), 'move_group.launch.py must exist')

        spec = importlib.util.spec_from_file_location('z1_dual_move_group_launch', launch_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        self.assertTrue(hasattr(module, 'generate_launch_description'))


if __name__ == '__main__':
    unittest.main()
