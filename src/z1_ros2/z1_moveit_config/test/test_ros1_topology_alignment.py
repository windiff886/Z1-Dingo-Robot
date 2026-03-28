import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
Z1_ROS2_ROOT = REPO_ROOT / 'src' / 'z1_ros2'


class Z1Ros2TopologyAlignmentTest(unittest.TestCase):
    def test_ros2_workspace_restores_ros1_key_packages_and_docs(self):
        required_paths = [
            Z1_ROS2_ROOT / 'z1_dual_moveit_config' / 'package.xml',
            Z1_ROS2_ROOT / 'z1_dual_moveit_config' / 'launch' / 'move_group.launch.py',
            Z1_ROS2_ROOT / 'z1_dual_moveit_config' / 'launch' / 'moveit_rviz.launch.py',
            Z1_ROS2_ROOT / 'z1_examples' / 'package.xml',
            Z1_ROS2_ROOT / 'z1_examples' / 'examples' / 'move_group_interface.cpp',
            Z1_ROS2_ROOT / 'z1_examples' / 'examples' / 'ros_gripper_ctrl.cpp',
            Z1_ROS2_ROOT / 'z1_rviz' / 'package.xml',
            Z1_ROS2_ROOT / 'z1_rviz' / 'launch' / 'view_model.launch.py',
            Z1_ROS2_ROOT / 'z1_rviz' / 'launch' / 'view_robot.launch.py',
            Z1_ROS2_ROOT / 'z1_rviz' / 'rviz' / 'model.rviz',
            Z1_ROS2_ROOT / 'z1_rviz' / 'rviz' / 'robot.rviz',
            Z1_ROS2_ROOT / 'doc' / 'controller.md',
            Z1_ROS2_ROOT / 'doc' / 'setup.md',
            Z1_ROS2_ROOT / 'doc' / 'usage.md',
        ]

        missing = [str(path.relative_to(REPO_ROOT)) for path in required_paths if not path.exists()]
        self.assertEqual([], missing, f'Missing ROS1-aligned Z1 resources: {missing}')

    def test_dual_moveit_and_rviz_packages_keep_ros1_names(self):
        dual_moveit_package = (Z1_ROS2_ROOT / 'z1_dual_moveit_config' / 'package.xml').read_text(
            encoding='utf-8')
        rviz_package = (Z1_ROS2_ROOT / 'z1_rviz' / 'package.xml').read_text(encoding='utf-8')
        examples_package = (Z1_ROS2_ROOT / 'z1_examples' / 'package.xml').read_text(
            encoding='utf-8')

        self.assertIn('<name>z1_dual_moveit_config</name>', dual_moveit_package)
        self.assertIn('<name>z1_rviz</name>', rviz_package)
        self.assertIn('<name>z1_examples</name>', examples_package)


if __name__ == '__main__':
    unittest.main()
