import importlib.util
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = REPO_ROOT / 'src' / 'unidingo_description_ros2'
LAUNCH_DIR = PACKAGE_ROOT / 'launch'
CONFIG_DIR = PACKAGE_ROOT / 'config'
RVIZ_DIR = PACKAGE_ROOT / 'rviz'
SCRIPTS_DIR = PACKAGE_ROOT / 'scripts'


class UnidingoDescriptionLaunchContractTest(unittest.TestCase):
    def test_expected_launch_entrypoints_exist(self):
        required_paths = [
            LAUNCH_DIR / 'display.launch.py',
            LAUNCH_DIR / 'dual_arm_display.launch.py',
            LAUNCH_DIR / 'dual_arm_gazebo_moveit.launch.py',
            LAUNCH_DIR / 'dual_arm_hw.launch.py',
            LAUNCH_DIR / 'dual_arm_world.launch.py',
            LAUNCH_DIR / 'sim_base_control.launch.py',
            LAUNCH_DIR / 'sim_teleop.launch.py',
            LAUNCH_DIR / 'view_d455_only.launch.py',
            CONFIG_DIR / 'arm_simulation_controllers.yaml',
            CONFIG_DIR / 'base_simulation_controllers.yaml',
            CONFIG_DIR / 'gazebo_simulation_controllers.yaml',
            CONFIG_DIR / 'gains_omni.yaml',
            CONFIG_DIR / 'pan_tilt_controllers.yaml',
            CONFIG_DIR / 'robot_localization.yaml',
            CONFIG_DIR / 'teleop_omni.yaml',
            CONFIG_DIR / 'twist_mux.yaml',
            CONFIG_DIR / 'z1_gazebo_pid.yaml',
            RVIZ_DIR / 'dual_arm.rviz',
            RVIZ_DIR / 'd455_only.rviz',
            SCRIPTS_DIR / 'set_dual_arm_safe_pose.py',
        ]

        missing = [str(path.relative_to(REPO_ROOT)) for path in required_paths if not path.exists()]
        self.assertEqual([], missing, f'Missing expected description entrypoints: {missing}')

    def test_launch_files_are_importable(self):
        launch_files = [
            'display.launch.py',
            'dual_arm_display.launch.py',
            'dual_arm_gazebo_moveit.launch.py',
            'dual_arm_hw.launch.py',
            'dual_arm_world.launch.py',
            'sim_base_control.launch.py',
            'sim_teleop.launch.py',
            'view_d455_only.launch.py',
        ]

        for file_name in launch_files:
            with self.subTest(file_name=file_name):
                launch_path = LAUNCH_DIR / file_name
                self.assertTrue(launch_path.exists(), f'{file_name} must exist')
                spec = importlib.util.spec_from_file_location(file_name.replace('.', '_'), launch_path)
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                self.assertTrue(hasattr(module, 'generate_launch_description'))

    def test_dual_arm_gazebo_moveit_launch_references_safe_pose_script(self):
        launch_path = LAUNCH_DIR / 'dual_arm_gazebo_moveit.launch.py'
        self.assertTrue(launch_path.exists(), 'dual_arm_gazebo_moveit.launch.py must exist')

        launch_text = launch_path.read_text(encoding='utf-8')
        self.assertIn('set_dual_arm_safe_pose.py', launch_text)
        self.assertIn("'auto_safe_pose'", launch_text)
        self.assertIn("'auto_stow'", launch_text)

    def test_dual_arm_gazebo_moveit_launch_references_ros1_style_control_chain(self):
        launch_path = LAUNCH_DIR / 'dual_arm_gazebo_moveit.launch.py'
        launch_text = launch_path.read_text(encoding='utf-8')

        for expected_snippet in [
            'sim_base_control.launch.py',
            'sim_teleop.launch.py',
            'joint_state_broadcaster',
            'left_arm/left_z1_joint_traj_controller',
            'right_arm/right_z1_joint_traj_controller',
            'mobile_base_controller',
            'pan_controller',
            'tilt_controller',
            'moveit_rviz.launch.py',
        ]:
            with self.subTest(expected_snippet=expected_snippet):
                self.assertIn(expected_snippet, launch_text)

    def test_dual_arm_gazebo_moveit_launch_passes_ros1_style_safe_pose_parameters(self):
        launch_path = LAUNCH_DIR / 'dual_arm_gazebo_moveit.launch.py'
        launch_text = launch_path.read_text(encoding='utf-8')

        for expected_snippet in [
            'controllers_to_load',
            'start_controllers',
            'pre_unpause_for_controllers',
            'disable_gravity_during_controller_start',
            'restore_gravity_after_safe_pose',
            'switch_timeout',
        ]:
            with self.subTest(expected_snippet=expected_snippet):
                self.assertIn(expected_snippet, launch_text)

    def test_dual_arm_gazebo_moveit_launch_keeps_ros1_defaults_and_controller_names(self):
        launch_path = LAUNCH_DIR / 'dual_arm_gazebo_moveit.launch.py'
        launch_text = launch_path.read_text(encoding='utf-8')

        self.assertIn("'dingo_race.world'", launch_text)
        self.assertIn("DeclareLaunchArgument('rviz', default_value='true')", launch_text)
        self.assertIn("DeclareLaunchArgument('auto_stow', default_value='true')", launch_text)
        self.assertIn('left_arm/left_z1_joint_traj_controller', launch_text)
        self.assertIn('right_arm/right_z1_joint_traj_controller', launch_text)

    def test_simulation_urdf_has_gazebo_ros2_control_hook(self):
        urdf_path = PACKAGE_ROOT / 'urdf' / 'uni_dingo_dual_arm.urdf.xacro'
        urdf_text = urdf_path.read_text(encoding='utf-8')

        for expected_snippet in [
            'sim_gazebo',
            'gazebo_ros2_control',
            'gazebo_simulation_controllers.yaml',
            'pan_tilt_yaw_joint',
            'pan_tilt_pitch_joint',
        ]:
            with self.subTest(expected_snippet=expected_snippet):
                self.assertIn(expected_snippet, urdf_text)

    def test_safe_pose_script_mentions_controller_and_physics_workflow(self):
        script_path = SCRIPTS_DIR / 'set_dual_arm_safe_pose.py'
        script_text = script_path.read_text(encoding='utf-8')

        for expected_snippet in [
            'LoadController',
            'SwitchController',
            'GetPhysicsProperties',
            'SetPhysicsProperties',
            'controllers_to_load',
            'start_controllers',
            'disable_gravity_during_controller_start',
            'restore_gravity_after_safe_pose',
            'pre_unpause_for_controllers',
        ]:
            with self.subTest(expected_snippet=expected_snippet):
                self.assertIn(expected_snippet, script_text)


if __name__ == '__main__':
    unittest.main()
