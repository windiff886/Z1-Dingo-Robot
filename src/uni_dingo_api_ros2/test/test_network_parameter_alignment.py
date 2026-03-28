import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
REAL_LAUNCH_PATH = REPO_ROOT / 'src' / 'uni_dingo_api_ros2' / 'launch' / 'real.launch.py'
UNIDINGO_XACRO_PATH = (
    REPO_ROOT / 'src' / 'unidingo_description_ros2' / 'urdf' / 'uni_dingo_dual_arm.urdf.xacro'
)
Z1_ROS2_CONTROL_XACRO_PATH = (
    REPO_ROOT / 'src' / 'z1_ros2' / 'z1_description' / 'urdf' / 'z1.ros2_control.xacro'
)
Z1_SYSTEM_CPP_PATH = (
    REPO_ROOT / 'src' / 'z1_ros2' / 'z1_hardware' / 'src' / 'z1_system.cpp'
)


class NetworkParameterAlignmentTest(unittest.TestCase):
    def test_real_launch_uses_ros1_style_dual_arm_network_arguments(self):
        launch_text = REAL_LAUNCH_PATH.read_text(encoding='utf-8')

        expected_args = {
            'left_arm_ip': '192.168.123.110',
            'right_arm_ip': '192.168.123.111',
            'left_port_to_mcu': '8881',
            'left_port_to_sdk': '8871',
            'left_own_port': '8872',
            'right_port_to_mcu': '8882',
            'right_port_to_sdk': '8873',
            'right_own_port': '8874',
        }

        for arg_name, default_value in expected_args.items():
            with self.subTest(arg_name=arg_name):
                self.assertIn(f"'{arg_name}'", launch_text)
                self.assertIn(f"default_value='{default_value}'", launch_text)

        for removed_arg in [
            'left_controller_ip',
            'left_sdk_port',
            'left_controller_port',
            'right_controller_ip',
            'right_sdk_port',
            'right_controller_port',
        ]:
            with self.subTest(removed_arg=removed_arg):
                self.assertNotIn(f"'{removed_arg}'", launch_text)

    def test_robot_description_xacro_passes_ros1_style_network_parameters(self):
        xacro_text = UNIDINGO_XACRO_PATH.read_text(encoding='utf-8')

        for arg_name in [
            'left_arm_ip',
            'left_port_to_mcu',
            'left_port_to_sdk',
            'left_own_port',
            'right_arm_ip',
            'right_port_to_mcu',
            'right_port_to_sdk',
            'right_own_port',
        ]:
            with self.subTest(arg_name=arg_name):
                self.assertIn(f'name="{arg_name}"', xacro_text)

        for passthrough in [
            'mcu_ip="$(arg left_arm_ip)"',
            'port_to_mcu="$(arg left_port_to_mcu)"',
            'port_to_sdk="$(arg left_port_to_sdk)"',
            'own_port="$(arg left_own_port)"',
            'mcu_ip="$(arg right_arm_ip)"',
            'port_to_mcu="$(arg right_port_to_mcu)"',
            'port_to_sdk="$(arg right_port_to_sdk)"',
            'own_port="$(arg right_own_port)"',
        ]:
            with self.subTest(passthrough=passthrough):
                self.assertIn(passthrough, xacro_text)

    def test_z1_ros2_control_macro_preserves_ros1_hardware_parameter_names(self):
        xacro_text = Z1_ROS2_CONTROL_XACRO_PATH.read_text(encoding='utf-8')

        for param_name in [
            'mcu_ip',
            'port_to_mcu',
            'port_to_sdk',
            'controller_ip',
            'own_port',
        ]:
            with self.subTest(param_name=param_name):
                self.assertIn(param_name, xacro_text)
                self.assertIn(f'<param name="{param_name}">', xacro_text)

    def test_z1_system_reads_ros1_style_hardware_parameters(self):
        cpp_text = Z1_SYSTEM_CPP_PATH.read_text(encoding='utf-8')

        for hardware_key in [
            '"mcu_ip"',
            '"port_to_mcu"',
            '"port_to_sdk"',
            '"controller_ip"',
            '"own_port"',
        ]:
            with self.subTest(hardware_key=hardware_key):
                self.assertIn(hardware_key, cpp_text)

        self.assertIn('UNITREE_ARM_SDK::UnitreeArm', cpp_text)
        self.assertIn('controller_ip_', cpp_text)
        self.assertIn('own_port_', cpp_text)
        self.assertIn('port_to_sdk_', cpp_text)


if __name__ == '__main__':
    unittest.main()
