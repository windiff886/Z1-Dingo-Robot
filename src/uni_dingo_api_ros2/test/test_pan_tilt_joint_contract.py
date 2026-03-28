import pathlib
import unittest

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]

ROBOT_CONFIG = REPO_ROOT / "src/uni_dingo_api_ros2/config/robot_config.yaml"
PAN_TILT_BRINGUP = REPO_ROOT / "src/pan_tilt_ros2/pan_tilt_bringup/launch/pan_tilt_bringup.launch.py"
PAN_TILT_DRIVER = REPO_ROOT / "src/pan_tilt_ros2/pan_tilt_driver/src/pan_tilt_driver_node.cpp"
HEAD_API = REPO_ROOT / "src/uni_dingo_api_ros2/uni_dingo_api/api/head_api.py"
PAN_TILT_XACRO = REPO_ROOT / "src/pan_tilt_ros2/pan_tilt_description/urdf/pan_tilt.urdf.xacro"


class PanTiltJointContractTest(unittest.TestCase):
    def test_pan_tilt_joint_names_are_consistent(self):
        with ROBOT_CONFIG.open("r", encoding="utf-8") as fh:
            config = yaml.safe_load(fh)

        expected_pan_joint = config["subsystems"]["head"]["pan_joint"]
        expected_tilt_joint = config["subsystems"]["head"]["tilt_joint"]

        self.assertEqual(expected_pan_joint, "pan_tilt_yaw_joint")
        self.assertEqual(expected_tilt_joint, "pan_tilt_pitch_joint")

        head_api_text = HEAD_API.read_text(encoding="utf-8")
        self.assertIn("'pan_tilt_yaw_joint'", head_api_text)
        self.assertIn("'pan_tilt_pitch_joint'", head_api_text)

        bringup_text = PAN_TILT_BRINGUP.read_text(encoding="utf-8")
        self.assertIn("'yaw_joint_name': 'pan_tilt_yaw_joint'", bringup_text)
        self.assertIn("'pitch_joint_name': 'pan_tilt_pitch_joint'", bringup_text)

        driver_text = PAN_TILT_DRIVER.read_text(encoding="utf-8")
        self.assertIn('declare_parameter("yaw_joint_name", std::string("pan_tilt_yaw_joint"))', driver_text)
        self.assertIn('declare_parameter("pitch_joint_name", std::string("pan_tilt_pitch_joint"))', driver_text)
        self.assertNotIn("iqr_pan_tilt_yaw_joint", driver_text)
        self.assertNotIn("iqr_pan_tilt_pitch_joint", driver_text)

        xacro_text = PAN_TILT_XACRO.read_text(encoding="utf-8")
        self.assertIn('<xacro:property name="prefix" value="pan_tilt"/>', xacro_text)
        self.assertIn('name="${prefix}_yaw_joint"', xacro_text)
        self.assertIn('name="${prefix}_pitch_joint"', xacro_text)


if __name__ == "__main__":
    unittest.main()
