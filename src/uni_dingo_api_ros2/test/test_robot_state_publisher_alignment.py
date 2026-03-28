import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = REPO_ROOT / 'src' / 'uni_dingo_api_ros2'

if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from uni_dingo_api.robot_state_publisher_node import RobotStatePublisherNode
from uni_dingo_msgs.msg import SubsystemStatus


def _make_publisher():
    publisher = SimpleNamespace(
        _odom=None,
        _joint_state=None,
        _left_arm_joints=[f'left_joint{i}' for i in range(1, 7)],
        _right_arm_joints=[f'right_joint{i}' for i in range(1, 7)],
        _head_joints=['pan_tilt_yaw_joint', 'pan_tilt_pitch_joint'],
        _make_status=RobotStatePublisherNode._make_status,
    )
    publisher._build_gripper_state = (
        lambda side: RobotStatePublisherNode._build_gripper_state(publisher, side)
    )
    return publisher


class RobotStatePublisherAlignmentTest(unittest.TestCase):
    def test_subsystem_statuses_match_ros1_ready_semantics(self):
        publisher = _make_publisher()

        statuses = RobotStatePublisherNode._build_subsystem_statuses(publisher)
        statuses_by_name = {status.name: status for status in statuses}

        self.assertEqual(statuses_by_name['base'].status, SubsystemStatus.STATUS_OK)
        self.assertFalse(statuses_by_name['base'].ready)
        self.assertEqual(statuses_by_name['base'].message, 'Waiting for odometry')

        self.assertEqual(statuses_by_name['left_arm'].status, SubsystemStatus.STATUS_OK)
        self.assertFalse(statuses_by_name['left_arm'].ready)
        self.assertEqual(statuses_by_name['left_arm'].message, 'Waiting for joint states')

        self.assertEqual(statuses_by_name['right_arm'].status, SubsystemStatus.STATUS_OK)
        self.assertFalse(statuses_by_name['right_arm'].ready)
        self.assertEqual(statuses_by_name['right_arm'].message, 'Waiting for joint states')

        self.assertEqual(statuses_by_name['head'].status, SubsystemStatus.STATUS_OK)
        self.assertTrue(statuses_by_name['head'].ready)
        self.assertEqual(statuses_by_name['head'].message, 'OK')

        self.assertEqual(statuses_by_name['perception'].status, SubsystemStatus.STATUS_OK)
        self.assertTrue(statuses_by_name['perception'].ready)
        self.assertEqual(statuses_by_name['perception'].message, 'OK')

    def test_state_headers_only_copy_source_timestamps(self):
        publisher = _make_publisher()
        publisher._odom = Odometry()
        publisher._odom.header.stamp.sec = 123
        publisher._odom.header.stamp.nanosec = 456
        publisher._odom.header.frame_id = 'odom'

        publisher._joint_state = JointState()
        publisher._joint_state.header.stamp.sec = 789
        publisher._joint_state.header.stamp.nanosec = 12
        publisher._joint_state.header.frame_id = 'joint_states'
        publisher._joint_state.name = [
            'left_joint1',
            'pan_tilt_yaw_joint',
            'pan_tilt_pitch_joint',
        ]
        publisher._joint_state.position = [0.1, 0.2, 0.3]
        publisher._joint_state.velocity = [0.0, 0.0, 0.0]
        publisher._joint_state.effort = [0.0, 0.0, 0.0]

        base = RobotStatePublisherNode._build_base_state(publisher)
        arm = RobotStatePublisherNode._build_arm_state(publisher, 'left')
        head = RobotStatePublisherNode._build_head_state(publisher)

        self.assertEqual((base.header.stamp.sec, base.header.stamp.nanosec), (123, 456))
        self.assertEqual(base.header.frame_id, '')

        self.assertEqual((arm.header.stamp.sec, arm.header.stamp.nanosec), (789, 12))
        self.assertEqual(arm.header.frame_id, '')

        self.assertEqual((head.header.stamp.sec, head.header.stamp.nanosec), (789, 12))
        self.assertEqual(head.header.frame_id, '')

    def test_arm_state_only_reports_present_joint_samples(self):
        publisher = _make_publisher()
        publisher._joint_state = JointState()
        publisher._joint_state.name = ['left_joint1', 'left_joint3', 'left_gripper']
        publisher._joint_state.position = [0.1, 0.3, 0.5]
        publisher._joint_state.velocity = [0.01, 0.03, 0.05]
        publisher._joint_state.effort = [1.0, 3.0, 5.0]

        arm = RobotStatePublisherNode._build_arm_state(publisher, 'left')

        self.assertEqual(arm.joint_names, [f'left_joint{i}' for i in range(1, 7)])
        self.assertEqual(list(arm.joint_positions), [0.1, 0.3])
        self.assertEqual(list(arm.joint_velocities), [0.01, 0.03])
        self.assertEqual(list(arm.joint_efforts), [1.0, 3.0])
        self.assertEqual(arm.gripper.position, 0.5)


if __name__ == '__main__':
    unittest.main()
