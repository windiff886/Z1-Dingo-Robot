import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = REPO_ROOT / 'src' / 'uni_dingo_api_ros2'

if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from geometry_msgs.msg import Pose, PointStamped

from uni_dingo_api.api.action_future import ActionFuture
from uni_dingo_api.api.arm_api import ArmAPI
from uni_dingo_api.api.head_api import HeadAPI
from uni_dingo_api.api.perception_api import PerceptionAPI
from uni_dingo_api.api.robot_api import RobotAPI


class _FakeLogger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


class _FakeNode:
    def get_logger(self):
        return _FakeLogger()


class _FakeFuture:
    def __init__(self, success=True):
        self._success = success
        self.wait_calls = []

    def wait(self, timeout=None):
        self.wait_calls.append(timeout)
        return self

    def is_success(self):
        return self._success


class _FakeArm:
    def __init__(self):
        self.pose_calls = []
        self.joint_calls = []
        self.enabled = True

    def move_to_pose(self, pose):
        self.pose_calls.append(pose)
        return _FakeFuture()

    def move_to_joint(self, joints):
        self.joint_calls.append(joints)
        return _FakeFuture()

    def is_enabled(self):
        return self.enabled


class _FakeBase:
    def __init__(self):
        self.move_to_calls = []
        self.enabled = True
        self._odom = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 12.5))
        )

    def move_to(self, x, y, theta):
        self.move_to_calls.append((x, y, theta))
        return _FakeFuture()

    def is_enabled(self):
        return self.enabled

    def is_moving(self):
        return False

    def get_pose_2d(self):
        return SimpleNamespace(x=1.0, y=2.0, theta=0.3)

    def get_velocity(self):
        return SimpleNamespace(
            linear=SimpleNamespace(x=0.1, y=0.2),
            angular=SimpleNamespace(z=0.3),
        )

    def get_odom(self):
        return self._odom

    def get_topics(self):
        return {'cmd_vel': '/cmd_vel', 'odom': '/odometry/filtered'}


class _FakeHead:
    def __init__(self, enabled=True):
        self.enabled = enabled

    def is_enabled(self):
        return self.enabled

    def get_angles(self):
        return (0.1, -0.2)

    def get_angles_deg(self):
        return (5.7, -11.5)

    def get_status(self):
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 9.5)))


class _FakePerception:
    def __init__(self, enabled=True):
        self.enabled = enabled
        self._latest_rgb = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 21.0))
        )

    def is_enabled(self):
        return self.enabled

    def has_data(self):
        return True

    def get_topics(self):
        return {
            'rgb': '/d455/color/image_raw',
            'depth': '/d455/aligned_depth_to_color/image_raw',
            'points': '/d455/depth/color/points',
            'camera_info': '/d455/color/camera_info',
        }


class ArmAPIContractTest(unittest.TestCase):
    def test_get_end_effector_pose_returns_pose_from_tf(self):
        arm = ArmAPI.__new__(ArmAPI)
        arm._node = _FakeNode()
        arm._side = 'left'
        arm._ee_links = {'left': 'left_link06', 'right': 'right_link06'}
        arm._tf_buffer = SimpleNamespace(
            lookup_transform=lambda target, source, time: SimpleNamespace(
                transform=SimpleNamespace(
                    translation=SimpleNamespace(x=0.1, y=0.2, z=0.3),
                    rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            )
        )

        pose = arm.get_end_effector_pose(frame='base_link')

        self.assertIsNotNone(pose)
        self.assertEqual(pose.position.x, 0.1)
        self.assertEqual(pose.position.y, 0.2)
        self.assertEqual(pose.position.z, 0.3)
        self.assertEqual(pose.orientation.w, 1.0)

    def test_move_both_dispatches_dual_pose_motion(self):
        left_pose = Pose()
        right_pose = Pose()

        arm = ArmAPI.__new__(ArmAPI)
        arm._side = 'both'
        arm._enabled = True
        arm.move_to_pose_calls = []

        def fake_move_to_pose(pose, frame='base_link', side=None):
            arm.move_to_pose_calls.append((pose, frame, side))
            return ActionFuture.success()

        arm.move_to_pose = fake_move_to_pose

        future = arm.move_both(left_pose, right_pose, frame='base_link')
        future.wait(timeout=1.0)

        self.assertTrue(future.is_success())
        self.assertEqual(arm.move_to_pose_calls, [
            (left_pose, 'base_link', 'left'),
            (right_pose, 'base_link', 'right'),
        ])


class HeadAPIContractTest(unittest.TestCase):
    def test_look_at_accepts_ros1_frame_keyword(self):
        head = HeadAPI.__new__(HeadAPI)
        head._enabled = False

        future = HeadAPI.look_at(head, PointStamped(), frame='map')

        self.assertFalse(future.is_success())
        self.assertEqual(future.get_error(), 'Head disabled')

    def test_is_moving_matches_ros1_stub_behavior(self):
        head = HeadAPI.__new__(HeadAPI)

        self.assertFalse(head.is_moving())


class PerceptionAPIContractTest(unittest.TestCase):
    def test_preserves_ros1_optical_frame_config(self):
        with patch('uni_dingo_api.api.perception_api.tf2_ros.Buffer', return_value=SimpleNamespace()), \
                patch('uni_dingo_api.api.perception_api.tf2_ros.TransformListener', return_value=None):
            perception = PerceptionAPI(
                node=_FakeNode(),
                mode='real',
                config={'optical_frame': 'camera_color_optical_frame'},
            )

        self.assertEqual(perception._optical_frame, 'camera_color_optical_frame')


class RobotAPIContractTest(unittest.TestCase):
    def test_is_ready_considers_emergency_stop_and_enabled_subsystems(self):
        robot = RobotAPI.__new__(RobotAPI)
        robot._emergency_stopped = False
        robot.base = _FakeBase()
        robot.left_arm = _FakeArm()
        robot.right_arm = _FakeArm()
        robot.head = _FakeHead(enabled=False)
        robot.perception = _FakePerception(enabled=False)

        self.assertTrue(robot.is_ready())

        robot.right_arm.enabled = False
        self.assertFalse(robot.is_ready())

        robot.right_arm.enabled = True
        robot._emergency_stopped = True
        self.assertFalse(robot.is_ready())

    def test_get_state_matches_ros1_shape_for_head_and_perception_metadata(self):
        robot = RobotAPI.__new__(RobotAPI)
        robot._mode = 'real'
        robot._emergency_stopped = False
        robot.base = _FakeBase()
        robot.left_arm = SimpleNamespace(
            is_enabled=lambda: True,
            is_moving=lambda: False,
            get_joint_states=lambda: {
                'positions': [0.1] * 6,
                'timestamp': 5.0,
            },
            get_end_effector_pose=lambda: SimpleNamespace(
                position=SimpleNamespace(x=0.1, y=0.2, z=0.3),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
        robot.right_arm = None
        robot.head = _FakeHead(enabled=True)
        robot.perception = _FakePerception(enabled=True)

        state = robot.get_state()

        self.assertIn('timestamp', state)
        self.assertEqual(state['base']['timestamp'], 12.5)
        self.assertEqual(state['left_arm']['timestamp'], 5.0)
        self.assertEqual(state['head']['pan_deg'], 5.7)
        self.assertEqual(state['head']['tilt_deg'], -11.5)
        self.assertEqual(state['head']['timestamp'], 9.5)
        self.assertEqual(state['perception']['topics']['rgb'], '/d455/color/image_raw')
        self.assertEqual(state['perception']['timestamp'], 21.0)

    def test_whole_body_move_sequences_base_and_arm_goals(self):
        robot = RobotAPI.__new__(RobotAPI)
        robot._emergency_stopped = False
        robot.base = _FakeBase()
        robot.left_arm = _FakeArm()
        robot.right_arm = _FakeArm()

        right_pose = Pose()
        future = robot.whole_body_move(
            base_goal=(1.0, 2.0, 0.3),
            left_arm_goal=[0.1] * 6,
            right_arm_goal=right_pose,
        )
        future.wait(timeout=1.0)

        self.assertTrue(future.is_success())
        self.assertEqual(robot.base.move_to_calls, [(1.0, 2.0, 0.3)])
        self.assertEqual(robot.left_arm.joint_calls, [[0.1] * 6])
        self.assertEqual(robot.right_arm.pose_calls, [right_pose])

    def test_whole_body_move_rejects_emergency_stop(self):
        robot = RobotAPI.__new__(RobotAPI)
        robot._emergency_stopped = True

        future = robot.whole_body_move()

        self.assertTrue(future.is_done())
        self.assertFalse(future.is_success())
        self.assertIn('Emergency', future.get_error())


if __name__ == '__main__':
    unittest.main()
