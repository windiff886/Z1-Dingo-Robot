import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = REPO_ROOT / 'src' / 'uni_dingo_api_ros2'

if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time

from uni_dingo_api.api.arm_api import ArmAPI


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

    def get_clock(self):
        return self

    def now(self):
        return self

    def to_msg(self):
        return Time()


class _FakePlanResult:
    def __init__(self, trajectory='fake_trajectory'):
        self.trajectory = trajectory


class _FakeMoveGroup:
    def __init__(self, plan_result=None):
        self.plan_result = plan_result or _FakePlanResult()
        self.goal_calls = []
        self.start_state_calls = 0

    def set_start_state_to_current_state(self):
        self.start_state_calls += 1

    def set_goal_state(self, **kwargs):
        self.goal_calls.append(kwargs)

    def plan(self):
        return self.plan_result


class _FakeMoveIt:
    def __init__(self):
        self.execute_calls = []

    def execute(self, trajectory, controllers=None):
        self.execute_calls.append({
            'trajectory': trajectory,
            'controllers': controllers,
        })
        return True

    def get_robot_model(self):
        return object()


class ArmAPIMoveItBridgeTest(unittest.TestCase):
    def test_move_to_pose_uses_side_specific_controller(self):
        arm = ArmAPI.__new__(ArmAPI)
        arm._node = _FakeNode()
        arm._side = 'left'
        arm._enabled = True
        arm._ee_links = {'left': 'left_link06'}
        arm._move_groups = {'left': _FakeMoveGroup()}
        arm._moveit = _FakeMoveIt()
        arm._controllers = {'left': 'left_arm_controller'}
        arm._init_moveit = lambda: True

        future = arm.move_to_pose(Pose(), side='left')
        future.wait(timeout=1.0)

        self.assertTrue(future.is_success())
        self.assertEqual(
            [{'trajectory': 'fake_trajectory', 'controllers': ['left_arm_controller']}],
            arm._moveit.execute_calls,
        )

    def test_move_to_named_uses_right_side_controller(self):
        arm = ArmAPI.__new__(ArmAPI)
        arm._node = _FakeNode()
        arm._side = 'right'
        arm._enabled = True
        arm._move_groups = {'right': _FakeMoveGroup()}
        arm._moveit = _FakeMoveIt()
        arm._controllers = {'right': 'right_arm_controller'}
        arm._init_moveit = lambda: True

        future = arm.move_to_named('home', side='right')
        future.wait(timeout=1.0)

        self.assertTrue(future.is_success())
        self.assertEqual(
            [{'trajectory': 'fake_trajectory', 'controllers': ['right_arm_controller']}],
            arm._moveit.execute_calls,
        )
        self.assertEqual(
            [{'configuration_name': 'home'}],
            arm._move_groups['right'].goal_calls,
        )

    def test_move_both_to_named_uses_each_arm_controller(self):
        arm = ArmAPI.__new__(ArmAPI)
        arm._side = 'both'
        arm._enabled = True
        arm._move_groups = {
            'left': _FakeMoveGroup(),
            'right': _FakeMoveGroup(),
        }
        arm._moveit = _FakeMoveIt()
        arm._controllers = {
            'left': 'left_arm_controller',
            'right': 'right_arm_controller',
        }
        arm._init_moveit = lambda: True

        future = arm.move_to_named('home')
        future.wait(timeout=1.0)

        self.assertTrue(future.is_success())
        self.assertEqual(2, len(arm._moveit.execute_calls))
        self.assertEqual(
            [
                {'trajectory': 'fake_trajectory', 'controllers': ['left_arm_controller']},
                {'trajectory': 'fake_trajectory', 'controllers': ['right_arm_controller']},
            ],
            arm._moveit.execute_calls,
        )
        self.assertEqual(
            [{'configuration_name': 'home'}],
            arm._move_groups['left'].goal_calls,
        )
        self.assertEqual(
            [{'configuration_name': 'home'}],
            arm._move_groups['right'].goal_calls,
        )


if __name__ == '__main__':
    unittest.main()
