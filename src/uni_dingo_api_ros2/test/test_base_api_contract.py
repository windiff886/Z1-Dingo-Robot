import importlib.util
import math
import sys
import unittest
from pathlib import Path

from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry


REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = REPO_ROOT / 'src' / 'uni_dingo_api_ros2'
BASE_LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'includes' / 'base.launch.py'
REAL_LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'real.launch.py'

if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from uni_dingo_api.api.base_api import BaseAPI


class _FakeLogger:
    def __init__(self):
        self.warnings = []

    def info(self, *_args, **_kwargs):
        pass

    def warn(self, message, *_args, **_kwargs):
        self.warnings.append(message)


class _FakePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _FakeNode:
    def __init__(self):
        self.logger = _FakeLogger()
        self.publishers = {}
        self.subscriptions = {}

    def create_publisher(self, _msg_type, topic, _qos):
        publisher = _FakePublisher(topic)
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, _msg_type, topic, callback, _qos, **_kwargs):
        self.subscriptions[topic] = callback
        return callback

    def destroy_subscription(self, _subscription):
        return True

    def get_logger(self):
        return self.logger


class BaseLaunchContractTest(unittest.TestCase):
    def test_vendored_clearpath_base_launch_is_preferred(self):
        spec = importlib.util.spec_from_file_location('base_launch', BASE_LAUNCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        launch_path = module.resolve_base_launch_artifacts()

        self.assertEqual(launch_path.name, 'platform.launch.py')
        self.assertIn('/src/dingo_ros2/', str(launch_path))
        self.assertTrue(launch_path.exists())

    def test_missing_clearpath_package_fails_fast(self):
        spec = importlib.util.spec_from_file_location('base_launch', BASE_LAUNCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        with self.assertRaises(RuntimeError) as ctx:
            module.resolve_base_launch_artifacts(
                package_name='missing_clearpath_package',
                launch_file='robot.launch.py',
            )

        self.assertIn('missing_clearpath_package', str(ctx.exception))

    def test_base_adapter_details_are_not_exposed_as_real_launch_arguments(self):
        real_launch_text = REAL_LAUNCH_PATH.read_text(encoding='utf-8')
        base_launch_text = BASE_LAUNCH_PATH.read_text(encoding='utf-8')

        for hidden_arg in [
            'base_package',
            'base_launch_file',
            'base_namespace',
            'base_config_path',
            'base_config_arg_name',
        ]:
            with self.subTest(hidden_arg=hidden_arg):
                self.assertNotIn(f"'{hidden_arg}'", real_launch_text)
                self.assertNotIn(f"'{hidden_arg}'", base_launch_text)


class BaseAPIContractTest(unittest.TestCase):
    def test_move_publishes_clamped_cmd_vel(self):
        node = _FakeNode()
        api = BaseAPI(node=node, config={
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odometry/filtered',
            'max_linear_vel': 1.0,
            'max_angular_vel': 2.0,
        })

        api.move(linear_x=5.0, linear_y=-5.0, angular_z=4.0)

        publisher = node.publishers['/cmd_vel']
        self.assertEqual(len(publisher.messages), 1)
        msg = publisher.messages[-1]
        self.assertEqual(msg.linear.x, 1.0)
        self.assertEqual(msg.linear.y, -1.0)
        self.assertEqual(msg.angular.z, 2.0)

    def test_odom_subscription_updates_pose_and_velocity(self):
        node = _FakeNode()
        api = BaseAPI(node=node, config={
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odometry/filtered',
        })

        odom = Odometry()
        odom.pose.pose.position.x = 1.2
        odom.pose.pose.position.y = -0.4
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=math.sin(0.5 / 2.0), w=math.cos(0.5 / 2.0))
        odom.twist.twist.linear.x = 0.3
        odom.twist.twist.angular.z = 0.2

        node.subscriptions['/odometry/filtered'](odom)

        pose = api.get_pose_2d()
        velocity = api.get_velocity()
        self.assertAlmostEqual(pose.x, 1.2)
        self.assertAlmostEqual(pose.y, -0.4)
        self.assertAlmostEqual(pose.theta, 0.5, places=4)
        self.assertAlmostEqual(velocity.linear.x, 0.3)
        self.assertAlmostEqual(velocity.angular.z, 0.2)
        self.assertTrue(api.is_moving())

    def test_get_topics_reports_configured_topics(self):
        node = _FakeNode()
        api = BaseAPI(node=node, config={
            'cmd_vel_topic': '/base/cmd_vel',
            'odom_topic': '/base/odom',
        })

        self.assertEqual(api.get_topics(), {
            'cmd_vel': '/base/cmd_vel',
            'odom': '/base/odom',
        })

    def test_move_to_accepts_ros1_frame_keyword(self):
        node = _FakeNode()
        api = BaseAPI(node=node, config={
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odometry/filtered',
        })
        api._enabled = False

        future = api.move_to(1.0, 2.0, 0.3, frame='map')

        self.assertFalse(future.is_success())
        self.assertEqual(future.get_error(), 'Base disabled or emergency stopped')


if __name__ == '__main__':
    unittest.main()
