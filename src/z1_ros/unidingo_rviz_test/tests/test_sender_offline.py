#!/usr/bin/env python3
"""Test sender decisions with in-memory ROS API stand-ins, without ROS or sockets.

These checks do not validate ROS serialization, tf2 implementation or RViz UI.
"""
import copy
import importlib.util
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch


class Time:
    def __init__(self, seconds=0):
        self.seconds = seconds

    @staticmethod
    def now():
        return Time(1000)

    def is_zero(self):
        return self.seconds == 0

    def to_sec(self):
        return self.seconds

    def __sub__(self, other):
        return Time(self.seconds - other.seconds)


class Header:
    def __init__(self, stamp=None, frame_id=''):
        self.stamp = stamp or Time()
        self.frame_id = frame_id


def pose():
    return NS(position=NS(x=0.3, y=0.0, z=0.2), orientation=NS(x=0., y=0., z=0., w=1.))


class PoseStamped:
    def __init__(self, header=None, pose=None):
        self.header = header or Header()
        self.pose = pose or globals()['pose']()


class PoseGoal:
    def __init__(self):
        self.header = Header()


class ControlCommand(PoseGoal):
    ENABLE, DISABLE, STOP, RESUME, JOINT_STREAM, POSE_STREAM, IMPEDANCE, HOLD = range(8)


class Status:
    DISABLED, HOLDING, JOINT_GOAL, POSE_GOAL, JOINT_STREAM, POSE_STREAM, IMPEDANCE, STOPPING, AWAIT_RESUME, FAULT = range(10)


class MotionResult:
    ACCEPTED, SUCCEEDED, REJECTED, CANCELED, STOPPED, FAILED = range(6)


class JointState:
    def __init__(self, header=None, name=None, position=None):
        self.header, self.name, self.position = header or Header(), name or [], position or []


class Marker:
    SPHERE, TEXT_VIEW_FACING, ADD = 2, 9, 0

    def __init__(self, **kwargs):
        self.header, self.pose = Header(), pose()
        self.scale, self.color = NS(x=0, y=0, z=0), NS(r=0, g=0, b=0, a=0)
        self.__dict__.update(kwargs)


class MarkerArray:
    def __init__(self):
        self.markers = []


class InteractiveMarker:
    def __init__(self):
        self.header, self.controls = Header(), []


class InteractiveMarkerControl:
    MENU, ROTATE_AXIS, MOVE_AXIS, FIXED = 1, 5, 3, 1

    def __init__(self, **kwargs):
        self.orientation = NS(x=0, y=0, z=0, w=0)
        self.markers = []
        self.__dict__.update(kwargs)


class Feedback:
    POSE_UPDATE, MOUSE_UP = 1, 5


class Publisher:
    def __init__(self, topic, *_args, **_kwargs):
        self.topic, self.messages, self.connections = topic, [], 1

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))

    def get_num_connections(self):
        return self.connections


class Menu:
    def __init__(self):
        self.actions = {}

    def insert(self, label, callback):
        self.actions[label] = callback

    def apply(self, *_args):
        pass


def load_sender():
    modules = {}

    def module(name, **attributes):
        result = ModuleType(name)
        result.__dict__.update(attributes)
        modules[name] = result
        return result

    module('rospy', Publisher=Publisher, Subscriber=Mock(), Timer=Mock(),
           Time=Time, Duration=Time, get_param=lambda name, default: default,
           resolve_name=lambda name: name.replace('~', '/rviz_tcp_test/'),
           loginfo=Mock(), logwarn=Mock())
    module('tf2_ros', Buffer=Mock, TransformListener=Mock, TransformException=LookupError)
    module('tf2_geometry_msgs')
    module('geometry_msgs')
    module('geometry_msgs.msg', PoseStamped=PoseStamped)
    module('interactive_markers')
    module('interactive_markers.interactive_marker_server', InteractiveMarkerServer=lambda *_args: Mock())
    module('interactive_markers.menu_handler', MenuHandler=Menu)
    module('sensor_msgs')
    module('sensor_msgs.msg', JointState=JointState)
    module('std_msgs')
    module('std_msgs.msg', Header=Header)
    module('visualization_msgs')
    module('visualization_msgs.msg', InteractiveMarker=InteractiveMarker,
           InteractiveMarkerControl=InteractiveMarkerControl,
           InteractiveMarkerFeedback=Feedback, Marker=Marker, MarkerArray=MarkerArray)
    module('z1_msgs')
    module('z1_msgs.msg', ControlCommand=ControlCommand, MotionResult=MotionResult,
           PoseGoal=PoseGoal, Status=Status)
    path = Path(__file__).resolve().parents[1] / 'scripts/interactive_tcp_test.py'
    spec = importlib.util.spec_from_file_location('tcp_sender_under_test', path)
    sender = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(sender)
    return sender


SENDER = load_sender()


class SenderDecisions(unittest.TestCase):
    def setUp(self):
        self.node = SENDER.TcpTest()
        self.arm = self.node.arms['left']
        self.refresh()
        self.arm.target = PoseStamped(Header(Time.now(), 'base_link'))
        self.node.buffer.transform.side_effect = lambda msg, frame, timeout: PoseStamped(Header(Time(), frame), copy.deepcopy(msg.pose))

    def refresh(self):
        self.node.on_status(self.arm, NS(header=Header(Time.now(), self.arm.base),
            mode=Status.HOLDING, feedback_valid=True, active_command_id=0, reason='holding'))
        self.node.on_tcp(self.arm, PoseStamped(Header(Time.now(), self.arm.base)))

    def feedback(self, event):
        return NS(event_type=event, header=Header(Time.now(), 'base_link'), pose=pose())

    def test_startup_has_no_motion_or_implicit_enable(self):
        self.assertFalse(self.arm.goal_pub.messages)
        self.assertFalse(self.arm.control_pub.messages)
        self.assertFalse(self.arm.created)

    def test_both_arms_send_transformed_pose_in_own_base(self):
        for side in ('left', 'right'):
            self.arm = self.node.arms[side]
            self.refresh()
            self.arm.target = PoseStamped(Header(Time.now(), 'base_link'))
            transformed = pose()
            transformed.position.x, transformed.position.y = 0.123, -0.456
            transformed.orientation.z, transformed.orientation.w = 0.6, 0.8
            self.node.buffer.transform.side_effect = None
            self.node.buffer.transform.return_value = PoseStamped(Header(Time(), self.arm.base), transformed)
            self.node.send(self.arm)
            sent = self.arm.goal_pub.messages[-1]
            self.assertEqual(sent.header.frame_id, side + '_link00')
            self.assertFalse(sent.header.stamp.is_zero())
            self.assertEqual(sent.pose, transformed)
            self.assertEqual(sent.speed_scale, 0.15)
            args = self.node.buffer.transform.call_args.args
            self.assertEqual(args[1], self.arm.base)
            self.assertTrue(args[0].header.stamp.is_zero())
            self.assertEqual(self.arm.target.header.stamp.seconds, 1000)  # Input not mutated.
            self.assertEqual(self.arm.pending, sent.command_id)

    def test_modes_and_pending_reject_without_queue(self):
        for mode in (Status.DISABLED, Status.JOINT_GOAL, Status.POSE_GOAL,
                     Status.POSE_STREAM, Status.STOPPING, Status.AWAIT_RESUME, Status.FAULT):
            self.arm.status.mode = mode
            with self.assertRaises(ValueError):
                self.node.send(self.arm)
        self.arm.status.mode = Status.HOLDING
        self.node.send(self.arm)
        with self.assertRaises(ValueError):
            self.node.send(self.arm)
        self.assertEqual(len(self.arm.goal_pub.messages), 1)

    def test_stale_and_invalid_telemetry_block_motion(self):
        changes = (
            lambda: setattr(self.arm.status, 'feedback_valid', False),
            lambda: setattr(self.arm, 'status_at', time.monotonic() - 1),
            lambda: setattr(self.arm, 'tcp_at', time.monotonic() - 1),
            lambda: setattr(self.arm.status.header, 'stamp', Time(999)),
            lambda: setattr(self.arm.tcp.header, 'stamp', Time(1001)),
            lambda: setattr(self.arm.tcp.header, 'stamp', Time(0)),
        )
        for change in changes:
            self.refresh()
            change()
            with self.assertRaises(ValueError):
                self.node.send(self.arm)
        self.assertFalse(self.arm.goal_pub.messages)

    def test_invalid_quaternion_or_position_rejected(self):
        for field, value in (('w', 0), ('w', float('nan')), ('x', float('inf'))):
            p = pose()
            setattr(p.orientation, field, value)
            self.assertFalse(SENDER.valid_pose(p))
        self.arm.target.pose.position.z = float('nan')
        with self.assertRaises(ValueError):
            self.node.send(self.arm)

    def test_missing_subscriber_or_tf_never_sets_pending(self):
        self.arm.goal_pub.connections = 0
        with self.assertRaises(ValueError):
            self.node.send(self.arm)
        self.arm.goal_pub.connections = 1
        self.node.buffer.transform.side_effect = LookupError('missing TF')
        self.node.handle(self.arm, lambda: self.node.send(self.arm))
        self.assertIsNone(self.arm.pending)
        self.assertFalse(self.arm.goal_pub.messages)

    def test_release_sends_once_and_never_streams_updates(self):
        self.node.feedback(self.arm, self.feedback(Feedback.POSE_UPDATE))
        self.assertFalse(self.arm.goal_pub.messages)
        self.node.feedback(self.arm, self.feedback(Feedback.MOUSE_UP))
        self.node.feedback(self.arm, self.feedback(Feedback.MOUSE_UP))
        self.assertEqual(len(self.arm.goal_pub.messages), 1)

    def test_manual_send_and_bad_feedback(self):
        self.node.send_on_release = False
        self.node.feedback(self.arm, self.feedback(Feedback.POSE_UPDATE))
        self.node.feedback(self.arm, self.feedback(Feedback.MOUSE_UP))
        self.assertFalse(self.arm.goal_pub.messages)
        self.node.send(self.arm)
        self.assertEqual(len(self.arm.goal_pub.messages), 1)
        bad = self.feedback(Feedback.POSE_UPDATE)
        bad.header.frame_id = 'map'
        self.node.feedback(self.arm, bad)
        self.assertIn('invalid marker frame', self.arm.note)

    def test_results_never_retry_or_replay_a_goal(self):
        self.node.send(self.arm)
        command_id = self.arm.pending
        for outcome, identity in ((MotionResult.ACCEPTED, command_id), (MotionResult.SUCCEEDED, command_id + 10)):
            self.node.on_result(self.arm, NS(command_id=identity, outcome=outcome, reason='test'))
            self.assertEqual(self.arm.pending, command_id)
        self.node.on_result(self.arm, NS(command_id=command_id, outcome=MotionResult.REJECTED, reason='IK failed'))
        self.assertIsNone(self.arm.pending)
        self.node.tick(None)
        self.assertEqual(len(self.arm.goal_pub.messages), 1)

    def test_stop_disable_work_without_feedback_other_operations_do_not(self):
        self.arm.status.feedback_valid = False
        for operation in (ControlCommand.STOP, ControlCommand.DISABLE):
            self.node.operate(self.arm, operation)
        for operation in (ControlCommand.ENABLE, ControlCommand.HOLD, ControlCommand.RESUME):
            with self.assertRaises(ValueError):
                self.node.operate(self.arm, operation)
        self.assertEqual(len(self.arm.control_pub.messages), 2)

    def test_marker_starts_at_measured_tcp_and_menu_callbacks_are_distinct(self):
        self.arm.tcp.pose.position.x = 0.417
        self.node.create_marker(self.arm)
        marker = self.node.server.insert.call_args.args[0]
        self.assertEqual(marker.pose.position.x, 0.417)
        self.assertEqual(len(marker.controls), 7)
        self.arm.menu.actions['ENABLE (Disabled -> Holding)'](None)
        self.arm.menu.actions['STOP'](None)
        self.assertEqual([m.operation for m in self.arm.control_pub.messages], [ControlCommand.ENABLE, ControlCommand.STOP])

    def test_joint_display_keeps_feedback_stamp_and_has_no_synthetic_arm_values(self):
        names = ['left_joint' + str(i) for i in range(1, 7)]
        msg = JointState(Header(Time(997)), names + ['left_gripper_motor'], [0.1] * 7)
        self.node.on_joints(self.arm, msg)
        self.assertEqual(self.node.joints.messages[-1].header.stamp.seconds, 997)
        self.assertEqual(self.node.joints.messages[-1].name, names)
        self.arm.status.feedback_valid = False
        self.node.on_joints(self.arm, msg)
        self.assertEqual(len(self.node.joints.messages), 1)
        self.node.on_aux(JointState(name=['left_joint1', 'pan_tilt_yaw_joint'], position=[99, 0.2]))
        self.node.tick(None)
        auxiliary = self.node.joints.messages[-1]
        self.assertNotIn('left_joint1', auxiliary.name)
        self.assertEqual(auxiliary.position[auxiliary.name.index('pan_tilt_yaw_joint')], 0.2)


if __name__ == '__main__':
    unittest.main()
