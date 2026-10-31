#!/usr/bin/env python3
"""Exercise the SDK adapter without ROS nodes, RViz, network or hardware."""
import copy
import importlib.util
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((PACKAGE / 'config/rviz_tcp_test.yaml').read_text())
START = [0., .8, -1.1, .3, 0., 0.]


class Time:
    def __init__(self, seconds=0): self.seconds = seconds
    @staticmethod
    def now(): return Time(1000)
    def is_zero(self): return self.seconds == 0
    def to_sec(self): return self.seconds
    def __sub__(self, other): return Time(self.seconds - other.seconds)


class Header:
    def __init__(self, stamp=None, frame_id=''):
        self.stamp, self.frame_id = stamp or Time(), frame_id


def pose(): return NS(position=NS(x=.3, y=0., z=.2), orientation=NS(x=0., y=0., z=0., w=1.))


class Message:
    def __init__(self, **kwargs): self.__dict__.update(kwargs)


class PoseStamped:
    def __init__(self, header=None, pose=None):
        self.header, self.pose = header or Header(), pose or globals()['pose']()


class PoseGoal(Message):
    def __init__(self, **kwargs):
        self.header = Header()
        super().__init__(**kwargs)


class ControlCommand(PoseGoal):
    ENABLE, DISABLE, STOP, RESUME, JOINT_STREAM, POSE_STREAM, IMPEDANCE, HOLD = range(8)


class Status:
    DISABLED, HOLDING, JOINT_GOAL, POSE_GOAL, JOINT_STREAM, POSE_STREAM, IMPEDANCE, STOPPING, AWAIT_RESUME, FAULT = range(10)


class MotionResult:
    ACCEPTED, SUCCEEDED, REJECTED, CANCELED, STOPPED, FAILED = range(6)


class JointState:
    def __init__(self, header=None, name=None, position=None):
        self.header, self.name, self.position = header or Header(), name or [], position or []


class RobotState:
    def __init__(self): self.joint_state = JointState()


class DisplayTrajectory:
    def __init__(self): self.trajectory_start, self.trajectory = RobotState(), []


class Publisher:
    def __init__(self, topic, *_args, **_kwargs):
        self.topic, self.messages, self.connections = topic, [], 1
    def publish(self, message): self.messages.append(copy.deepcopy(message))
    def get_num_connections(self): return self.connections


class ServiceError(Exception): pass


class Future:
    def __init__(self, callback, request): self.callback, self.request, self.finished = callback, request, False
    def done(self): return self.finished
    def complete(self):
        self.finished = True
        try: self.response = self.callback(self.request)
        except ServiceError as error: self.response = error
    def result(self):
        if isinstance(self.response, ServiceError): raise self.response
        return self.response


class Executor:
    def __init__(self, **_kwargs): self.jobs = []
    def submit(self, callback, request):
        result = Future(callback, request); self.jobs.append(result); return result
    def shutdown(self, **_kwargs): pass


def load_sender():
    modules = {}
    def module(name, **attributes):
        result = ModuleType(name); result.__dict__.update(attributes); modules[name] = result
    module('rospy', Publisher=Publisher, Subscriber=Mock(), Timer=Mock(), on_shutdown=Mock(),
           Time=Time, Duration=Time, ServiceProxy=lambda *_args: Mock(), ServiceException=ServiceError,
           get_param=lambda name: CONFIG[name[1:]], loginfo=Mock(), logwarn=Mock())
    for name in ('geometry_msgs', 'sensor_msgs', 'std_msgs', 'z1_msgs', 'moveit_msgs', 'unidingo_rviz_test'): module(name)
    module('geometry_msgs.msg', PoseStamped=PoseStamped)
    module('sensor_msgs.msg', JointState=JointState)
    module('std_msgs.msg', Header=Header, String=Message)
    module('z1_msgs.msg', ControlCommand=ControlCommand, MotionResult=MotionResult, PoseGoal=PoseGoal, Status=Status)
    module('moveit_msgs.msg', DisplayTrajectory=DisplayTrajectory, RobotState=RobotState, RobotTrajectory=Message)
    module('unidingo_rviz_test.msg', EditorCommand=Message, EditorTarget=Message, PreviewStatus=Message)
    module('unidingo_rviz_test.srv', PreviewPose=Message, PreviewPoseRequest=Message)
    spec = importlib.util.spec_from_file_location('sdk_adapter', PACKAGE / 'scripts/interactive_tcp_test.py')
    sender = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules): spec.loader.exec_module(sender)
    sender.ThreadPoolExecutor = Executor
    return sender


SENDER = load_sender()


class AdapterContract(unittest.TestCase):
    def setUp(self):
        self.node = SENDER.TcpTest()
        for arm in self.node.arms.values():
            self.refresh(arm)
            arm.preview_client.side_effect = self.response
        self.arm = self.node.arms['left']

    def refresh(self, arm):
        self.node.on_status(arm, NS(header=Header(Time.now(), arm.base), mode=Status.HOLDING,
            feedback_valid=True, active_command_id=0, reason='holding', model_revision='model-v1'))
        self.node.on_tcp(arm, PoseStamped(Header(Time.now(), arm.base)))
        joints = JointState(Header(Time.now(), arm.base), arm.names, list(START))
        self.node.on_joints(arm, joints); self.node.on_reference(arm, joints)

    def target(self, arm=None, editing=True, valid=True):
        arm = arm or self.arm
        return NS(header=Header(Time.now(), arm.base), arm=arm.side, revision=arm.editor_revision+1, editing=editing, valid=valid, pose=pose())

    def response(self, request):
        goal = list(request.start_positions); goal[0] += .02
        return NS(success=True, reason='', model_revision='model-v1', start_tolerance=.015,
            trajectory=NS(header=request.header, joint_names=[request.header.frame_id[:-6]+'joint'+str(i) for i in range(1,7)], points=[
                NS(positions=list(request.start_positions), time_from_start=Time()), NS(positions=goal, time_from_start=Time(1))]))

    def preview(self, arm=None):
        arm = arm or self.arm
        self.node.on_target(arm, self.target(arm)); self.node.update_preview(arm)
        self.node.executor.jobs[-1].complete(); self.node.update_preview(arm)
        self.assertTrue(self.node.ready_to_send(arm))

    def test_startup_does_not_command_or_plan(self):
        self.node.tick(None)
        self.assertFalse(self.node.executor.jobs)
        for arm in self.node.arms.values():
            self.assertFalse(arm.goal_pub.messages); self.assertFalse(arm.control_pub.messages)

    def test_moveit_goal_only_requests_readonly_preview(self):
        self.preview()
        self.assertFalse(self.arm.goal_pub.messages)
        self.assertEqual(self.arm.preview_client.call_args.args[0].header.frame_id, self.arm.base)
        self.assertEqual(self.arm.preview_client.call_args.args[0].target_pose, self.arm.target.pose)

    def test_send_publishes_once_with_sdk_base_id_and_speed(self):
        self.preview(); self.node.send(self.arm)
        msg = self.arm.goal_pub.messages[0]
        self.assertEqual(msg.header.frame_id, self.arm.base)
        self.assertEqual(msg.pose, self.arm.target.pose)
        self.assertEqual(msg.speed_scale, .05)
        self.assertEqual(msg.command_id, self.arm.sent_id)
        self.assertFalse(self.node.ready_to_send(self.arm))

    def test_double_send_cannot_overwrite_the_first_command(self):
        self.preview(); self.node.send(self.arm); first = self.arm.sent_id
        with self.assertRaises(ValueError): self.node.send(self.arm)
        self.assertEqual(self.arm.sent_id, first); self.assertEqual(len(self.arm.goal_pub.messages), 1)
        self.node.on_result(self.arm, NS(command_id=first, outcome=MotionResult.SUCCEEDED, reason='arrived'))
        self.assertEqual(self.node.editor.messages[-1].operation, 'reset')
        self.assertEqual(self.node.editor.messages[-1].expected_revision, self.arm.sent_editor_revision)
        self.assertEqual(self.arm.sent_id, 0)

    def test_accepted_result_keeps_pending_command(self):
        self.preview(); self.node.send(self.arm); first = self.arm.sent_id
        self.node.on_result(self.arm, NS(command_id=first, outcome=MotionResult.ACCEPTED, reason='accepted'))
        self.assertEqual(self.arm.sent_id, first); self.assertFalse(self.node.ready_to_send(self.arm))

    def test_success_preserves_a_newly_edited_goal(self):
        self.preview(); self.node.send(self.arm); first = self.arm.sent_id
        self.node.on_target(self.arm, self.target()); before = len(self.node.editor.messages)
        self.node.on_result(self.arm, NS(command_id=first, outcome=MotionResult.SUCCEEDED, reason='arrived'))
        self.assertEqual(len(self.node.editor.messages), before)
        self.assertFalse(self.arm.following); self.assertTrue(self.arm.requested)

    def test_tab_ack_without_a_new_native_edit_still_allows_success_reset(self):
        self.preview(); self.node.send(self.arm); first=self.arm.sent_id
        revision=self.arm.editor_revision
        self.node.on_action(NS(data='left/select'))
        ack=self.target(); ack.revision=revision
        self.node.on_target(self.arm,ack)
        self.node.on_result(self.arm,NS(command_id=first,outcome=MotionResult.SUCCEEDED,reason='arrived'))
        self.assertEqual(self.node.editor.messages[-1].operation,'reset')
        self.assertEqual(self.node.editor.messages[-1].expected_revision,revision)

    def test_rejected_command_keeps_target_without_replay(self):
        self.preview(); self.node.send(self.arm)
        self.node.on_result(self.arm, NS(command_id=self.arm.sent_id, outcome=MotionResult.REJECTED, reason='busy'))
        self.node.tick(None)
        self.assertEqual(len(self.arm.goal_pub.messages), 1); self.assertEqual(self.arm.sent_id, 0)
        self.assertFalse(self.arm.following)

    def test_unrelated_zero_result_does_not_change_editor(self):
        self.preview(); note = self.arm.note
        self.node.on_result(self.arm, NS(command_id=0, outcome=MotionResult.ACCEPTED, reason='control'))
        self.assertEqual(note, self.arm.note)

    def test_source_age_blocks_disabled_preview_start(self):
        self.arm.status.mode = Status.DISABLED; self.arm.positions = None
        self.node.on_joints(self.arm, JointState(Header(Time(990)), self.arm.names, [.2] * 6))
        self.assertIsNone(self.arm.positions)
        with self.assertRaises(ValueError): self.node.start_positions(self.arm)

    def test_delayed_source_does_not_gain_a_full_new_lifetime(self):
        stamp = Time(999.8)
        self.node.on_joints(self.arm, JointState(Header(stamp), self.arm.names, list(START)))
        self.assertGreaterEqual(time.monotonic() - self.arm.positions_at, .19)
        self.assertEqual(self.node.joints.messages[-1].header.stamp.seconds, stamp.seconds)

    def test_zero_future_and_stale_joint_timestamps_are_rejected(self):
        for stamp in (Time(), Time(1001), Time(999)):
            self.arm.positions = self.arm.reference = None
            msg = JointState(Header(stamp), self.arm.names, list(START))
            self.node.on_joints(self.arm, msg); self.node.on_reference(self.arm, msg)
            self.assertIsNone(self.arm.positions); self.assertIsNone(self.arm.reference)

    def test_home_and_reset_are_official_editor_actions_not_commands(self):
        for action in ('both/home', 'left/home', 'right/reset'): self.node.on_action(NS(data=action))
        self.assertEqual([m.arm+'/'+m.operation for m in self.node.editor.messages if m.operation != 'clear_path'], ['left/home','right/home','left/home','right/reset'])
        self.assertFalse(self.arm.goal_pub.messages); self.assertFalse(self.node.executor.jobs)

    def test_measured_editor_updates_do_not_request_preview(self):
        self.node.on_target(self.arm, self.target(editing=False)); self.node.tick(None)
        self.assertTrue(self.arm.following); self.assertFalse(self.node.executor.jobs)

    def test_invalid_moveit_ik_clears_sendable_preview(self):
        self.preview(); self.node.on_target(self.arm, self.target(valid=False)); self.node.tick(None)
        self.assertFalse(self.node.ready_to_send(self.arm)); self.assertFalse(self.arm.requested)
        self.assertIn('cannot reach', self.arm.note)

    def test_reset_during_inflight_preview_ignores_old_result(self):
        self.node.on_target(self.arm, self.target()); self.node.update_preview(self.arm)
        future = self.node.executor.jobs[-1]
        self.node.on_target(self.arm, self.target(editing=False)); future.complete(); self.node.update_preview(self.arm)
        self.assertIsNone(self.arm.preview); self.assertTrue(self.arm.following)

    def test_new_goal_during_inflight_preview_ignores_old_result(self):
        self.node.on_target(self.arm, self.target()); self.node.update_preview(self.arm)
        future = self.node.executor.jobs[-1]
        self.node.on_target(self.arm, self.target()); future.complete(); self.node.update_preview(self.arm)
        self.assertIsNone(self.arm.preview); self.assertIsNotNone(self.arm.future)

    def test_start_move_during_rpc_triggers_new_reference(self):
        self.node.on_target(self.arm, self.target()); self.node.update_preview(self.arm)
        self.arm.reference[0] += .1
        self.node.executor.jobs[-1].complete(); self.node.update_preview(self.arm)
        self.assertIsNone(self.arm.preview); self.assertIsNotNone(self.arm.future)

    def test_feedback_loss_after_rpc_does_not_enable_send(self):
        self.node.on_target(self.arm, self.target()); self.node.update_preview(self.arm)
        self.arm.status.feedback_valid = False
        self.node.executor.jobs[-1].complete(); self.node.update_preview(self.arm)
        self.assertIsNone(self.arm.preview); self.assertTrue(self.arm.requested)

    def test_preview_model_must_match_controller(self):
        self.arm.preview_client.side_effect = lambda request: NS(success=True, model_revision='other')
        self.node.on_target(self.arm, self.target()); self.node.update_preview(self.arm)
        self.node.executor.jobs[-1].complete(); self.node.update_preview(self.arm)
        self.assertFalse(self.node.ready_to_send(self.arm)); self.assertIn('model differs',self.arm.note)

    def test_service_failure_is_reported_without_a_command(self):
        self.arm.preview_client.side_effect = ServiceError('offline')
        self.node.on_target(self.arm, self.target()); self.node.update_preview(self.arm)
        self.node.executor.jobs[-1].complete(); self.node.update_preview(self.arm)
        self.assertIn('offline',self.arm.note); self.assertFalse(self.arm.goal_pub.messages)

    def test_stop_is_independent_of_preview_and_feedback(self):
        self.node.on_target(self.arm, self.target()); self.node.update_preview(self.arm)
        self.arm.status.feedback_valid = False; self.node.on_action(NS(data='left/stop'))
        self.assertEqual(self.arm.control_pub.messages[-1].operation, ControlCommand.STOP)
        self.assertFalse(self.node.executor.jobs[-1].done())

    def test_preset_request_invalidates_old_plan_before_editor_ack(self):
        self.preview(); self.node.on_action(NS(data='left/home'))
        with self.assertRaises(ValueError): self.node.send(self.arm)
        self.assertFalse(self.arm.requested); self.assertIsNone(self.arm.preview)
        self.node.on_target(self.arm,self.target()); self.node.update_preview(self.arm)
        self.node.executor.jobs[-1].complete(); self.node.update_preview(self.arm)
        self.assertTrue(self.node.ready_to_send(self.arm))

    def test_preset_request_also_invalidates_inflight_old_plan(self):
        self.node.on_target(self.arm,self.target()); self.node.update_preview(self.arm)
        future=self.node.executor.jobs[-1]
        self.node.on_action(NS(data='left/reset')); future.complete(); self.node.update_preview(self.arm)
        self.assertIsNone(self.arm.preview); self.assertFalse(self.node.ready_to_send(self.arm))

    def test_tab_switch_waits_for_editor_ack_before_path_publication(self):
        self.preview(self.node.arms['right']); before = len(self.node.paths.messages)
        self.node.on_action(NS(data='right/select'))
        self.assertEqual(self.node.selected,'right'); self.assertIsNone(self.node.arms['right'].preview)
        self.assertEqual(self.node.paths.messages[-1].trajectory, [])
        self.assertEqual(len(self.node.paths.messages),before+1)

    def test_path_start_includes_both_arms_and_uses_real_names(self):
        self.preview(); path = self.node.paths.messages[-1]
        values = dict(zip(path.trajectory_start.joint_state.name,path.trajectory_start.joint_state.position))
        self.assertEqual([values[n] for n in self.arm.names], START)
        self.assertEqual([values[n] for n in self.node.arms['right'].names], START)
        self.assertEqual(path.model_id,'uni_dingo_dual_arm')
        self.assertFalse(any('preview_' in n for n in values))

    def test_panel_status_has_base_target_and_pending_disables_send(self):
        self.preview(); self.node.send(self.arm); self.node.tick(None)
        status = next(m for m in self.node.ui_status.messages if m.arm=='left')
        self.assertFalse(status.ready); self.assertEqual(status.target_pose,self.arm.target.pose)
        self.assertEqual(status.header.frame_id,self.arm.base)

    def test_stale_feedback_clears_reference_without_forwarding_old_joints(self):
        self.preview(); self.arm.status.feedback_valid = False
        count=len(self.node.joints.messages)
        self.node.on_joints(self.arm,JointState(Header(Time.now()),self.arm.names,list(START)))
        self.assertEqual(len(self.node.joints.messages),count)
        self.node.tick(None); self.assertIsNone(self.arm.preview)

    def test_missing_command_subscriber_does_not_publish(self):
        self.preview(); self.arm.goal_pub.connections=0
        with self.assertRaises(ValueError): self.node.send(self.arm)
        self.assertFalse(self.arm.goal_pub.messages)

    def test_holding_uses_reference_and_disabled_uses_measured_seed(self):
        self.arm.reference[0]=.2; self.preview()
        self.assertEqual(self.arm.preview_client.call_args.args[0].start_positions[0],.2)
        self.arm.status.mode=Status.DISABLED; self.preview()
        self.assertEqual(self.arm.preview_client.call_args.args[0].start_positions,START)


if __name__ == '__main__': unittest.main()
