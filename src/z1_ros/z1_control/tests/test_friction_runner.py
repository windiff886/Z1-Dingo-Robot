#!/usr/bin/env python3
"""Exercise the complete ROS runner with a deterministic in-process plant."""
import csv
import importlib.util
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import yaml
from test_friction_identification import ROOT, profile
from z1_control.friction import Point


class Message:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.points = []


def message_module():
    msgs = ModuleType('z1_msgs.msg')
    for name in ('ControlCommand', 'ControlSample', 'JointGoal', 'JointTrajectory',
                 'MotionResult', 'Status', 'TrajectoryPoint'):
        setattr(msgs, name, type(name, (Message,), {}))
    for i, name in enumerate(('ENABLE', 'DISABLE', 'STOP', 'RESUME', 'JOINT_STREAM',
                              'POSE_STREAM', 'IMPEDANCE', 'HOLD')):
        setattr(msgs.ControlCommand, name, i)
    for i, name in enumerate(('ACCEPTED', 'SUCCEEDED', 'REJECTED', 'CANCELED', 'STOPPED', 'FAILED')):
        setattr(msgs.MotionResult, name, i)
    for i, name in enumerate(('DISABLED', 'HOLDING', 'JOINT_GOAL', 'POSE_GOAL', 'JOINT_STREAM',
                              'POSE_STREAM', 'IMPEDANCE', 'STOPPING', 'AWAIT_RESUME', 'FAULT',
                              'JOINT_TRAJECTORY', 'CARTESIAN_PATH')):
        setattr(msgs.Status, name, i)
    return msgs


class Plant:
    def __init__(self, msgs):
        self.msgs = msgs
        self.time = 1000.
        self.sequence = 0
        self.callbacks = {}
        self.q = np.zeros(6)
        self.v = np.zeros(6)
        self.mode = msgs.Status.DISABLED
        self.active = None
        self.points = None
        self.control = yaml.safe_load((ROOT.parent / 'z1_bringup/config/control.yaml').read_text())
        self.drop_next = False
        self.commands = []
        self.friction_enabled = False

    def ros(self):
        ros = ModuleType('rospy')
        plant = self
        class Publisher:
            def __init__(self, topic, *args, **kwargs): self.topic = topic
            def get_num_connections(self): return 1
            def publish(self, message): plant.publish(self.topic, message)
        ros.Publisher = Publisher
        ros.Subscriber = lambda topic, cls, callback, **kwargs: self.callbacks.setdefault(topic, callback)
        ros.Time = SimpleNamespace(now=lambda: self.time)
        ros.Duration = SimpleNamespace(from_sec=float)
        ros.is_shutdown = lambda: False
        ros.on_shutdown = lambda callback: None
        ros.loginfo = ros.logerr = lambda *args: None
        def parameter(name):
            key = name.rsplit('/', 1)[-1]
            if key == 'device_has_gripper': return True
            if key == 'effective_model_xml': return (ROOT.parent / 'z1_description/urdf/z1_control.urdf').read_text()
            if key == 'model_file': return str(ROOT.parent / 'z1_description/urdf/z1_control.urdf')
            if key == 'friction': return dict(enabled=self.friction_enabled,
                coulomb_nm=[.25]*6, viscous_nm_s_per_rad=[.8]*6,
                velocity_scale_rad_s=.01, impedance_velocity_filter_s=.02,
                model_revision='simulated-gripper-model')
            return self.control[key]
        ros.get_param = parameter
        return ros

    def result(self, identifier, outcome):
        result = self.msgs.MotionResult()
        result.command_id, result.outcome, result.reason = identifier, outcome, 'simulated'
        self.callbacks['/left_arm/result'](result)

    def publish(self, topic, message):
        self.commands.append((topic, message, self.q.copy()))
        if topic.endswith('/control'):
            if message.operation == self.msgs.ControlCommand.STOP:
                if self.active:
                    self.result(self.active, self.msgs.MotionResult.CANCELED)
                self.active, self.points = None, None
                self.mode = self.msgs.Status.AWAIT_RESUME
            elif message.operation == self.msgs.ControlCommand.HOLD:
                self.mode = self.msgs.Status.HOLDING
            return
        self.active, self.began = message.command_id, self.time
        if topic.endswith('/joint_goal'):
            self.points = [Point(0., self.q.copy(), np.zeros(6), np.zeros(6)),
                           Point(.08, np.array(message.positions), np.zeros(6), np.zeros(6))]
            self.mode = self.msgs.Status.JOINT_GOAL
        else:
            self.points = [Point(p.time_from_start, np.array(p.positions),
                                 np.array(p.velocities), np.array(p.accelerations)) for p in message.points]
            self.mode = self.msgs.Status.JOINT_TRAJECTORY
        self.result(message.command_id, self.msgs.MotionResult.ACCEPTED)

    def advance(self, unused=.005):
        self.time += .004
        if self.points:
            elapsed = self.time - self.began
            self.q, self.v, _ = profile(self.points, elapsed)
            if elapsed >= self.points[-1].time:
                self.q, self.v = self.points[-1].position.copy(), np.zeros(6)
                self.result(self.active, self.msgs.MotionResult.SUCCEEDED)
                self.active, self.points = None, None
                self.mode = self.msgs.Status.HOLDING
        else:
            self.v = np.zeros(6)
        status = self.msgs.Status()
        status.mode, status.model_revision, status.reason = self.mode, 'simulated-gripper-model', ''
        status.friction_compensation_enabled = self.friction_enabled
        self.callbacks['/left_arm/status'](status)
        self.sequence += 1
        message = self.msgs.ControlSample()
        message.feedback_sequence = self.sequence
        message.feedback_valid = True
        message.dropped_samples = 0
        message.received_at, message.sent_at = self.time, self.time - .002
        message.has_gripper, message.gripper_position = True, 0.
        message.has_temperature, message.temperature = True, [30] * 8
        message.mode, message.active_command_id = self.mode, self.active or 0
        message.command_enabled = self.mode != self.msgs.Status.DISABLED
        message.friction_compensation_enabled = self.friction_enabled
        message.friction_feedforward = np.zeros(6)
        message.position = self.q.copy()
        message.velocity = self.v.copy()
        message.gravity = 6.3 + 3.5 * np.sin(self.q)
        message.effort = 6. + 4. * np.sin(self.q) + .25 * np.sign(self.v) + .8 * self.v
        message.command_position, message.command_velocity = self.q.copy(), self.v.copy()
        message.command_effort = message.gravity.copy()
        message.command_kp, message.command_kd = self.control['mcu_kp'], self.control['mcu_kd']
        if not self.drop_next:
            self.callbacks['/left_arm/control_samples'](message)
        self.drop_next = False


class Runner(unittest.TestCase):
    def run_experiment(self, directory, missing=False, joints=(6,), initial_position=None):
        msgs = message_module()
        plant = Plant(msgs)
        if initial_position is not None:
            plant.q = np.array(initial_position, dtype=float)
        modules = {'rospy': plant.ros(), 'z1_msgs': ModuleType('z1_msgs'), 'z1_msgs.msg': msgs}
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('friction_test_runner', ROOT / 'python/z1_control/friction_experiment.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            config = yaml.safe_load((ROOT / 'config/friction_identification.yaml').read_text())
            config.update(arms=['left'], joints=list(joints), speeds=[.04, .08, .12], amplitude=.08,
                          settle_seconds=.02, startup_delay=0., static_repeats=1,
                          static_dwell=.08, static_speed=.01, static_offset=.005,
                          min_steady_duration=.2, output_directory=directory)
            experiment = module.Experiment(config)
            plant.advance()
            if missing:
                original_consume = experiment.consume
                def consume(arm, sample):
                    if plant.sequence > 50: plant.drop_next = True
                    original_consume(arm, sample)
                experiment.consume = consume
            with patch.object(module.time, 'monotonic', side_effect=lambda: plant.time), \
                    patch.object(module.time, 'sleep', side_effect=plant.advance):
                experiment.run()
            return experiment, plant

    def test_full_acquisition_fit_breakaway_and_offline_refit(self):
        with tempfile.TemporaryDirectory() as directory:
            experiment, plant = self.run_experiment(directory)
            self.assertEqual(experiment.report['status'], 'complete', experiment.report)
            result = experiment.report['results']['left']['joint6']
            self.assertTrue(result['usable'])
            self.assertAlmostEqual(result['coulomb_nm'], .25, places=3)
            self.assertAlmostEqual(result['viscous_nm_s_per_rad'], .8, places=3)
            self.assertTrue(result['static']['usable'])
            self.assertEqual(plant.mode, plant.msgs.Status.HOLDING)
            self.assertAlmostEqual(plant.q[5], 0., places=8)
            spec = importlib.util.spec_from_file_location('friction_refit', ROOT / 'scripts/friction_fit.py')
            fit = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(fit)
            output = fit.refit(experiment.directory)
            refitted = yaml.safe_load(output.read_text())
            self.assertAlmostEqual(refitted['results']['left']['joint6']['coulomb_nm'], .25, places=3)
            self.assertTrue((experiment.directory / 'left_model.urdf').is_file())

    def test_missing_feedback_requests_stop_and_preserves_partial_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            experiment, plant = self.run_experiment(directory, missing=True)
            self.assertEqual(experiment.report['status'], 'failed')
            self.assertIn('sequence gap', experiment.report['error'])
            self.assertEqual(plant.mode, plant.msgs.Status.AWAIT_RESUME)
            self.assertTrue((experiment.directory / 'samples.csv').stat().st_size > 100)

    def test_all_six_joints_return_to_fixed_working_pose_between_tests(self):
        # Deliberately not Home: all selected joints need bidirectional travel.
        center = np.array([.15, .6, -.8, .1, .1, .1])
        with tempfile.TemporaryDirectory() as directory:
            experiment, plant = self.run_experiment(directory, joints=range(1, 7),
                                                     initial_position=center)
            self.assertEqual(experiment.report['status'], 'complete', experiment.report)
            self.assertEqual(list(experiment.report['results']['left']),
                             ['joint%d' % j for j in range(1, 7)])
            previous_goal, seen = None, []
            for topic, message, before in plant.commands:
                if topic.endswith('/joint_goal'):
                    previous_goal = (message, before)
                elif topic.endswith('/joint_trajectory'):
                    trial = experiment.trials[message.command_id]
                    if trial['kind'] == 'sweep' and trial['joint'] not in seen:
                        seen.append(trial['joint'])
                        goal, preparation_start = previous_goal
                        # Before preparing the next axis, the prior axis has
                        # already returned to the same fixed six-joint center.
                        np.testing.assert_allclose(preparation_start, center, atol=1e-8)
                        start = center.copy()
                        start[trial['joint']-1] -= experiment.config['amplitude']
                        np.testing.assert_allclose(goal.positions, start, atol=1e-8)
            self.assertEqual(seen, [1, 2, 3, 4, 5, 6])
            np.testing.assert_allclose(plant.q, center, atol=1e-8)
            self.assertEqual(plant.mode, plant.msgs.Status.HOLDING)

    def test_six_axis_home_is_rejected_before_any_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            experiment, plant = self.run_experiment(directory, joints=range(1, 7),
                initial_position=[0., 0., 0., -.07435, 0., 0.])
            self.assertEqual(experiment.report['status'], 'failed')
            self.assertIn('joint 2 has insufficient travel', experiment.report['error'])
            self.assertEqual(plant.commands, [])


if __name__ == '__main__':
    unittest.main()
