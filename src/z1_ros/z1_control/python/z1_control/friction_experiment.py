"""Shared ROS experiment acquisition, guards, trajectories and identification runner."""
import csv
from collections import deque
from datetime import datetime
import math
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rospy
import yaml
from z1_msgs.msg import (ControlCommand, ControlSample, JointGoal, JointTrajectory,
                         MotionResult, Status, TrajectoryPoint)
from z1_control.friction import (BreakawayDetector, identify_joint, static_ramp,
                                 sweep)


VECTORS = ('position', 'velocity', 'effort', 'gravity', 'command_position',
           'command_velocity', 'command_effort', 'command_kp', 'command_kd', 'friction_feedforward')
SCALARS = ('arm', 'received_at', 'sent_at', 'sequence', 'has_gripper', 'mode',
           'active_command_id', 'kind', 'joint', 'repeat', 'direction', 'speed',
           'gripper_position', 'has_temperature', 'friction_compensation_enabled')


class Arm:
    def __init__(self, side):
        self.side, self.ns = side, '/' + side + '_arm'
        self.lock = threading.Lock()
        self.status = self.latest = None
        self.arrived = 0.
        self.pending = deque()
        self.collect = False
        self.error = None
        self.sequence = 0
        self.results = {}
        self.goals = rospy.Publisher(self.ns + '/joint_goal', JointGoal, queue_size=1)
        self.trajectories = rospy.Publisher(self.ns + '/joint_trajectory', JointTrajectory, queue_size=1)
        self.controls = rospy.Publisher(self.ns + '/control', ControlCommand, queue_size=4)
        self.subscribers = [
            rospy.Subscriber(self.ns + '/control_samples', ControlSample, self.sample,
                             queue_size=4096, tcp_nodelay=True),
            rospy.Subscriber(self.ns + '/status', Status, self.on_status, queue_size=1),
            rospy.Subscriber(self.ns + '/result', MotionResult, self.result, queue_size=128)]

    def sample(self, message):
        with self.lock:
            if self.collect:
                if self.sequence and message.feedback_sequence != self.sequence + 1:
                    self.error = 'control_samples sequence gap'
                if len(self.pending) >= 8192:
                    self.error = 'experiment sample queue overflow'
                else:
                    self.pending.append(message)
            self.sequence = message.feedback_sequence
            self.latest, self.arrived = message, time.monotonic()

    def on_status(self, message):
        with self.lock:
            self.status = message

    def result(self, message):
        with self.lock:
            self.results[message.command_id] = message

    def view(self):
        with self.lock:
            return self.latest, self.status, self.arrived, self.error

    def drain(self):
        with self.lock:
            samples = list(self.pending)
            self.pending.clear()
            return samples


class Experiment:
    require_uncompensated = True
    minimum_repeats = 3
    minimum_speeds = 3
    def __init__(self, config):
        self.config = config
        self.validate_config()
        self.arms = [Arm(side) for side in config['arms']]
        self.identifier = time.time_ns()
        self.trials, self.models, self.centers = {}, {}, {}
        self.owned = set()
        self.issued_ids = set()
        self.current = None
        self.axis = None
        self.detector = None
        self.static_id = None
        self.rows = []
        self.file = None
        directory = Path(config['output_directory']).expanduser()
        self.directory = directory / (datetime.now().strftime('%Y%m%d-%H%M%S-') + str(time.time_ns() % 1000000000))
        self.directory.mkdir(parents=True, exist_ok=False)
        self.report = {'schema_version': 1, 'status': 'preflight', 'config': config,
                       'models': {}, 'results': {}, 'torque_source': 'MCU feedback effort; joint-side Nm',
                       'compensation_applied': False}
        self.save()
        rospy.on_shutdown(self.shutdown)

    def validate_config(self):
        c = self.config
        if not c['arms'] or len(set(c['arms'])) != len(c['arms']) or any(a not in ('left', 'right') for a in c['arms']):
            raise ValueError('arms must select left and/or right once')
        if not c['joints'] or len(set(c['joints'])) != len(c['joints']) or any(type(j) is not int or j not in range(1, 7) for j in c['joints']):
            raise ValueError('joints must contain unique indices 1..6')
        if type(c['repeats']) is not int or c['repeats'] < self.minimum_repeats:
            raise ValueError('at least %d repeats are needed'%self.minimum_repeats)
        if len(set(c['speeds'])) < self.minimum_speeds:
            raise ValueError('at least %d different speeds are needed'%self.minimum_speeds)
        for key in ('amplitude', 'acceleration', 'jerk', 'sample_timeout', 'startup_timeout',
                    'max_tracking_error', 'other_joint_tolerance', 'static_offset', 'static_speed',
                    'breakaway_displacement', 'breakaway_persistence', 'goal_speed_scale',
                    'static_dwell', 'max_sample_gap', 'velocity_window', 'steady_acceleration',
                    'max_steady_gap', 'min_steady_duration', 'min_position_overlap'):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(key + ' must be finite and positive')
        if not 0 < c['goal_speed_scale'] <= 1 or c['settle_seconds'] < 0 or c['startup_delay'] < 0:
            raise ValueError('invalid goal speed or waiting time')
        if len(c['effort_limit']) != 6 or any(not math.isfinite(x) or x <= 0 for x in c['effort_limit']):
            raise ValueError('effort_limit must contain six positive joint-side torque thresholds')
        if type(c['static_enabled']) is not bool or type(c['static_repeats']) is not int or c['static_repeats'] < 1:
            raise ValueError('invalid static experiment configuration')
        if not 0 < c['speed_tolerance'] < 1 or not 0 < c['min_steady_fraction'] <= 1:
            raise ValueError('invalid steady-data acceptance fractions')
        if c['static_offset'] > c['amplitude']:
            raise ValueError('static_offset must not exceed the experiment amplitude')
        for speed in c['speeds']:
            sweep(np.zeros(6), 0, 2 * c['amplitude'], speed, c['acceleration'], c['jerk'])

    def save(self):
        temporary = self.directory / 'result.yaml.tmp'
        temporary.write_text(yaml.safe_dump(self.report, allow_unicode=True, sort_keys=False))
        temporary.replace(self.directory / 'result.yaml')

    def next_id(self):
        self.identifier = max(self.identifier + 1, time.time_ns())
        return self.identifier

    def shutdown(self):
        for arm in self.arms:
            if arm.side in self.owned:
                message = ControlCommand()
                message.header.stamp = rospy.Time.now()
                message.command_id, message.operation = self.next_id(), ControlCommand.STOP
                arm.controls.publish(message)

    def pump(self):
        if rospy.is_shutdown():
            raise RuntimeError('ROS shutdown interrupted experiment')
        for arm in self.arms:
            for message in arm.drain():
                self.consume(arm, message)
            latest, status, arrived, error = arm.view()
            if error:
                raise RuntimeError(arm.side + ': ' + error)
            if arm.side in self.owned:
                if latest is None or time.monotonic() - arrived > self.config['sample_timeout']:
                    raise RuntimeError(arm.side + ': feedback publisher became stale')
                if status is not None and status.mode == Status.FAULT:
                    raise RuntimeError(arm.side + ': ' + status.reason)
        time.sleep(.005)

    def consume(self, arm, m):
        if not m.feedback_valid or m.dropped_samples:
            raise RuntimeError(arm.side + ': invalid feedback or native recorder overflow')
        values = np.array([getattr(m, name) for name in VECTORS], dtype=float)
        if not np.isfinite(values).all():
            raise RuntimeError(arm.side + ': nonfinite experiment sample')
        if bool(m.has_gripper) != self.models[arm.side]['has_gripper']:
            raise RuntimeError(arm.side + ': MCU attachment changed during experiment')
        if m.active_command_id and m.active_command_id not in self.issued_ids:
            raise RuntimeError(arm.side + ': another client submitted a motion command')
        if (np.abs(m.effort) > self.config['effort_limit']).any():
            raise RuntimeError(arm.side + ': measured torque exceeded experiment threshold')
        if m.command_enabled and max(abs(a - b) for a, b in zip(m.position, m.command_position)) > self.config['max_tracking_error']:
            raise RuntimeError(arm.side + ': tracking error exceeded experiment threshold')
        if self.require_uncompensated and m.command_enabled and m.friction_compensation_enabled:
            raise RuntimeError(arm.side + ': disable friction compensation before identification')
        if self.current is arm and self.axis is not None:
            delta = np.abs(np.array(m.position) - self.centers[arm.side])
            if delta[self.axis] > self.config['amplitude'] + self.config['max_tracking_error']:
                raise RuntimeError(arm.side + ': experiment travel boundary exceeded')
            delta[self.axis] = 0
            if delta.max() > self.config['other_joint_tolerance']:
                raise RuntimeError(arm.side + ': a non-tested joint moved')
        trial = self.trials.get(m.active_command_id, {})
        row = {'arm': arm.side, 'received_at': m.received_at, 'sent_at': m.sent_at,
               'sequence': m.feedback_sequence, 'has_gripper': m.has_gripper, 'mode': m.mode,
               'active_command_id': m.active_command_id, 'kind': trial.get('kind', 'hold'),
               'joint': trial.get('joint', 0), 'repeat': trial.get('repeat', -1),
               'direction': trial.get('direction', 0), 'speed': trial.get('speed', 0.)}
        row['gripper_position'], row['has_temperature'] = m.gripper_position, m.has_temperature
        row['friction_compensation_enabled'] = m.friction_compensation_enabled
        row.update({name: getattr(m, name) for name in VECTORS})
        self.writer.writerow([row[key] for key in SCALARS] +
                             [value for name in VECTORS for value in row[name]] + list(m.temperature))
        if row['kind'] == 'sweep':
            self.rows.append(row)
        if self.detector is not None and m.active_command_id == self.static_id:
            j = self.axis
            self.detector.update(m.received_at, m.position[j], m.effort[j] - m.gravity[j])
        return row

    def wait(self, predicate, timeout, reason):
        end = time.monotonic() + timeout
        while not predicate():
            self.pump()
            if time.monotonic() >= end:
                raise RuntimeError(reason)

    def pause(self, duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            self.pump()

    def preflight(self):
        for arm in self.arms:
            self.wait(lambda: arm.view()[0] is not None and arm.view()[1] is not None and
                      arm.goals.get_num_connections() and arm.trajectories.get_num_connections() and
                      arm.controls.get_num_connections(), self.config['startup_timeout'],
                      arm.side + ': controller/raw telemetry not ready')
            sample, status, arrived, _ = arm.view()
            if not sample.feedback_valid or time.monotonic() - arrived > self.config['sample_timeout']:
                raise RuntimeError(arm.side + ': fresh valid feedback required')
            if status.mode not in (Status.DISABLED, Status.HOLDING):
                raise RuntimeError(arm.side + ': controller must be Disabled or Holding')
            if self.require_uncompensated and status.friction_compensation_enabled:
                raise RuntimeError(arm.side + ': identification requires friction compensation disabled')
            ns = arm.ns + '/controller/'
            installed = rospy.get_param(ns + 'device_has_gripper')
            if bool(installed) != bool(sample.has_gripper):
                raise RuntimeError(arm.side + ': model selection and MCU feedback disagree')
            xml = rospy.get_param(ns + 'effective_model_xml')
            root = ET.fromstring(xml)
            limits = [root.find("joint[@name='joint%d']/limit" % j) for j in range(1, 7)]
            lower = np.array([float(limit.get('lower')) for limit in limits])
            upper = np.array([float(limit.get('upper')) for limit in limits])
            center = self.config['centers'].get(arm.side)
            center = np.clip(np.array(sample.position), lower, upper) if not center else np.array(center, dtype=float)
            if center.shape != (6,) or not np.isfinite(center).all():
                raise ValueError(arm.side + ': center must contain six finite joint positions')
            if (center < lower).any() or (center > upper).any():
                raise ValueError(arm.side + ': center outside model limits')
            for joint in self.config['joints']:
                j = joint - 1
                margin = self.config['amplitude'] + self.config['max_tracking_error']
                if center[j] - margin <= lower[j] or center[j] + margin >= upper[j]:
                    raise ValueError(arm.side + ': joint %d has insufficient travel around center; choose a working pose' % joint)
            host_velocity = rospy.get_param(ns + 'velocity')
            if max(self.config['speeds']) > min(host_velocity[j - 1] for j in self.config['joints']):
                raise ValueError('experiment speed exceeds controller limits')
            self.centers[arm.side] = center
            self.models[arm.side] = {'has_gripper': bool(installed), 'revision': status.model_revision,
                                     'center': center.tolist(),
                                     'model_file': rospy.get_param(ns + 'model_file'),
                                     'initial_gripper_position': float(sample.gripper_position),
                                     'control_parameters': {key: rospy.get_param(ns + key) for key in
                                         ('period', 'feedback_timeout', 'velocity', 'acceleration',
                                          'jerk', 'torque', 'torque_rate', 'kp', 'kd', 'mcu_kp', 'mcu_kd')}}
            self.report['models'][arm.side] = self.models[arm.side]
            self.models[arm.side]['friction'] = rospy.get_param(ns + 'friction')
            (self.directory / (arm.side + '_model.urdf')).write_text(xml)
        self.file = (self.directory / 'samples.csv').open('w', newline='', buffering=1024 * 1024)
        self.writer = csv.writer(self.file)
        self.writer.writerow(list(SCALARS) + [name + str(j) for name in VECTORS for j in range(1, 7)] +
                             ['temperature' + str(j) for j in range(1, 9)])
        self.report['status'] = 'running'
        self.save()
        rospy.loginfo('Friction experiment: %s; starts in %.1f s; left/right attachments come from MCU',
                      self.directory, self.config['startup_delay'])
        self.pause(self.config['startup_delay'])

    def goal(self, arm, position):
        message = JointGoal()
        message.header.stamp = rospy.Time.now()
        message.command_id = self.next_id()
        message.positions = position.tolist()
        message.speed_scale = self.config['goal_speed_scale']
        self.owned.add(arm.side)
        self.issued_ids.add(message.command_id)
        arm.goals.publish(message)
        self.finish(arm, message.command_id, 120.)

    def trajectory(self, arm, points, trial):
        message = JointTrajectory()
        message.header.stamp = rospy.Time.now()
        message.command_id = self.next_id()
        message.retime = False
        for point in points:
            item = TrajectoryPoint()
            item.time_from_start = rospy.Duration.from_sec(point.time)
            item.positions, item.velocities = point.position.tolist(), point.velocity.tolist()
            item.accelerations = point.acceleration.tolist()
            item.has_velocity = item.has_acceleration = True
            message.points.append(item)
        self.trials[message.command_id] = trial
        self.owned.add(arm.side)
        self.issued_ids.add(message.command_id)
        arm.trajectories.publish(message)
        return message.command_id

    def finish(self, arm, identifier, timeout):
        def finished():
            with arm.lock:
                result = arm.results.get(identifier)
            if result is None or result.outcome == MotionResult.ACCEPTED:
                return False
            if result.outcome != MotionResult.SUCCEEDED:
                raise RuntimeError(arm.side + ': motion ended: ' + result.reason)
            return True
        self.wait(finished, timeout, arm.side + ': finite motion completion timeout')
        self.pause(self.config['settle_seconds'])

    def operation(self, arm, operation):
        message = ControlCommand()
        message.header.stamp = rospy.Time.now()
        message.command_id, message.operation = self.next_id(), operation
        arm.controls.publish(message)

    def stop_and_hold(self, arm):
        self.operation(arm, ControlCommand.STOP)
        self.wait(lambda: arm.view()[1].mode == Status.AWAIT_RESUME, 8., 'STOP did not settle')
        self.operation(arm, ControlCommand.HOLD)
        self.wait(lambda: arm.view()[1].mode == Status.HOLDING, 3., 'HOLD was not applied')

    def breakaway(self, arm, joint, center):
        results = {1: [], -1: []}
        for repeat in range(self.config['static_repeats']):
            for direction in (1, -1):
                self.goal(arm, center)
                stationary = []
                until = time.monotonic() + self.config['static_dwell']
                while time.monotonic() < until:
                    self.pump()
                    stationary.append(arm.view()[0].position[joint])
                base = float(np.median(stationary))
                noise = float(1.4826 * np.median(np.abs(np.array(stationary) - base)))
                threshold = max(self.config['breakaway_displacement'], 5 * noise)
                self.detector = BreakawayDetector(base, direction, threshold,
                                                  self.config['breakaway_persistence'])
                points = static_ramp(center, joint, direction * self.config['static_offset'],
                                     self.config['static_speed'])
                identifier = self.trajectory(arm, points, {'kind': 'static', 'joint': joint + 1,
                                             'repeat': repeat, 'direction': direction,
                                             'speed': self.config['static_speed']})
                self.static_id = identifier
                end = time.monotonic() + points[-1].time + 1.
                while self.detector.result is None and time.monotonic() < end:
                    self.pump()
                    with arm.lock:
                        result = arm.results.get(identifier)
                    if result and result.outcome not in (MotionResult.ACCEPTED, MotionResult.SUCCEEDED):
                        raise RuntimeError('static ramp rejected/failed: ' + result.reason)
                detected = self.detector.result
                self.detector = None
                self.static_id = None
                self.stop_and_hold(arm)
                results[direction].append(detected)
                self.report['static_progress'] = {'arm': arm.side, 'joint': joint + 1,
                                                  'repeat': repeat, 'direction': direction,
                                                  'detected': detected}
                self.save()
        positive = [r['residual_torque_nm'] for r in results[1] if r is not None]
        negative = [r['residual_torque_nm'] for r in results[-1] if r is not None]
        complete = len(positive) == len(negative) == self.config['static_repeats']
        force = float(.5 * (np.median(positive) - np.median(negative))) if complete else None
        return {'usable': bool(complete and force > 0),
                'breakaway_nm': force,
                'positive_trials': results[1], 'negative_trials': results[-1],
                'displacement_floor_rad': self.config['breakaway_displacement'],
                'dwell_seconds': self.config['static_dwell'],
                'reason': 'measured threshold' if complete else 'one or more ramps did not break away within reference offset cap'}

    def run(self):
        try:
            self.preflight()
            for arm in self.arms:
                self.current = arm
                self.axis = None
                with arm.lock:
                    arm.collect = True
                self.goal(arm, self.centers[arm.side])
                self.report['results'][arm.side] = {}
                for joint in self.config['joints']:
                    self.axis, self.rows = joint - 1, []
                    center = self.centers[arm.side]
                    rospy.loginfo('%s joint%d: paired velocity sweeps', arm.side, joint)
                    for repeat in range(self.config['repeats']):
                        # Alternating order limits speed/temperature correlation.
                        speeds = self.config['speeds'] if repeat % 2 == 0 else list(reversed(self.config['speeds']))
                        for speed in speeds:
                            start = center.copy()
                            start[self.axis] -= self.config['amplitude']
                            self.goal(arm, start)
                            for direction in (1, -1):
                                end = center[self.axis] + direction * self.config['amplitude']
                                points = sweep(start, self.axis, end, speed,
                                               self.config['acceleration'], self.config['jerk'])
                                identifier = self.trajectory(arm, points, {
                                    'kind': 'sweep', 'joint': joint, 'repeat': repeat,
                                    'direction': direction, 'speed': speed})
                                self.finish(arm, identifier, points[-1].time + 10.)
                                start = points[-1].position
                            self.file.flush()
                    self.goal(arm, center)
                    for message in arm.drain():
                        self.consume(arm, message)
                    try:
                        result = identify_joint(self.rows, self.axis, self.config)
                    except ValueError as error:
                        result = {'usable': False, 'reason': str(error)}
                    if self.config['static_enabled']:
                        result['static'] = self.breakaway(arm, self.axis, center)
                        if (result['static']['usable'] and result.get('coulomb_nm') is not None and
                                result['static']['breakaway_nm'] < .8 * result['coulomb_nm']):
                            result['static'].update(usable=False,
                                reason='breakaway below moving Coulomb estimate; check onset threshold and torque timing')
                    self.report['results'][arm.side]['joint' + str(joint)] = result
                    self.goal(arm, center)
                    self.file.flush()
                    self.save()
                    rospy.loginfo('%s joint%d fit usable=%s: %s', arm.side, joint,
                                  result['usable'], result.get('reason'))
                self.axis = None
                with arm.lock:
                    arm.collect = False
                for message in arm.drain():
                    self.consume(arm, message)
            usable = all(result['usable'] and
                         (not self.config['static_enabled'] or result['static']['usable'])
                         for joints in self.report['results'].values() for result in joints.values())
            self.report['status'] = 'complete' if usable else 'complete_with_rejections'
            self.save()
            rospy.loginfo('Friction identification complete: %s/result.yaml; controllers stay in Holding', self.directory)
        except Exception as error:
            # Stop physical motion before attempting more file IO (including
            # reporting a disk-full or recording failure).
            self.shutdown()
            self.report.update(status='failed', error=str(error))
            try:
                self.save()
            except OSError as recording_error:
                rospy.logerr('Cannot persist failure report: %s', recording_error)
            rospy.logerr('Friction experiment stopped: %s; partial data: %s', error, self.directory)
        finally:
            for arm in self.arms:
                with arm.lock:
                    arm.collect = False
            if self.file:
                self.file.close()


