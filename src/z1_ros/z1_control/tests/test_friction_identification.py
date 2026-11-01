#!/usr/bin/env python3
"""Synthetic physical data and experiment wiring; no MCU or ROS is needed."""
import copy
import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
from z1_control.friction import (BreakawayDetector, fit_coulomb_viscous,
                                identify_joint, paired_measurement, sweep)


def profile(points, time):
    i = min(np.searchsorted([p.time for p in points], time, side='right') - 1, len(points) - 2)
    p, n = points[max(0, i)], points[max(0, i) + 1]
    dt = n.time - p.time
    s = min(1., max(0., (time - p.time) / dt))
    c0, c1, c2 = p.position, p.velocity * dt, .5 * p.acceleration * dt * dt
    dp, dv, da = n.position - c0 - c1 - c2, n.velocity * dt - c1 - 2 * c2, n.acceleration * dt * dt - 2 * c2
    c3, c4, c5 = 10 * dp - 4 * dv + .5 * da, -15 * dp + 7 * dv - da, 6 * dp - 3 * dv + .5 * da
    q = c0 + c1 * s + c2 * s**2 + c3 * s**3 + c4 * s**4 + c5 * s**5
    v = (c1 + 2 * c2 * s + 3 * c3 * s**2 + 4 * c4 * s**3 + 5 * c5 * s**4) / dt
    a = (2 * c2 + 6 * c3 * s + 12 * c4 * s**2 + 20 * c5 * s**3) / dt**2
    return q, v, a


def samples(speed, direction, repeat=0, wrong_validation=False):
    start = np.zeros(6)
    start[5] = -.15 * direction
    points = sweep(start, 5, .15 * direction, speed, .3, 2.)
    result = []
    for t in np.arange(0., points[-1].time, .004):
        q, v, a = profile(points, t)
        # Actual lag differs by direction, so matching trajectory time is wrong.
        actual = q.copy()
        actual[5] -= direction * .002
        g_actual = 6. + 4. * np.sin(actual)
        g_model = 6.3 + 3.5 * np.sin(actual)  # deliberate gravity/bias error
        friction = .25 * direction + .8 * v
        if wrong_validation:
            friction += .2 * direction
        tau = g_actual + friction
        result.append({'received_at': 1000. + t, 'position': actual,
                       'velocity': np.ones(6) * .09, # noisy raw velocity is not the regressor
                       'effort': tau, 'gravity': g_model,
                       'command_velocity': v, 'command_effort': g_model,
                       'kind': 'sweep', 'repeat': repeat, 'direction': direction,
                       'speed': speed})
    return result


class Friction(unittest.TestCase):
    def test_sweep_is_true_cruise_and_respects_derivative_limits(self):
        for direction in (1, -1):
            start = np.zeros(6); start[5] = -.15 * direction
            points = sweep(start, 5, .15 * direction, .12, .3, 2.)
            for t in np.linspace(0, points[-1].time, 1001):
                q, v, a = profile(points, t)
                self.assertLessEqual(abs(q[5]), .150000001)
                self.assertLessEqual(abs(v[5]), .120000001)
                self.assertLessEqual(abs(a[5]), .300000001)
            middle = .5 * (points[1].time + points[2].time)
            _, v, a = profile(points, middle)
            self.assertAlmostEqual(v[5], direction * .12)
            self.assertAlmostEqual(a[5], 0.)
            with self.assertRaisesRegex(ValueError, 'too short'):
                sweep(np.zeros(6), 5, .001, .12, .3, 2.)

    def test_position_pairing_cancels_gravity_error_and_actual_lag(self):
        point = paired_measurement(samples(.04, 1), samples(.04, -1), 5, .04)
        self.assertAlmostEqual(point['friction'], .25 + .8 * .04, places=5)
        self.assertAlmostEqual(point['speed'], .04, places=5)

    def test_training_and_independent_repeat_validation(self):
        rows = []
        for repeat in range(3):
            for speed in (.02, .04, .08):
                for direction in (1, -1):
                    rows.extend(samples(speed, direction, repeat))
        result = identify_joint(rows, 5, {'repeats': 3})
        self.assertTrue(result['usable'], result)
        self.assertAlmostEqual(result['coulomb_nm'], .25, places=4)
        self.assertAlmostEqual(result['viscous_nm_s_per_rad'], .8, places=4)
        for row in rows:
            if row['repeat'] == 2:
                row['effort'] = row['effort'] + .2 * row['direction']
        rejected = identify_joint(rows, 5, {'repeats': 3})
        self.assertFalse(rejected['usable'])

    def test_stall_and_missing_samples_are_not_fitted(self):
        positive = samples(.04, 1)
        for row in positive:
            row['position'] = np.zeros(6)
        with self.assertRaisesRegex(ValueError, 'stick-slip'):
            paired_measurement(positive, samples(.04, -1), 5, .04)
        positive = samples(.04, 1)
        del positive[100:120]
        with self.assertRaisesRegex(ValueError, 'missing feedback'):
            paired_measurement(positive, samples(.04, -1), 5, .04)

    def test_bad_torque_sign_is_not_a_usable_positive_friction(self):
        with self.assertRaisesRegex(ValueError, 'sign'):
            fit_coulomb_viscous([{'speed': v, 'friction': -.2} for v in (.01, .04, .1)])

    def test_breakaway_uses_persistent_position_and_pre_onset_torque(self):
        detector = BreakawayDetector(0., 1, .00005, .05)
        for i in range(50):
            self.assertIsNone(detector.update(i * .004, (-1)**i * .000003, .1 + .001*i))
        self.assertIsNone(detector.update(.2, .0001, .8))
        self.assertIsNone(detector.update(.204, 0., .9))  # isolated excursion resets
        for i in range(20):
            result = detector.update(.3 + i*.004, .0001, 1.)
        self.assertIsNotNone(result)
        self.assertLess(result['residual_torque_nm'], 1.)


class Wiring(unittest.TestCase):
    def test_default_identifies_both_arms_all_six_joints(self):
        config = yaml.safe_load((ROOT / 'config/friction_identification.yaml').read_text())
        self.assertEqual(config['arms'], ['left', 'right'])
        self.assertEqual(config['joints'], [1, 2, 3, 4, 5, 6])

    def test_one_launch_owns_controllers_and_enables_native_recorder(self):
        launch = ET.parse(ROOT.parent / 'z1_bringup/launch/friction_identification.launch').getroot()
        self.assertEqual(launch.find('include').get('file'), '$(find z1_bringup)/launch/controllers.launch')
        self.assertEqual({p.get('name'): p.get('value') for p in launch.findall('param')},
                         {side + '_arm/controller/record_control_samples': 'true' for side in ('left', 'right')})
        self.assertEqual(launch.find('include/arg').attrib, {'name':'disable_friction','value':'true'})
        controllers = ET.parse(ROOT.parent / 'z1_bringup/launch/controllers.launch').getroot()
        self.assertEqual(controllers.find("arg[@name='disable_friction']").get('default'), 'false')
        for side in ('left','right'):
            node = controllers.find("group[@ns='%s_arm']/node"%side)
            self.assertEqual(node.find("param[@name='friction_force_disabled']").get('value'), '$(arg disable_friction)')
        node = launch.find('node')
        self.assertEqual(node.get('type'), 'friction_identify.py')
        self.assertIn('friction_identification.yaml', node.find('rosparam').get('file'))

    def test_experiment_rejects_bad_configuration_before_motion(self):
        # Import the ROS runner with message names stubbed; configuration checking
        # itself must remain independent of a running master or live hardware.
        rospy = types.ModuleType('rospy')
        msgs = types.ModuleType('z1_msgs.msg')
        for name in ('ControlCommand', 'ControlSample', 'JointGoal', 'JointTrajectory',
                     'MotionResult', 'Status', 'TrajectoryPoint'):
            setattr(msgs, name, type(name, (), {}))
        sys.modules['rospy'] = rospy
        sys.modules['z1_msgs'] = types.ModuleType('z1_msgs')
        sys.modules['z1_msgs.msg'] = msgs
        spec = importlib.util.spec_from_file_location('friction_runner', ROOT / 'python/z1_control/friction_experiment.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        config = yaml.safe_load((ROOT / 'config/friction_identification.yaml').read_text())
        experiment = module.Experiment.__new__(module.Experiment)
        experiment.config = config
        experiment.validate_config()
        for key, value in [('arms', ['left', 'left']), ('joints', [0]),
                           ('repeats', 2), ('static_dwell', float('nan')),
                           ('amplitude', .00001), ('speeds', [.04])]:
            experiment.config = copy.deepcopy(config)
            experiment.config[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                experiment.validate_config()


if __name__ == '__main__':
    unittest.main()
