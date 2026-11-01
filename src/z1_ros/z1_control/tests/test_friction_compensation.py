#!/usr/bin/env python3
"""Profile persistence, honest A/B metrics and the real runner over stubbed ROS."""
import copy
import importlib.util
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import yaml
from test_friction_identification import ROOT, profile as sample_profile
from test_friction_runner import Plant, message_module
from z1_control.friction_comparison import round_trip, compare_joint, plot_joint
from z1_control.friction_profile import merge_identification, export_profile


class Profiles(unittest.TestCase):
    def setUp(self):
        self.profile = yaml.safe_load((ROOT / 'config/friction_compensation.yaml').read_text())
        self.report = dict(
            status='complete',
            models={'left': dict(has_gripper=True, revision='left-model')},
            results={
                'left': {
                    'joint6': dict(
                        usable=True,
                        coulomb_nm=0.25,
                        viscous_nm_s_per_rad=0.8,
                        static={'usable': False},
                    )
                }
            },
        )

    def test_partial_export_preserves_other_arm_and_makes_backup(self):
        result = merge_identification(self.profile, self.report)
        self.assertEqual(result['arms']['left']['coulomb_nm'], [0, 0, 0, 0, 0, 0.25])
        self.assertEqual(result['arms']['right'], self.profile['arms']['right'])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'friction.yaml'
            original = yaml.safe_dump(self.profile)
            path.write_text(original)
            recording = Path(directory) / 'result.yaml'
            recording.write_text(yaml.safe_dump(self.report))
            backup = export_profile(recording, path)
            self.assertEqual(backup.read_text(), original)
            self.assertEqual(
                yaml.safe_load(path.read_text())['arms']['left']['model_revision'], 'left-model'
            )

    def test_rejected_nonfinite_and_mixed_model_results_do_not_replace_profile(self):
        for update in (
            {'usable': False},
            {'usable': 'false'},
            {'coulomb_nm': float('nan')},
            {'viscous_nm_s_per_rad': -0.2},
        ):
            report = copy.deepcopy(self.report)
            report['results']['left']['joint6'].update(update)
            with self.subTest(update=update), self.assertRaises(ValueError):
                merge_identification(self.profile, report)
        self.profile['arms']['left']['coulomb_nm'][0] = 0.1
        self.profile['arms']['left']['model_revision'] = 'old-model'
        with self.assertRaisesRegex(ValueError, 'another model'):
            merge_identification(self.profile, self.report)


def synthetic_rows(improved=True):
    rows = []
    center = np.array([0, 0.8, -1.2, 0.4, 0, 0])
    points = round_trip(center, 5, 0.1, 0.04, 0.3, 2.0)
    for repeat in range(2):
        for case in ('uncompensated', 'compensated'):
            lag = 0.01 if case == 'uncompensated' or not improved else 0.002
            for t in np.arange(0, points[-1].time, 0.004):
                q, v, a = sample_profile(points, t)
                actual = q.copy()
                actual[5] -= lag * np.tanh(v[5] / 0.01)
                rows.append(
                    dict(
                        kind=case,
                        repeat=repeat,
                        received_at=1000 + t,
                        position=actual,
                        command_position=q,
                        command_velocity=v,
                        friction_compensation_enabled=case == 'compensated',
                    )
                )
    return rows


class Metrics(unittest.TestCase):
    def test_improvement_and_no_improvement_are_both_reported_honestly(self):
        config = dict(repeats=2, speeds=[0.04], max_sample_gap=0.02, minimum_rms_reduction=0.1)
        result = compare_joint(synthetic_rows(), 5, config)
        self.assertTrue(result['improved'])
        self.assertGreater(result['position_rms_reduction_percent'], 70)
        equal = compare_joint(synthetic_rows(False), 5, config)
        self.assertFalse(equal['improved'])
        self.assertEqual(equal['verdict'], 'no_clear_improvement')
        bad = synthetic_rows()
        bad[100]['friction_compensation_enabled'] = True
        with self.assertRaisesRegex(ValueError, 'switch'):
            compare_joint(bad, 5, config)


class ComparisonRunner(unittest.TestCase):
    def exercise(self, directory, zeros=False, missing=False):
        msgs = message_module()
        plant = Plant(msgs)
        ros = plant.ros()

        def switch(enabled):
            plant.friction_enabled = enabled
            return SimpleNamespace(success=True, message='applied')

        ros.ServiceProxy = lambda *a, **k: switch
        ros.wait_for_service = lambda *a, **k: None
        if zeros:
            original = ros.get_param

            def param(name):
                value = original(name)
                if name.endswith('/friction'):
                    value.update(coulomb_nm=[0] * 6, viscous_nm_s_per_rad=[0] * 6)
                return value

            ros.get_param = param
        srv = ModuleType('std_srvs.srv')
        srv.SetBool = type('SetBool', (), {})
        modules = {
            'rospy': ros,
            'z1_msgs': ModuleType('z1_msgs'),
            'z1_msgs.msg': msgs,
            'std_srvs': ModuleType('std_srvs'),
            'std_srvs.srv': srv,
        }
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location(
                'z1_control.friction_experiment', ROOT / 'python/z1_control/friction_experiment.py'
            )
            identify = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(identify)
            with patch.dict(sys.modules, {'z1_control.friction_experiment': identify}):
                spec = importlib.util.spec_from_file_location(
                    'friction_compare', ROOT / 'scripts/friction_compare.py'
                )
                compare = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(compare)
                config = yaml.safe_load((ROOT / 'config/friction_identification.yaml').read_text())
                config.update(
                    yaml.safe_load((ROOT / 'config/friction_comparison.yaml').read_text())
                )
                config.update(
                    arms=['left'],
                    joints=[6],
                    speeds=[0.08],
                    repeats=2,
                    amplitude=0.08,
                    startup_delay=0.0,
                    settle_seconds=0.02,
                    output_directory=directory,
                )
                experiment = compare.Comparison(config)
                plant.advance()
                if missing:
                    original_consume = experiment.consume

                    def consume(arm, sample):
                        if plant.sequence > 50:
                            plant.drop_next = True
                        return original_consume(arm, sample)

                    experiment.consume = consume
                with patch.object(
                    identify.time, 'monotonic', side_effect=lambda: plant.time
                ), patch.object(identify.time, 'sleep', side_effect=plant.advance):
                    experiment.run()
                return experiment, plant

    def test_identical_trajectories_switch_restore_and_no_invented_improvement(self):
        with tempfile.TemporaryDirectory() as directory:
            experiment, plant = self.exercise(directory)
            self.assertEqual(experiment.report['status'], 'complete', experiment.report)
            self.assertEqual(
                [t['case'] for t in experiment.report['completed_trials']],
                ['uncompensated', 'compensated', 'compensated', 'uncompensated'],
            )
            templates = [m for topic, m, _ in plant.commands if topic.endswith('/joint_trajectory')]
            arrays = [
                [(p.time_from_start, p.positions, p.velocities, p.accelerations) for p in m.points]
                for m in templates
            ]
            self.assertEqual(arrays, [arrays[0]] * 4)
            self.assertFalse(plant.friction_enabled)
            self.assertEqual(plant.mode, plant.msgs.Status.HOLDING)
            result = yaml.safe_load((experiment.directory / 'comparison.yaml').read_text())
            self.assertFalse(result['all_improved'])  # fixture tracks perfectly in both cases
            self.assertTrue((experiment.directory / 'left_joint6.png').is_file())

    def test_zero_profile_refuses_motion_and_missing_feedback_stops(self):
        with tempfile.TemporaryDirectory() as directory:
            experiment, plant = self.exercise(directory, zeros=True)
            self.assertEqual(experiment.report['status'], 'failed')
            self.assertIn('coefficients are zero', experiment.report['error'])
            self.assertEqual(plant.commands, [])
        with tempfile.TemporaryDirectory() as directory:
            experiment, plant = self.exercise(directory, missing=True)
            self.assertEqual(experiment.report['status'], 'failed')
            self.assertEqual(plant.mode, plant.msgs.Status.AWAIT_RESUME)
            self.assertFalse(experiment.report['switches_restored'])


if __name__ == '__main__':
    unittest.main()
