#!/usr/bin/env python3
"""Hardware A/B test: identical joint trajectories and gains, friction switch only."""
import numpy as np
import rospy
from std_srvs.srv import SetBool
from z1_control.friction_experiment import Experiment
from z1_control.friction_comparison import CASES, round_trip, analyze_recording


class Comparison(Experiment):
    require_uncompensated = False
    minimum_repeats = 2
    minimum_speeds = 1

    def __init__(self, config):
        super().__init__(config)
        self.report.update(
            recording_type='friction_comparison',
            completed_trials=[],
            trajectory_templates={},
            control_law='position/joint_trajectory',
        )
        self.switches = {
            a.side: rospy.ServiceProxy(a.ns + '/controller/set_friction_compensation', SetBool)
            for a in self.arms
        }
        self.original = {}
        self.save()

    def validate_config(self):
        super().validate_config()
        if len(self.config['speeds']) != 1:
            raise ValueError('comparison uses exactly one speed')
        if self.config['repeats'] % 2:
            raise ValueError('comparison requires an even number of AB/BA pairs')
        if (
            not np.isfinite(self.config['minimum_rms_reduction'])
            or not 0 < self.config['minimum_rms_reduction'] < 1
        ):
            raise ValueError('minimum_rms_reduction must be in (0,1)')

    def preflight(self):
        super().preflight()
        for arm in self.arms:
            rospy.wait_for_service(
                arm.ns + '/controller/set_friction_compensation',
                timeout=self.config['startup_timeout'],
            )
            self.original[arm.side] = bool(arm.view()[1].friction_compensation_enabled)
            profile = self.models[arm.side]['friction']
            for j in self.config['joints']:
                if (
                    profile['coulomb_nm'][j - 1] == 0
                    and profile['viscous_nm_s_per_rad'][j - 1] == 0
                ):
                    raise ValueError(
                        '%s J%d: coefficients are zero; identify/export/restart before A/B testing'
                        % (arm.side, j)
                    )
        self.report['initial_switches'] = self.original.copy()

    def switch(self, arm, enabled):
        response = self.switches[arm.side](enabled)
        if not response.success:
            raise RuntimeError(arm.side + ': ' + response.message)
        self.wait(
            lambda: arm.view()[1].friction_compensation_enabled == enabled,
            3.0,
            arm.side + ': friction switch acknowledgment timeout',
        )
        self.wait(
            lambda: arm.view()[0].command_enabled
            and arm.view()[0].friction_compensation_enabled == enabled,
            3.0,
            arm.side + ': actually sent friction switch not observed',
        )
        # Both cases wait identically; service application never edits gains.
        self.pause(self.config['settle_seconds'])

    def consume(self, arm, sample):
        row = super().consume(arm, sample)
        if row['kind'] in CASES:
            if bool(sample.friction_compensation_enabled) != (row['kind'] == 'compensated'):
                raise RuntimeError(arm.side + ': actual compensation switch changed during trial')
            controls = self.models[arm.side]['control_parameters']
            if not (
                np.array_equal(sample.command_kp, controls['mcu_kp'])
                and np.array_equal(sample.command_kd, controls['mcu_kd'])
            ):
                raise RuntimeError(arm.side + ': MCU gains changed during comparison')
        return row

    def run(self):
        completed = False
        try:
            self.preflight()
            for arm in self.arms:
                self.current, self.axis = arm, None
                with arm.lock:
                    arm.collect = True
                center = self.centers[arm.side]
                self.goal(arm, center)
                self.report['trajectory_templates'][arm.side] = {}
                for joint in self.config['joints']:
                    self.axis = joint - 1
                    points = round_trip(
                        center,
                        self.axis,
                        self.config['amplitude'],
                        self.config['speeds'][0],
                        self.config['acceleration'],
                        self.config['jerk'],
                    )
                    self.report['trajectory_templates'][arm.side]['joint' + str(joint)] = [
                        {
                            'time': float(p.time),
                            'position': p.position.tolist(),
                            'velocity': p.velocity.tolist(),
                            'acceleration': p.acceleration.tolist(),
                        }
                        for p in points
                    ]
                    for repeat in range(self.config['repeats']):
                        order = CASES if repeat % 2 == 0 else tuple(reversed(CASES))
                        for case in order:
                            # Identical off-state preparation and approach direction
                            # prevent the previous case's endpoint becoming a new start.
                            self.switch(arm, False)
                            self.goal(arm, center)
                            self.goal(arm, points[0].position)
                            self.switch(arm, case == 'compensated')
                            rospy.loginfo('%s J%d pair%d: %s', arm.side, joint, repeat + 1, case)
                            identifier = self.trajectory(
                                arm,
                                points,
                                dict(
                                    kind=case,
                                    joint=joint,
                                    repeat=repeat,
                                    direction=0,
                                    speed=self.config['speeds'][0],
                                ),
                            )
                            self.finish(arm, identifier, points[-1].time + 10.0)
                            for sample in arm.drain():
                                self.consume(arm, sample)
                            self.report['completed_trials'].append(
                                dict(
                                    command_id=identifier,
                                    arm=arm.side,
                                    joint=joint,
                                    repeat=repeat,
                                    case=case,
                                )
                            )
                            self.file.flush()
                            self.save()
                    self.switch(arm, False)
                    self.goal(arm, center)
                self.axis = None
                self.switch(arm, self.original[arm.side])
                with arm.lock:
                    arm.collect = False
                for sample in arm.drain():
                    self.consume(arm, sample)
            self.report['status'] = 'complete'
            self.report['switches_restored'] = True
            self.save()
            completed = True
        except Exception as error:
            self.shutdown()
            self.report.update(status='failed', error=str(error), switches_restored=False)
            try:
                self.save()
            except OSError as recording_error:
                rospy.logerr('Cannot persist failure: %s', recording_error)
            rospy.logerr('Comparison stopped: %s; partial data: %s', error, self.directory)
        finally:
            for arm in self.arms:
                with arm.lock:
                    arm.collect = False
            if self.file:
                self.file.close()
        if completed:
            report = analyze_recording(self.directory)
            rospy.loginfo(
                'Comparison: all_improved=%s; %s/comparison.yaml',
                report['all_improved'],
                self.directory,
            )


def main():
    rospy.init_node('friction_comparison')
    config = rospy.get_param('~')
    Comparison(config).run()
    rospy.spin()


if __name__ == '__main__':
    main()
