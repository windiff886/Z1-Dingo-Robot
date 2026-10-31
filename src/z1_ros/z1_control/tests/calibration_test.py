#!/usr/bin/env python3
"""Exercise the C++ calibration executable and vendor library on loopback only."""
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import yaml

BINARY = str(Path(sys.argv.pop(1)).resolve())
HOME = [0.0, 0.0, 0.0, -0.07435, 0.0, 0.0]


def local_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


class Firmware:
    def __init__(self, address, version):
        assert address in ('127.0.0.1', '127.0.0.2')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((address, 8880))
        self.socket.settimeout(0.01)
        self.version = version
        self.reply = 'success'
        self.joints = 6
        self.moving = self.nonfinite = self.motor_error = False
        self.motor_error_code = 1
        self.drifting = False
        self.reply_delay = 0.0
        self.velocity_noise = False
        self.calibrated = False
        self.bad_zero_joint = 3
        self.initial_positions = [0.3] * 6
        self.calibrated_positions = HOME.copy()
        self.calibrations = 0
        self.received_packets = []
        self.commands = []
        self.error = None
        self.attempted = threading.Event()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=1)
        self.socket.close()

    def run(self):
        try:
            while not self.stopping.is_set():
                try:
                    data, source = self.socket.recvfrom(1500)
                except socket.timeout:
                    continue
                self.received_packets.append(data)
                if data == b'\x01':
                    self.calibrations += 1
                    self.attempted.set()
                    self.calibrated = True
                    if self.reply == 'drop':
                        continue
                    reply = {'reject': b'\xff', 'malformed': b'\x10\x00',
                             'unexpected': b'\x03'}.get(self.reply, b'\x10')
                    if self.reply == 'foreign':
                        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as other:
                            other.sendto(reply, source)
                        continue
                elif data == b'\x02':
                    motors = self.joints + 1
                    # Real MCU returns the header and motor flags together,
                    # even when IOUDP first asks for only the two-byte header.
                    reply = bytes((0x20, motors)) + bytes([1] * motors)
                elif len(data) == self.joints * 20:
                    command = struct.unpack('<' + 'f' * (len(data) // 4), data)
                    self.commands.append(command)
                    reply = b''
                    for joint in range(self.joints):
                        position = (0.0 if joint == 6 else
                                    self.calibrated_positions[joint] if self.calibrated else
                                    self.initial_positions[joint])
                        if self.calibrated and self.reply == 'bad_zero' and joint == self.bad_zero_joint:
                            position = HOME[joint] - 0.08
                        if self.drifting:
                            position += 0.02 * len(self.commands)
                        if self.nonfinite or joint == 6:
                            position = float('nan')
                        velocity = 0.1 if self.moving else 0.0
                        if self.velocity_noise:
                            velocity += 0.08 if len(self.commands) % 2 else -0.08
                        reply += struct.pack('<4f', 0.0, velocity, 0.0, position)
                        if self.version == 2:
                            error = self.motor_error_code if self.motor_error and joint == 0 else 0
                            reply += bytes((25, error, 0, 25, 0, 0))
                    time.sleep(self.reply_delay)
                else:
                    raise RuntimeError(f'unexpected packet: {data!r}')
                self.socket.sendto(reply, source)
        except BaseException as error:
            self.error = error


class DualArmCalibration(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='z1-calibration-test-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.left = Firmware('127.0.0.1', 1)
        self.addCleanup(self.left.close)
        self.right = Firmware('127.0.0.2', 2)
        self.addCleanup(self.right.close)
        ports = {local_port()}
        while len(ports) < 2:
            ports.add(local_port())
        self.communication = {
            side + '_arm': {'controller': {'mcu_ip': address, 'local_port': port,
                                          'receive_budget_us': 5000}}
            for side, address, port in zip(('left', 'right'), ('127.0.0.1', '127.0.0.2'), ports)
        }
        self.options = dict(communication_config='communication.yaml', arms=['left', 'right'],
                            sample_count=3, sample_interval_ms=10, feedback_timeout_ms=300,
                            receive_budget_us=5000, reply_timeout_ms=50, max_joint_speed=0.02,
                            max_position_drift=0.01, zero_reference=HOME.copy(),
                            zero_tolerance=0.05)

    def tearDown(self):
        for peer in (self.left, self.right):
            self.assertIsNone(peer.error)
            self.assertTrue(all(value == 0 for command in peer.commands for value in command),
                            'calibration tool sent an active motor command')

    def arguments(self):
        (self.root / 'communication.yaml').write_text(yaml.safe_dump(self.communication))
        config = self.root / 'calibration.yaml'
        config.write_text(yaml.safe_dump(self.options))
        return [BINARY, str(config)]

    def run_tool(self, confirmation='left ZERO\nright ZERO\n'):
        return subprocess.run(self.arguments(), input=confirmation, text=True,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=12)

    def test_both_arms_use_official_calibration_and_verify_zero_v1_v2(self):
        result = self.run_tool()
        self.assertEqual(result.returncode, 0, result.stdout)
        for side, peer in (('left', self.left), ('right', self.right)):
            self.assertIn(f'{side}: SUCCEEDED', result.stdout)
            self.assertEqual(peer.calibrations, 1)

    def test_calibration_uses_maintenance_receive_budget(self):
        for side, peer in (('left', self.left), ('right', self.right)):
            self.communication[side + '_arm']['controller']['receive_budget_us'] = 1000
            peer.reply_delay = 0.0025
        result = self.run_tool()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(self.left.calibrations, 1)
        self.assertEqual(self.right.calibrations, 1)

    def test_receive_budget_must_leave_time_in_sampling_interval(self):
        self.options['receive_budget_us'] = self.options['sample_interval_ms'] * 1000
        result = self.run_tool()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('receive_budget_us must be smaller than sample_interval_ms', result.stdout)
        self.assertEqual(self.left.commands + self.right.commands, [])
        self.assertEqual(self.left.calibrations + self.right.calibrations, 0)

    def test_positive_timing_values_use_relationships_not_arbitrary_caps(self):
        self.options['arms'] = ['left']
        original = self.options.copy()
        cases = (
            {'sample_count': 1001, 'feedback_timeout_ms': 20000},
            {'sample_interval_ms': 101, 'feedback_timeout_ms': 1000},
            {'feedback_timeout_ms': 50},
            {'reply_timeout_ms': 1001},
            {'sample_interval_ms': 20, 'receive_budget_us': 15000},
        )
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as occupied:
            occupied.bind(('0.0.0.0', self.communication['left_arm']['controller']['local_port']))
            for values in cases:
                with self.subTest(values=values):
                    self.options = dict(original, **values)
                    result = self.run_tool()
                    self.assertNotEqual(result.returncode, 0, result.stdout)
                    self.assertIn('UDP bind failed', result.stdout)
        self.assertEqual(self.left.received_packets + self.right.received_packets, [])

    def test_sampling_window_multiplication_cannot_wrap(self):
        self.options.update(sample_count=4294967295, sample_interval_ms=2,
                            feedback_timeout_ms=4294967295, receive_budget_us=1000)
        result = self.run_tool()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('feedback_timeout_ms must exceed the sampling window', result.stdout)
        self.assertEqual(self.left.received_packets + self.right.received_packets, [])

    def test_reply_timeout_must_fit_transport_microseconds(self):
        self.options['reply_timeout_ms'] = 4294968
        result = self.run_tool()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('reply_timeout_ms is outside its supported range', result.stdout)
        self.assertEqual(self.left.received_packets + self.right.received_packets, [])

    def test_invalid_reference_rejected_before_connecting(self):
        for reference in (None, [0.0] * 5, [0.0] * 7,
                          [0.0, 0.0, 0.0, float('nan'), 0.0, 0.0],
                          [0.0, 0.0, 0.0, float('inf'), 0.0, 0.0]):
            with self.subTest(reference=reference):
                self.options['zero_reference'] = reference
                result = self.run_tool()
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertIn('zero_reference must contain six finite angles', result.stdout)
                self.assertEqual(self.left.received_packets + self.right.received_packets, [])

    def test_j4_checks_both_sides_of_home_and_rejects_zero(self):
        self.options['arms'] = ['left']
        for offset, passed in ((-0.049, True), (0.049, True),
                               (-0.051, False), (0.051, False), (0.07435, False)):
            with self.subTest(offset=offset):
                self.left.calibrated = False
                self.left.calibrated_positions[3] = HOME[3] + offset
                result = self.run_tool('left ZERO\n')
                self.assertEqual(result.returncode, 0 if passed else 1, result.stdout)
                self.assertIn('left: SUCCEEDED' if passed else
                              'left: ACKNOWLEDGED_UNVERIFIED', result.stdout)
                if not passed:
                    self.assertIn('reference=-0.07435', result.stdout)

    def test_check_only_verifies_feedback_without_calibrating(self):
        args = self.arguments()
        result = subprocess.run([args[0], '--check-only', args[1]],
                                input='left ZERO\nright ZERO\n', text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=5)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(result.stdout.count('passive feedback check passed'), 2)
        self.assertEqual(self.left.calibrations + self.right.calibrations, 0)

    def test_stationary_velocity_noise_uses_official_filter(self):
        self.options['arms'] = ['right']
        self.right.velocity_noise = True
        result = self.run_tool('right ZERO\n')
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(self.right.calibrations, 1)

    def test_position_motion_blocks_calibration_even_with_zero_reported_velocity(self):
        self.options['arms'] = ['right']
        self.right.drifting = True
        result = self.run_tool('right ZERO\n')
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('arm joints are moving', result.stdout)
        self.assertEqual(self.right.calibrations, 0)

    def test_single_arm_selection_does_not_connect_or_calibrate_the_other(self):
        self.options['arms'] = ['right']
        self.right.joints = 7  # A nonfinite gripper position does not enter six-axis verification.
        result = self.run_tool('right ZERO\n')
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('right: SUCCEEDED', result.stdout)
        self.assertEqual(self.left.commands, [])
        self.assertEqual(self.left.calibrations, 0)

    def test_confirmation_or_eof_never_sends_calibration(self):
        for confirmation in ('', 'yes\n', 'right ZERO\n'):
            result = self.run_tool(confirmation)
            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertEqual(self.left.calibrations + self.right.calibrations, 0)

    def test_second_port_busy_prevents_calibration_of_either_arm(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as occupied:
            occupied.bind(('0.0.0.0', self.communication['right_arm']['controller']['local_port']))
            result = self.run_tool()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('UDP bind failed', result.stdout)
        self.assertEqual(self.left.calibrations + self.right.calibrations, 0)

    def test_bad_config_rejected_before_connecting(self):
        self.communication['right_arm']['controller']['mcu_ip'] = '127.0.0.1'
        result = self.run_tool()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertEqual(self.left.commands + self.right.commands, [])
        self.assertEqual(self.left.calibrations + self.right.calibrations, 0)

    def test_moving_drifting_or_nonfinite_joints_block_calibration(self):
        for attribute in ('moving', 'drifting', 'nonfinite'):
            setattr(self.left, attribute, True)
            result = self.run_tool()
            setattr(self.left, attribute, False)
            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn('left: NOT_ATTEMPTED', result.stdout)
            self.assertEqual(self.left.calibrations + self.right.calibrations, 0)

    def test_motor_fault_blocks_calibration(self):
        self.options['arms'] = ['right']
        self.right.motor_error = True
        result = self.run_tool('right ZERO\n')
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('motor fault', result.stdout)
        self.assertEqual(self.right.calibrations, 0)

    def test_vendor_unclassified_motor_status_does_not_block_calibration(self):
        self.options['arms'] = ['right']
        self.right.motor_error = True
        self.right.motor_error_code = 0x40
        result = self.run_tool('right ZERO\n')
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('right: SUCCEEDED', result.stdout)
        self.assertEqual(self.right.calibrations, 1)

    def test_known_motor_error_with_extra_status_bits_blocks_calibration(self):
        self.options['arms'] = ['right']
        self.right.motor_error = True
        self.right.motor_error_code = 0x41
        result = self.run_tool('right ZERO\n')
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('MCU motor fault: joint=1 motor=1 temperature=25 error=0x41 connection=0',
                      result.stdout)
        self.assertIn('right: NOT_ATTEMPTED', result.stdout)
        self.assertEqual(self.right.calibrations, 0)

    def test_device_error_is_distinguished_from_unknown(self):
        self.left.reply = 'reject'
        result = self.run_tool()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('left: DEVICE_REJECTED', result.stdout)
        self.assertEqual(self.left.calibrations, 1)
        self.assertEqual(self.right.calibrations, 0)

    def test_lost_malformed_foreign_or_unexpected_reply_is_unknown_without_retry(self):
        for mode in ('drop', 'malformed', 'foreign', 'unexpected'):
            self.left.reply = mode
            previous = self.left.calibrations
            result = self.run_tool()
            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn('left: UNKNOWN', result.stdout)
            self.assertEqual(self.left.calibrations, previous + 1)
            self.assertEqual(self.right.calibrations, 0)

    def test_acknowledgment_without_zero_feedback_is_not_success(self):
        self.left.reply = 'bad_zero'
        result = self.run_tool()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('left: ACKNOWLEDGED_UNVERIFIED', result.stdout)
        self.assertIn('zero_tolerance=0.05 rad: J4=-0.15435 rad', result.stdout)
        self.assertIn('reference=-0.07435, error=-0.08 rad', result.stdout)
        self.assertIn('six-joint feedback [rad]:', result.stdout)
        self.assertEqual(self.left.calibrations, 1)
        self.assertEqual(self.right.calibrations, 0)

    def test_every_main_joint_is_checked_after_calibration(self):
        self.left.reply = 'bad_zero'
        for joint in range(6):
            with self.subTest(joint=joint + 1):
                self.left.bad_zero_joint = joint
                self.left.calibrated = False
                before = self.left.calibrations
                result = self.run_tool()
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertIn('left: ACKNOWLEDGED_UNVERIFIED', result.stdout)
                self.assertIn(f'J{joint + 1}=', result.stdout)
                self.assertIn('error=-0.08 rad', result.stdout)
                self.assertEqual(self.left.calibrations, before + 1)
                self.assertEqual(self.right.calibrations, 0)

    def test_wire_calibration_is_one_whole_arm_byte_with_passive_commands(self):
        self.options['arms'] = ['left']
        self.left.joints = 7
        self.left.version = 2
        self.left.initial_positions = [-0.0011741, 0.0010856, -0.0002773,
                                       -0.074611, -0.10159, 0.0019411]
        result = self.run_tool('left ZERO\n')
        self.assertEqual(result.returncode, 0, result.stdout)
        packets = self.left.received_packets
        self.assertEqual(packets.count(b'\x01'), 1)
        marker = packets.index(b'\x01')
        self.assertEqual(packets[:2], [b'\x02', b'\x02'])
        self.assertTrue(all(packet == bytes(140) for packet in packets[2:marker]))
        self.assertGreaterEqual(len(packets[2:marker]), self.options['sample_count'])
        self.assertTrue(all(packet == bytes(140) for packet in packets[marker + 1:]))
        self.assertGreaterEqual(len(packets[marker + 1:]), self.options['sample_count'])
        self.assertEqual(self.right.received_packets, [])

    def test_partial_success_is_reported_per_arm(self):
        self.right.reply = 'drop'
        result = self.run_tool()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn('left: SUCCEEDED', result.stdout)
        self.assertIn('right: UNKNOWN', result.stdout)
        self.assertEqual(self.left.calibrations, 1)
        self.assertEqual(self.right.calibrations, 1)

    def test_interrupt_during_vendor_wait_does_not_start_second_arm(self):
        process = subprocess.Popen(self.arguments(), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True)
        try:
            process.stdin.write('left ZERO\nright ZERO\n')
            process.stdin.flush()
            self.assertTrue(self.left.attempted.wait(timeout=5))
            process.send_signal(signal.SIGINT)
            output, _ = process.communicate(timeout=5)
            self.assertNotEqual(process.returncode, 0, output)
            self.assertIn('left: ACKNOWLEDGED_UNVERIFIED', output)
            self.assertEqual(self.left.calibrations, 1)
            self.assertEqual(self.right.calibrations, 0)
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()


if __name__ == '__main__':
    unittest.main(verbosity=2)
