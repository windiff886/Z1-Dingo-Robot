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
import unittest

import yaml

BINARY = str(Path(sys.argv.pop(1)).resolve())


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
        self.drifting = False
        self.calibrated = False
        self.calibrations = 0
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
            stage = 0
            while not self.stopping.is_set():
                try:
                    data, source = self.socket.recvfrom(1500)
                except socket.timeout:
                    continue
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
                    reply = bytes((0x20, motors)) if stage % 2 == 0 else bytes([1] * (motors + 2))
                    stage += 1
                elif len(data) == self.joints * 20:
                    command = struct.unpack('<' + 'f' * (len(data) // 4), data)
                    self.commands.append(command)
                    reply = b''
                    for joint in range(self.joints):
                        position = 0.0 if self.calibrated and self.reply != 'bad_zero' else 0.3
                        if self.drifting:
                            position += 0.02 * len(self.commands)
                        if self.nonfinite or joint == 6:
                            position = float('nan')
                        reply += struct.pack('<4f', 0.0, 0.1 if self.moving else 0.0, 0.0, position)
                        if self.version == 2:
                            reply += bytes((25, int(self.motor_error and joint == 0), 0, 25, 0, 0))
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
                            reply_timeout_ms=50, max_joint_speed=0.02,
                            max_position_drift=0.01, zero_tolerance=0.05)

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
        self.assertEqual(self.right.calibrations, 0)

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
