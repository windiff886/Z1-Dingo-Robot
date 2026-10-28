"""Loopback-only MCU protocol fixture for ROS integration tests.

Exercises the production UDP handshake and codec. Feedback follows commanded
q/dq directly; no dynamics, contact, or hardware behavior is represented.
This file is not installed or exposed as a runtime transport.
"""

import ipaddress
import math
import socket
import struct
import threading


def unused_local_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


class McuFixture:
    def __init__(self, address, has_gripper=False, gripper_q=0.0):
        if not ipaddress.ip_address(address).is_loopback:
            raise ValueError('MCU test fixtures require a loopback address')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind((address, 8880))
        except BaseException:
            self.socket.close()
            raise
        self.socket.settimeout(0.02)
        self.stages = {}
        self.q = [0.0, 0.8, -1.1, 0.3, 0.0, 0.0]
        self.has_gripper = has_gripper
        self.gripper_q = gripper_q
        self.packets = 0
        self.commands = 0
        self.error = None
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            while not self.stopping.is_set():
                try:
                    data, source = self.socket.recvfrom(1500)
                except socket.timeout:
                    continue
                if not ipaddress.ip_address(source[0]).is_loopback:
                    continue
                self.packets += 1
                if len(data) == 1:
                    stage = self.stages.get(source, 0)
                    # One extra motor is paired on joint 2; a gripper adds one.
                    motors = 8 if self.has_gripper else 7
                    reply = bytes((0x20, motors)) if stage % 2 == 0 else bytes([1] * (motors + 2))
                    self.stages[source] = stage + 1
                elif len(data) in (120, 140):
                    values = [struct.unpack_from('<5f', data, joint * 20)
                              for joint in range(6)]
                    if not all(math.isfinite(v) for joint in values for v in joint):
                        raise ValueError('Nonfinite MCU command in test')
                    if self.has_gripper:
                        gripper = struct.unpack_from('<5f', data, 6 * 20)
                        if not all(math.isfinite(v) for v in gripper) or not -1 <= gripper[2] <= 0:
                            raise ValueError('Invalid gripper command reached MCU fixture')
                    self.commands += 1
                    active = any(v != 0.0 for joint in values for v in joint)
                    if active:
                        self.q = [joint[2] for joint in values]
                    reply = b''.join(
                        struct.pack('<4f', values[j][0] if active else 0.0,
                                    values[j][1] if active else 0.0,
                                    0.0, self.q[j]) for j in range(6))
                    if self.has_gripper:
                        reply += struct.pack('<4f', 0.0, 0.0, 0.0, self.gripper_q)
                else:
                    continue
                self.socket.sendto(reply, source)
        except BaseException as error:
            self.error = error

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=1)
        self.socket.close()
