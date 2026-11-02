"""Actual official SDK, two loopback-only controller protocol fixtures. No MCU."""
import concurrent.futures
from pathlib import Path
import socket
import struct
import re
import subprocess
import sys
import threading


class ControllerFixture:
    def __init__(self, position):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(.05)
        self.port = self.socket.getsockname()[1]
        self.position = position
        self.frames = []
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        while not self.done.is_set():
            try:
                data, source = self.socket.recvfrom(1500)
            except socket.timeout:
                continue
            if len(data) != 147 or source[0] != "127.0.0.1":
                continue
            state = struct.unpack_from("<I", data, 2)[0]
            self.frames.append((state, source[1], data))
            # Acknowledge state requests; INVALID startup is reported as PASSIVE.
            reply = struct.pack("<2sI", b"\xfe\xff", state or 1)
            for i in range(7):
                reply += struct.pack("<4f", 0.0, 0.0, 0.0, self.position + i * .01)
                reply += b"\x19\0\0" * 2
            reply += struct.pack("<6d", 0, 0, 0, .3, 0, .3)
            self.socket.sendto(reply, source)

    def close(self):
        self.done.set()
        self.thread.join()
        self.socket.close()


def unused_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def run_client(executable, fixture, gripper):
    port = unused_port()
    proc = subprocess.run([executable, "--controller-port", str(fixture.port),
        "--sdk-port", str(port), "--has-gripper", str(gripper).lower()],
        input="state\npassive\nstate\nquit\n", text=True, capture_output=True, timeout=12)
    assert proc.returncode == 0, (proc.stdout, proc.stderr)
    positions = re.findall(r"\bq=\s*([-0-9.]+)", proc.stdout)
    assert positions and all(abs(float(q)-fixture.position) < 1e-6 for q in positions), proc.stdout
    assert "PASSIVE acknowledged" in proc.stdout, proc.stdout
    assert fixture.frames and all(f[1] == port for f in fixture.frames), fixture.frames
    assert all(f[0] in (0, 1) for f in fixture.frames), "startup issued a motion state"
    return port


def main():
    executable = str(Path(sys.argv[1]).resolve())
    left, right = ControllerFixture(.125), ControllerFixture(-.25)
    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            a = pool.submit(run_client, executable, left, True)
            b = pool.submit(run_client, executable, right, False)
            assert a.result() != b.result()
        print("PASS: two actual SDK instances, separate ports/feedback, official PASSIVE, no motion on startup")
    finally:
        left.close()
        right.close()


if __name__ == "__main__":
    main()
