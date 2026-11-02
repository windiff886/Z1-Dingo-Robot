"""Two real official controllers and SDK clients; only MCU IO is replaced."""
import concurrent.futures
import os
import re
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import yaml

from test_sdk_loopback import unused_port


def main():
    launcher, controller, client, fixture = map(lambda p: str(Path(p).resolve()), sys.argv[1:])
    source = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        arms = {}
        ports = set()
        for name, ip, grip in (("left", "127.0.0.2", True), ("right", "127.0.0.3", False)):
            allocated = []
            while len(allocated) < 3:
                p = unused_port()
                if p not in ports:
                    ports.add(p)
                    allocated.append(p)
            arms[name] = dict(mcu_ip=ip, mcu_local_port=allocated[0],
                              controller_port=allocated[1], sdk_port=allocated[2],
                              has_gripper=grip, load_kg=0.0)
        config = root / "arms.yaml"
        config.write_text(yaml.safe_dump({"arms": arms}))
        env = dict(os.environ, LD_PRELOAD=fixture)
        with (root / "controllers.log").open("w+") as log:
            process = subprocess.Popen([sys.executable, launcher, "controllers",
                "--config", str(config), "--runtime-dir", str(root / "runtime"),
                "--template-dir", str(source.parent / "controller/config"),
                "--controller-bin", controller], env=env, stdout=log, stderr=log)
            try:
                time.sleep(1.5)  # upstream main waits one second before starting SDK communication
                if process.poll() is not None:
                    log.seek(0)
                    raise AssertionError(log.read())

                def call(name, expected):
                    result = subprocess.run([sys.executable, launcher, "client", "--arm", name,
                        "--config", str(config), "--runtime-dir", str(root / "runtime"),
                        "--client-bin", client],
                        input="state\npassive\nstate\nquit\n", text=True,
                        capture_output=True, timeout=10)
                    assert result.returncode == 0, (name, result.stdout, result.stderr)
                    positions = re.findall(r"\bq=\s*([-0-9.]+)", result.stdout)
                    assert positions and all(abs(float(q)-expected) < 1e-6 for q in positions), (name, result.stdout)
                    assert "PASSIVE acknowledged" in result.stdout, (name, result.stdout)

                with concurrent.futures.ThreadPoolExecutor(2) as pool:
                    left = pool.submit(call, "left", .125)
                    right = pool.submit(call, "right", -.25)
                    left.result()
                    right.result()
                assert process.poll() is None
                print("PASS: two actual official FSM/controllers + two actual SDK clients; MCU boundary replaced, no hardware")
            finally:
                process.send_signal(signal.SIGTERM) if process.poll() is None else None
                process.wait(timeout=8)
                if process.returncode:
                    log.seek(0)
                    print(log.read(), file=sys.stderr)
            assert process.returncode == 0


if __name__ == "__main__":
    main()
