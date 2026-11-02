import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("z1_launcher", ROOT / "scripts/z1_official.py")
launcher = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(launcher)


class ConfigurationTest(unittest.TestCase):
    def setUp(self):
        self.arms = launcher.load_config(ROOT / "config/arms.yaml")

    def test_unique_ports_and_attachments(self):
        self.assertTrue(self.arms["left"]["has_gripper"])
        self.assertFalse(self.arms["right"]["has_gripper"])
        ports = [a[k] for a in self.arms.values()
                 for k in ("controller_port", "sdk_port", "mcu_local_port")]
        self.assertEqual(len(set(ports)), 6)

    def test_runtime_isolation_preserves_labels(self):
        with tempfile.TemporaryDirectory() as directory:
            for name, arm in self.arms.items():
                run = launcher.prepare_runtime(directory, name, arm, ROOT.parent / "controller/config")
                root = ET.parse(run.parent / "config/config.xml").getroot()
                self.assertEqual(root.findtext("IP"), arm["mcu_ip"])
                self.assertEqual(root.findtext("Port"), str(arm["mcu_local_port"]))
                states = run.parent / "config/savedArmStates.csv"
                states.write_text("user_label, 0,0,0,0,0,0\n")
                launcher.prepare_runtime(directory, name, arm, ROOT.parent / "controller/config")
                self.assertTrue(states.read_text().startswith("user_label"))
            self.assertNotEqual(Path(directory, "left/config"), Path(directory, "right/config"))

    def test_rejects_port_overlap(self):
        with tempfile.TemporaryDirectory() as directory:
            import yaml
            self.arms["right"]["sdk_port"] = self.arms["left"]["sdk_port"]
            path = Path(directory, "bad.yaml")
            path.write_text(yaml.safe_dump({"arms": self.arms}))
            with self.assertRaises(ValueError):
                launcher.load_config(path)

    def test_supervisor_processes_and_cleanup(self):
        self.supervisor_case(False)

    def test_child_failure_does_not_stop_other_arm(self):
        self.supervisor_case(True)

    def supervisor_case(self, crash_left):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            fake = directory / "fake_controller.py"
            fake.write_text("""#!/usr/bin/env python3
import os, pathlib, signal, time
p = pathlib.Path.cwd().parent / 'started'
p.write_text(str(os.getpid())+' '+os.environ['Z1_CONTROLLER_PORT']+' '+os.environ['Z1_SDK_PORT'])
def done(*_):
    p.with_name('stopped').touch()
    raise SystemExit(0)
signal.signal(signal.SIGINT, done)
while True: time.sleep(.02)
""")
            fake.chmod(0o755)
            proc = subprocess.Popen([sys.executable, str(ROOT / "scripts/z1_official.py"),
                "controllers", "--config", str(ROOT / "config/arms.yaml"),
                "--runtime-dir", str(directory / "runtime"),
                "--template-dir", str(ROOT.parent / "controller/config"),
                "--controller-bin", str(fake)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                deadline = time.monotonic() + 5
                while not all((directory / f"runtime/{a}/started").exists() for a in self.arms):
                    if proc.poll() is not None or time.monotonic() > deadline:
                        self.fail("supervisor did not start both controller processes")
                    time.sleep(.02)
                for name, arm in self.arms.items():
                    tokens = (directory / f"runtime/{name}/started").read_text().split()
                    self.assertEqual(tokens[1:], [str(arm["controller_port"]), str(arm["sdk_port"])])
                if crash_left:
                    left_pid = int((directory / "runtime/left/started").read_text().split()[0])
                    right_pid = int((directory / "runtime/right/started").read_text().split()[0])
                    os.kill(left_pid, signal.SIGTERM)
                    time.sleep(.25)
                    self.assertIsNone(proc.poll())
                    os.kill(right_pid, 0)
                proc.send_signal(signal.SIGTERM)
                out, err = proc.communicate(timeout=8)
                self.assertEqual(proc.returncode, 1 if crash_left else 0, (out, err))
                for name in self.arms:
                    if not (crash_left and name == "left"):
                        self.assertTrue((directory / f"runtime/{name}/stopped").exists())
            finally:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGTERM)
                    proc.communicate(timeout=8)


if __name__ == "__main__":
    unittest.main()
