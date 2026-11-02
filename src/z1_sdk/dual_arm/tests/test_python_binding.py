"""Port-extended official binding against separate loopback controller fixtures."""
import os
from pathlib import Path
import subprocess
import sys

from test_sdk_loopback import ControllerFixture, unused_port


def main():
    module = Path(sys.argv[1]).resolve()
    left, right = ControllerFixture(.125), ControllerFixture(-.25)
    a, b = unused_port(), unused_port()
    try:
        code = f"""
import unitree_arm_interface as sdk
left = sdk.ArmInterface(True, {left.port}, {a}, 0.0)
right = sdk.ArmInterface(hasGripper=False, controllerPort={right.port}, sdkPort={b}, load=0.0)
try:
    left.loopOn()
    right.loopOn()
    assert left.setFsm(sdk.ArmFSMState.PASSIVE)
    assert right.setFsm(sdk.ArmFSMState.PASSIVE)
    assert abs(left.lowstate.getQ()[0] - .125) < 1e-6
    assert abs(right.lowstate.getQ()[0] + .25) < 1e-6
    assert left._ctrlComp.armModel.forwardKinematics(left.lowstate.getQ(), 6).shape == (4,4)
finally:
    left.loopOff()
    right.loopOff()
print('PASS: official Python API, per-arm ports and feedback, gripper/no-gripper models')
"""
        result = subprocess.run([sys.executable, "-c", code], text=True, capture_output=True,
            env=dict(os.environ, PYTHONPATH=str(module.parent)), timeout=12)
        assert result.returncode == 0, (result.stdout, result.stderr)
        assert left.frames and right.frames
        assert all(f[1] == a for f in left.frames) and all(f[1] == b for f in right.frames)
        print(result.stdout.strip())
    finally:
        left.close()
        right.close()


if __name__ == "__main__":
    main()
