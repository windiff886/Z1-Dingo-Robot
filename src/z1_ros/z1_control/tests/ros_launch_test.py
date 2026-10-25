#!/usr/bin/env python3
"""Verify arm.launch against loopback-only MCU protocol fixtures.

Run after sourcing a ROS 1 workspace containing the new Z1 packages:
  python3 ros_launch_test.py /path/to/z1_bringup/launch/arm.launch
Requires ROS 1 robot_state_publisher, xacro, rospy, roslaunch and tf2_msgs.
The production UDP driver is used; no physical robot or dynamics simulator.
"""
import os
from pathlib import Path
import socket
import sys
import tempfile
import time
import yaml
from mcu_fixture import McuFixture, unused_local_port


def main():
    if len(sys.argv) != 2:
        raise RuntimeError(__doc__)
    launch_file = str(Path(sys.argv[1]).resolve())
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = f'http://127.0.0.1:{port}'
    os.environ['ROS_IP'] = '127.0.0.1'
    from rosmaster.master import Master
    import roslaunch
    import rospy
    from tf2_msgs.msg import TFMessage
    from z1_msgs.msg import Status

    master = Master(port=port)
    master.start()
    launcher = None
    firmware = []
    configuration = tempfile.TemporaryDirectory(prefix='z1-launch-test-')
    try:
        rospy.init_node('z1_launch_smoke', disable_signals=True)
        states, transforms = {}, set()
        subscriptions = []
        for side in ('left', 'right'):
            subscriptions.append(rospy.Subscriber(
                f'/{side}_arm/status', Status,
                lambda message, key=side: states.__setitem__(key, message)))

        def received_tf(message):
            transforms.update((t.header.frame_id, t.child_frame_id)
                              for t in message.transforms)

        subscriptions.append(rospy.Subscriber('/tf', TFMessage, received_tf))
        subscriptions.append(rospy.Subscriber('/tf_static', TFMessage, received_tf))
        firmware.append(McuFixture('127.0.0.1'))
        firmware.append(McuFixture('127.0.0.2'))
        left_port, right_port = unused_local_port(), unused_local_port()
        while right_port == left_port:
            right_port = unused_local_port()
        communication = yaml.safe_load(
            (Path(launch_file).parent.parent / 'config/communication.yaml').read_text())
        for side, address, local_port in (('left', '127.0.0.1', left_port),
                                           ('right', '127.0.0.2', right_port)):
            communication[f'{side}_arm']['controller'].update(
                mcu_ip=address, local_port=local_port)
        config_file = Path(configuration.name) / 'communication.yaml'
        config_file.write_text(yaml.safe_dump(communication))
        run_id = roslaunch.rlutil.get_or_generate_uuid(None, False)
        launcher = roslaunch.parent.ROSLaunchParent(
            run_id, [(launch_file, [f'config:={config_file}'])])
        launcher.start()
        deadline = time.monotonic() + 12
        required = {(f'{side}_link00', f'{side}_link01')
                    for side in ('left', 'right')}
        while time.monotonic() < deadline:
            if len(states) == 2 and required <= transforms:
                break
            launcher.spin_once()
            time.sleep(0.02)
        assert len(states) == 2, 'dual launch did not publish both status topics'
        assert all(s.mode == Status.DISABLED and s.feedback_valid
                   and len(s.model_revision) == 64 for s in states.values()), states
        assert required <= transforms, f'missing prefixed transforms: {transforms}'
        for side in ('left', 'right'):
            description = rospy.get_param(f'/{side}_arm/robot_description')
            assert f'name="{side}_joint1"' in description
            assert f'{side}_link00' in description
        assert all(a.split('_')[0] == b.split('_')[0] for a, b in transforms), transforms
        assert all(peer.commands > 0 and peer.error is None for peer in firmware)
        print('PASS: MCU-only dual launch over loopback, independent namespaces, descriptions and prefixed TF')
    finally:
        if launcher is not None:
            launcher.shutdown()
        for peer in firmware:
            peer.close()
        rospy.signal_shutdown('test complete')
        master.stop()
        configuration.cleanup()


if __name__ == '__main__':
    main()
