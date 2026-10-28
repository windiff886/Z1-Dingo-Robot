#!/usr/bin/env python3
"""ROS 1 integration test using the production MCU transport over loopback.

Usage: ros_topics_test.py /path/to/z1_topic_node /path/to/z1_control.urdf
Requires generated z1_msgs and normal ROS 1 Python/runtime dependencies.
Uses two local protocol fixtures, no physical robot or dynamics simulator.
Only loopback addresses are passed to the production node.
"""
import os
import sys
import time
import signal
import socket
import subprocess
import math
import copy
from pathlib import Path
import yaml
from mcu_fixture import McuFixture, unused_local_port


def main():
    if len(sys.argv) != 3:
        raise RuntimeError(__doc__)
    executable, model = (str(Path(p).resolve()) for p in sys.argv[1:])
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        port = s.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = f'http://127.0.0.1:{port}'
    os.environ['ROS_IP'] = '127.0.0.1'
    from rosmaster.master import Master
    import rospy
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseStamped
    from z1_msgs.msg import (ControlCommand, JointGoal, JointStream, PoseStream,
                             JointReference, CartesianReference,
                             JointTrajectory, TrajectoryPoint, JointWaypoints,
                             JointWaypoint, LinearPath,
                             ImpedanceGains, MotionResult, Status)
    master = Master(port=port)
    master.start()
    children = []
    logs = []
    firmware = []
    try:
        rospy.init_node('z1_offline_integration', disable_signals=True)
        for address in ('127.0.0.1', '127.0.0.2'):
            firmware.append(McuFixture(address, has_gripper=address == '127.0.0.1',
                                       gripper_q=1.1803553206846118e-5))
        config_dir = Path(__file__).resolve().parents[2] / 'z1_bringup/config'
        communication = yaml.safe_load((config_dir / 'communication.yaml').read_text())
        control = yaml.safe_load((config_dir / 'control.yaml').read_text())
        states, joints, poses, commands, results = {}, {}, {}, {}, []
        subscribers = []
        local_ports = set()
        for index, arm in enumerate(('left', 'right'), start=1):
            address = f'127.0.0.{index}'
            local_port = unused_local_port()
            while local_port in local_ports:
                local_port = unused_local_port()
            local_ports.add(local_port)
            ns = f'/{arm}_arm'
            for topic, cls, store in [('status', Status, states), ('joint_states', JointState, joints), ('tcp_pose', PoseStamped, poses), ('commanded_joint_states', JointState, commands)]:
                subscribers.append(rospy.Subscriber(ns + '/' + topic, cls, lambda m, key=arm, target=store: target.__setitem__(key, m)))
            subscribers.append(rospy.Subscriber(ns + '/result', MotionResult, lambda m: results.append(m)))
            log = open(f'/tmp/z1-{arm}-integration.log', 'w')
            logs.append(log)
            parameters = dict(control)
            parameters.update(communication[f'{arm}_arm']['controller'])
            parameters.update(mcu_ip=address, local_port=local_port,
                              model_file=model, joint_prefix=f'{arm}_')
            rospy.set_param(ns + '/controller', parameters)
            children.append(subprocess.Popen(
                [executable, f'__ns:={ns}', '__name:=controller'],
                stdout=log, stderr=log))

        def wait(predicate, why, timeout=8):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if predicate():
                    return
                for arm, state in list(states.items()):
                    if state.mode == Status.FAULT:
                        raise RuntimeError(f'{arm} fault: {state.reason}')
                if any(p.poll() is not None for p in children):
                    raise RuntimeError('MCU child exited; inspect /tmp/z1-*-integration.log')
                if any(peer.error is not None for peer in firmware):
                    raise RuntimeError(f'MCU fixture failed: {[peer.error for peer in firmware]}')
                time.sleep(0.01)
            raise RuntimeError(why)

        wait(lambda: len(states) == 2 and all(s.feedback_valid for s in states.values()) and len(joints) == 2, 'no independent feedback')
        assert all(s.mode == Status.DISABLED for s in states.values()), 'startup enabled motion'
        assert all(n.startswith('left_') for n in joints['left'].name)
        assert all(n.startswith('right_') for n in joints['right'].name)
        assert states['left'].has_gripper and not states['right'].has_gripper
        assert joints['left'].position[6] > 0, 'raw gripper encoder allowance not exercised'
        wait(lambda: len(commands) == 2, 'missing host reference telemetry')
        assert commands['left'].name == [f'left_joint{i}' for i in range(1, 7)]
        wait(lambda: all(s.runtime_cycles > 0 for s in states.values()), 'no runtime timing statistics')
        first = states['left'].feedback_sequence
        began = time.monotonic()
        time.sleep(0.5)
        observed_rate = (states['left'].feedback_sequence - first) / (time.monotonic() - began)
        assert observed_rate > 0, 'loopback MCU feedback stopped'
        print(f'observed host loopback receive rate: {observed_rate:.1f} Hz; not a hardware rate measurement')
        control = rospy.Publisher('/left_arm/control', ControlCommand, queue_size=1)
        goal_pub = rospy.Publisher('/left_arm/joint_goal', JointGoal, queue_size=1)
        stream_pub = rospy.Publisher('/left_arm/joint_stream', JointStream, queue_size=1)
        pose_pub = rospy.Publisher('/left_arm/pose_stream', PoseStream, queue_size=1)
        gain_pub = rospy.Publisher('/left_arm/impedance_gains', ImpedanceGains, queue_size=1)
        joint_ref_pub = rospy.Publisher('/left_arm/joint_reference', JointReference, queue_size=32)
        pose_ref_pub = rospy.Publisher('/left_arm/pose_reference', CartesianReference, queue_size=32)
        trajectory_pub = rospy.Publisher('/left_arm/joint_trajectory', JointTrajectory, queue_size=1)
        waypoints_pub = rospy.Publisher('/left_arm/joint_waypoints', JointWaypoints, queue_size=1)
        linear_pub = rospy.Publisher('/left_arm/linear_path', LinearPath, queue_size=1)
        wait(lambda: all(p.get_num_connections() for p in (control, goal_pub, stream_pub, pose_pub, gain_pub, joint_ref_pub, pose_ref_pub, trajectory_pub, waypoints_pub, linear_pub)), 'subscriptions not connected')

        def operate(op):
            m = ControlCommand();m.header.stamp = rospy.Time.now();m.operation = op;control.publish(m)

        operate(ControlCommand.ENABLE)
        wait(lambda: states['left'].mode == Status.HOLDING, 'enable failed')
        goal = JointGoal();goal.command_id = 1;goal.speed_scale = 0.2
        goal.positions = list(joints['left'].position[:6]);goal.positions[0] += 0.03
        goal.header.stamp = rospy.Time.now();goal_pub.publish(goal)
        wait(lambda: any(e.command_id == 1 and e.outcome == MotionResult.SUCCEEDED for e in results), 'finite target did not succeed')
        goal.command_id = 2;goal.positions[0] = float('nan');goal_pub.publish(goal)
        wait(lambda: any(e.command_id == 2 and e.outcome == MotionResult.REJECTED for e in results), 'invalid target not rejected')

        def succeeded(command_id):
            wait(lambda: any(e.command_id == command_id and e.outcome in (MotionResult.SUCCEEDED, MotionResult.REJECTED, MotionResult.FAILED) for e in results), f'command {command_id} did not finish')
            events = [e for e in results if e.command_id == command_id]
            assert any(e.outcome == MotionResult.SUCCEEDED for e in events), [(e.outcome, e.reason) for e in events]

        # Timed trajectories execute once on the native clock, without a stream
        # session or periodic ROS target updates.
        trajectory = JointTrajectory();trajectory.command_id = 10
        endpoint = TrajectoryPoint();endpoint.time_from_start = rospy.Duration(0.8)
        endpoint.positions = list(joints['left'].position[:6]);endpoint.positions[0] += 0.01
        endpoint.has_velocity = endpoint.has_acceleration = True
        trajectory.points = [endpoint]
        trajectory.header.stamp = rospy.Time.now();trajectory_pub.publish(trajectory)
        wait(lambda: states['left'].mode == Status.JOINT_TRAJECTORY, 'timed trajectory mode not entered')
        assert abs(states['left'].trajectory_duration - 0.8) < 1e-6
        assert abs(states['left'].trajectory_time_scale - 1) < 1e-6
        succeeded(10)
        assert abs(joints['left'].position[0] - endpoint.positions[0]) < 0.005
        assert abs(commands['left'].position[0] - endpoint.positions[0]) < 1e-6
        assert all(math.isfinite(v) for v in commands['left'].effort)

        # An intentionally short trajectory is uniformly retimed before it is
        # installed; the adapter exposes the resulting duration and scale.
        trajectory.command_id = 11;trajectory.retime = True
        endpoint.time_from_start = rospy.Duration(0.02)
        endpoint.positions = list(joints['left'].position[:6]);endpoint.positions[0] += 0.02
        trajectory.points = [endpoint]
        trajectory.header.stamp = rospy.Time.now();trajectory_pub.publish(trajectory)
        wait(lambda: states['left'].mode == Status.JOINT_TRAJECTORY and states['left'].active_command_id == 11, 'retimed trajectory mode not entered')
        assert states['left'].trajectory_duration > 0.02
        assert states['left'].trajectory_time_scale > 1
        succeeded(11)

        waypoints = JointWaypoints();waypoints.command_id = 12;waypoints.speed_scale = 0.3
        start = list(joints['left'].position[:6])
        for offset in (0.008, 0.016):
            waypoint = JointWaypoint();waypoint.positions = start[:];waypoint.positions[0] += offset
            waypoints.waypoints.append(waypoint)
        waypoints.header.stamp = rospy.Time.now();waypoints_pub.publish(waypoints)
        wait(lambda: states['left'].mode == Status.JOINT_TRAJECTORY and states['left'].active_command_id == 12, 'continuous joint waypoints not entered')
        succeeded(12)
        assert abs(joints['left'].position[0] - waypoints.waypoints[-1].positions[0]) < 0.005

        linear = LinearPath();linear.command_id = 13;linear.speed_scale = 0.1
        target = copy.deepcopy(poses['left'].pose);target.position.x += 0.003
        linear.poses = [target];linear.header.frame_id = 'left_link00'
        linear.header.stamp = rospy.Time.now();linear_pub.publish(linear)
        wait(lambda: states['left'].mode == Status.CARTESIAN_PATH, 'MoveL mode not entered')
        succeeded(13)
        position = poses['left'].pose.position
        assert math.sqrt((position.x-target.position.x)**2 + (position.y-target.position.y)**2 + (position.z-target.position.z)**2) < 0.002

        for operation, mode, publisher, cls in (
            (ControlCommand.JOINT_STREAM, Status.JOINT_STREAM, stream_pub, JointStream),
            (ControlCommand.POSE_STREAM, Status.POSE_STREAM, pose_pub, PoseStream),
            (ControlCommand.IMPEDANCE, Status.IMPEDANCE, pose_pub, PoseStream),
            (ControlCommand.JOINT_STREAM, Status.JOINT_STREAM, joint_ref_pub, JointReference),
            (ControlCommand.POSE_STREAM, Status.POSE_STREAM, pose_ref_pub, CartesianReference),
            (ControlCommand.IMPEDANCE, Status.IMPEDANCE, pose_ref_pub, CartesianReference),
        ):
            if mode == Status.IMPEDANCE:
                gains = ImpedanceGains();gains.command_id = 3
                gains.stiffness = [30,30,30,0.2,0.2,0.2];gains.damping = [4,4,4,0.02,0.02,0.02]
                gain_pub.publish(gains)
                wait(lambda: any(e.command_id == 3 and e.outcome == MotionResult.ACCEPTED for e in results), 'impedance gains rejected')
            operate(operation)
            wait(lambda: states['left'].mode == mode, 'stream mode not entered')
            session = states['left'].session_id
            initial = list(joints['left'].position[:6])
            started = rospy.Time.now()
            for seq in range(1, 16):
                m = cls();m.session_id = session;m.sequence = seq;m.header.stamp = rospy.Time.now()
                if cls is JointStream:
                    m.positions = list(joints['left'].position[:6])
                elif cls is JointReference:
                    m.target_time = m.header.stamp + rospy.Duration(0.04)
                    phase = math.pi * (m.target_time - started).to_sec()
                    m.positions = initial[:];m.positions[0] += 0.01 * math.sin(phase)
                    m.has_velocity = m.has_acceleration = True
                    m.velocities[0] = 0.01 * math.pi * math.cos(phase)
                    m.accelerations[0] = -0.01 * math.pi**2 * math.sin(phase)
                else:
                    m.header.frame_id = 'left_link00';m.pose = poses['left'].pose
                    if cls is CartesianReference:
                        m.target_time = m.header.stamp + rospy.Duration(0.04)
                        m.has_velocity = m.has_acceleration = True
                publisher.publish(m)
                if seq == 8 and cls is CartesianReference and mode == Status.IMPEDANCE:
                    gains.command_id = 4;gains.stiffness = [40,40,40,0.3,0.3,0.3]
                    gain_pub.publish(gains)
                time.sleep(0.04)
            assert states['left'].mode == mode, 'fresh stream failed'
            if cls is JointReference:
                assert abs(joints['left'].position[0] - initial[0]) > 0.003, 'timed joint reference did not move the arm'
            if cls is CartesianReference and mode == Status.IMPEDANCE:
                wait(lambda: any(e.command_id == 4 and e.outcome == MotionResult.ACCEPTED for e in results), 'online impedance gains rejected')
            wait(lambda: states['left'].mode == Status.AWAIT_RESUME, 'stream timeout did not latch')
            # Delayed ordinary input cannot release the timeout latch.
            m.sequence = 100;m.header.stamp = rospy.Time.now()
            if cls in (JointReference, CartesianReference):m.target_time = m.header.stamp
            publisher.publish(m);time.sleep(0.06)
            assert states['left'].mode == Status.AWAIT_RESUME
            operate(ControlCommand.RESUME)
            wait(lambda: states['left'].mode == mode, 'explicit resume failed')
            new_session = states['left'].session_id
            assert new_session != session
            for seq in range(1, 8):
                m.session_id = new_session;m.sequence = seq;m.header.stamp = rospy.Time.now()
                if cls in (JointReference, CartesianReference):m.target_time = rospy.Time(0)
                publisher.publish(m);time.sleep(0.04)
            assert states['left'].mode == mode, 'new session not accepted'
            wait(lambda: states['left'].mode == Status.AWAIT_RESUME, 'second timeout failed')
            operate(ControlCommand.HOLD)
            wait(lambda: states['left'].mode == Status.HOLDING, 'stop-to-hold transition failed')

        before = states['right'].feedback_sequence
        children[0].send_signal(signal.SIGINT)
        children[0].wait(timeout=5)
        # The right process remains independent of left process lifecycle.
        deadline = time.monotonic() + 2
        while states['right'].feedback_sequence <= before and time.monotonic() < deadline:
            time.sleep(0.02)
        assert children[1].poll() is None and states['right'].feedback_sequence > before
        assert all(peer.commands > 0 for peer in firmware), 'MCU command path not exercised'
        print('PASS: MCU-only startup guards, real UDP codec, gripper encoder noise and bounded commands, ROS 1 topics, timed/retimed trajectories, continuous joint waypoints, MoveL, legacy and timed derivative streams, online impedance gains, timeout/resume and two-process isolation')
    finally:
        for p in children:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:p.kill();p.wait()
        for log in logs:log.close()
        for peer in firmware:peer.close()
        rospy.signal_shutdown('test complete')
        master.stop()


if __name__ == '__main__':
    main()
