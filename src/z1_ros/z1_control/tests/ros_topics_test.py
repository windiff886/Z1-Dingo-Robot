#!/usr/bin/env python3
"""Offline ROS 1 integration test. Starts an isolated master and TWO SIMULATION processes.

Usage: ros_topics_test.py /path/to/z1_topic_node /path/to/z1_control.urdf
Requires generated z1_msgs and normal ROS 1 Python/runtime dependencies.
No UDP MCU driver, physical robot, global ROS master, or existing nodes are used.
"""
import os
import sys
import time
import signal
import socket
import subprocess
from pathlib import Path


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
                             ImpedanceGains, MotionResult, Status)
    master = Master(port=port)
    master.start()
    children = []
    logs = []
    try:
        rospy.init_node('z1_offline_integration', disable_signals=True)
        states, joints, poses, results = {}, {}, {}, []
        subscribers = []
        for arm in ('left', 'right'):
            ns = f'/{arm}_arm'
            for topic, cls, store in [('status', Status, states), ('joint_states', JointState, joints), ('tcp_pose', PoseStamped, poses)]:
                subscribers.append(rospy.Subscriber(ns + '/' + topic, cls, lambda m, key=arm, target=store: target.__setitem__(key, m)))
            subscribers.append(rospy.Subscriber(ns + '/result', MotionResult, lambda m: results.append(m)))
            log = open(f'/tmp/z1-{arm}-integration.log', 'w')
            logs.append(log)
            children.append(subprocess.Popen([executable, f'__ns:={ns}', '__name:=controller',
                '_backend:=simulation', f'_model_file:={model}', f'_joint_prefix:={arm}_'], stdout=log, stderr=log))

        def wait(predicate, why, timeout=8):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if predicate():
                    return
                for arm, state in list(states.items()):
                    if state.mode == Status.FAULT:
                        raise RuntimeError(f'{arm} fault: {state.reason}')
                if any(p.poll() is not None for p in children):
                    raise RuntimeError('simulation child exited; inspect /tmp/z1-*-integration.log')
                time.sleep(0.01)
            raise RuntimeError(why)

        wait(lambda: len(states) == 2 and all(s.feedback_valid for s in states.values()) and len(joints) == 2, 'no independent feedback')
        assert all(s.mode == Status.DISABLED for s in states.values()), 'startup enabled motion'
        assert all(n.startswith('left_') for n in joints['left'].name)
        assert all(n.startswith('right_') for n in joints['right'].name)
        control = rospy.Publisher('/left_arm/control', ControlCommand, queue_size=1)
        goal_pub = rospy.Publisher('/left_arm/joint_goal', JointGoal, queue_size=1)
        stream_pub = rospy.Publisher('/left_arm/joint_stream', JointStream, queue_size=1)
        pose_pub = rospy.Publisher('/left_arm/pose_stream', PoseStream, queue_size=1)
        gain_pub = rospy.Publisher('/left_arm/impedance_gains', ImpedanceGains, queue_size=1)
        wait(lambda: all(p.get_num_connections() for p in (control, goal_pub, stream_pub, pose_pub, gain_pub)), 'subscriptions not connected')

        def operate(op):
            m = ControlCommand();m.header.stamp = rospy.Time.now();m.operation = op;control.publish(m)

        operate(ControlCommand.ENABLE)
        wait(lambda: states['left'].mode == Status.HOLDING, 'enable failed')
        time.sleep(1.5)  # settle computed-torque/gravity ramp before a finite movement
        goal = JointGoal();goal.command_id = 1;goal.speed_scale = 0.2
        goal.positions = list(joints['left'].position[:6]);goal.positions[0] += 0.03
        goal.header.stamp = rospy.Time.now();goal_pub.publish(goal)
        wait(lambda: any(e.command_id == 1 and e.outcome == MotionResult.SUCCEEDED for e in results), 'finite target did not succeed')
        goal.command_id = 2;goal.positions[0] = float('nan');goal_pub.publish(goal)
        wait(lambda: any(e.command_id == 2 and e.outcome == MotionResult.REJECTED for e in results), 'invalid target not rejected')

        for operation, mode, publisher, cls in (
            (ControlCommand.JOINT_STREAM, Status.JOINT_STREAM, stream_pub, JointStream),
            (ControlCommand.POSE_STREAM, Status.POSE_STREAM, pose_pub, PoseStream),
            (ControlCommand.IMPEDANCE, Status.IMPEDANCE, pose_pub, PoseStream),
        ):
            if mode == Status.IMPEDANCE:
                gains = ImpedanceGains();gains.command_id = 3
                gains.stiffness = [30,30,30,0.2,0.2,0.2];gains.damping = [4,4,4,0.02,0.02,0.02]
                gain_pub.publish(gains)
                wait(lambda: any(e.command_id == 3 and e.outcome == MotionResult.ACCEPTED for e in results), 'impedance gains rejected')
            operate(operation)
            wait(lambda: states['left'].mode == mode, 'stream mode not entered')
            session = states['left'].session_id
            for seq in range(1, 16):
                m = cls();m.session_id = session;m.sequence = seq;m.header.stamp = rospy.Time.now()
                if cls is JointStream:
                    m.positions = list(joints['left'].position[:6])
                else:
                    m.header.frame_id = 'left_link00';m.pose = poses['left'].pose
                publisher.publish(m)
                time.sleep(0.04)
            assert states['left'].mode == mode, 'fresh stream failed'
            wait(lambda: states['left'].mode == Status.AWAIT_RESUME, 'stream timeout did not latch')
            # Delayed ordinary input cannot release the timeout latch.
            m.sequence = 100;m.header.stamp = rospy.Time.now();publisher.publish(m);time.sleep(0.06)
            assert states['left'].mode == Status.AWAIT_RESUME
            operate(ControlCommand.RESUME)
            wait(lambda: states['left'].mode == mode, 'explicit resume failed')
            new_session = states['left'].session_id
            assert new_session != session
            for seq in range(1, 8):
                m.session_id = new_session;m.sequence = seq;m.header.stamp = rospy.Time.now();publisher.publish(m);time.sleep(0.04)
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
        print('PASS: ROS 1 topics, finite goal, invalid input, both streams, impedance, timeout/resume and two-process isolation')
    finally:
        for p in children:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:p.kill();p.wait()
        for log in logs:log.close()
        rospy.signal_shutdown('test complete')
        master.stop()


if __name__ == '__main__':
    main()
