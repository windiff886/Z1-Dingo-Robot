#!/usr/bin/env python3
"""RViz end-pose sender. All motion stays in the existing Z1 controller."""
import copy
import math
import threading
import time

import rospy
import tf2_geometry_msgs  # Registers PoseStamped transformations with tf2.
import tf2_ros
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback,
    Marker, MarkerArray,
)
from z1_msgs.msg import ControlCommand, MotionResult, PoseGoal, Status

MODE_NAMES = ('Disabled', 'Holding', 'JointGoal', 'PoseGoal', 'JointStream',
              'PoseStream', 'Impedance', 'Stopping', 'AwaitResume', 'Fault',
              'JointTrajectory', 'CartesianPath')

# Inactive components stay at zero for display. Arm positions use feedback only;
# gripper motor feedback has no calibrated conversion to jointGripper.
STATIC_DISPLAY_JOINTS = (
    'front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel',
    'pan_tilt_yaw_joint', 'pan_tilt_pitch_joint',
    'left_jointGripper', 'right_jointGripper',
)


def valid_pose(pose):
    q = pose.orientation
    values = (pose.position.x, pose.position.y, pose.position.z, q.x, q.y, q.z, q.w)
    return (all(math.isfinite(v) for v in values)
            and abs(math.sqrt(sum(v * v for v in values[3:])) - 1.0) <= 0.001)


class Arm:
    def __init__(self, side):
        self.side = side
        self.base = side + '_link00'
        self.name = side + '_tcp_target'
        self.collision_check_enabled = rospy.get_param(
            '/' + side + '_arm/controller/collision_check_enabled', True)
        self.status = self.tcp = self.target = None
        self.status_at = self.tcp_at = 0.0
        self.created = self.dirty = False
        self.pending = None
        self.note = 'Waiting for valid controller feedback'
        self.goal_pub = rospy.Publisher('/' + side + '_arm/pose_goal', PoseGoal, queue_size=1)
        self.control_pub = rospy.Publisher('/' + side + '_arm/control', ControlCommand, queue_size=4)


class TcpTest:
    def __init__(self):
        self.lock = threading.RLock()
        self.frame = 'base_link'
        self.send_on_release = rospy.get_param('~send_on_release')
        self.speed = float(rospy.get_param('~speed_scale'))
        self.timeout = float(rospy.get_param('~telemetry_timeout'))
        if not (0 < self.speed <= 1 and math.isfinite(self.timeout) and self.timeout > 0):
            raise ValueError('speed_scale must be in (0,1]; telemetry_timeout must be positive')
        self.command_id = time.time_ns()
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.server = InteractiveMarkerServer('~targets')
        self.joints = rospy.Publisher('~joint_states', JointState, queue_size=10)
        self.labels = rospy.Publisher('~status_markers', MarkerArray, queue_size=1)
        self.arms = {side: Arm(side) for side in ('left', 'right')}
        self.subscriptions = []
        for side, arm in self.arms.items():
            for topic, msg_type, callback in (
                    ('status', Status, self.on_status), ('tcp_pose', PoseStamped, self.on_tcp),
                    ('result', MotionResult, self.on_result), ('joint_states', JointState, self.on_joints)):
                self.subscriptions.append(rospy.Subscriber(
                    '/' + side + '_arm/' + topic, msg_type,
                    lambda msg, a=arm, cb=callback: cb(a, msg), queue_size=10))
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tick)
        rospy.loginfo('TCP test: drag a TCP handle and release to send=%s; speed_scale=%.2f',
                      self.send_on_release, self.speed)

    def next_id(self):
        self.command_id = max(self.command_id + 1, time.time_ns())
        return self.command_id

    def fresh(self, arm):
        now = time.monotonic()
        if arm.status is None or arm.tcp is None or not arm.status.feedback_valid:
            return False
        for msg, received in ((arm.status, arm.status_at), (arm.tcp, arm.tcp_at)):
            age = (rospy.Time.now() - msg.header.stamp).to_sec()
            if msg.header.stamp.is_zero() or not (-0.02 <= age <= self.timeout):
                return False
            if now - received > self.timeout:
                return False
        return True

    def on_status(self, arm, msg):
        with self.lock:
            if msg.header.frame_id != arm.base:
                return
            arm.status, arm.status_at = msg, time.monotonic()
            self.command_id = max(self.command_id, msg.active_command_id)

    def on_tcp(self, arm, msg):
        with self.lock:
            if msg.header.frame_id == arm.base and valid_pose(msg.pose):
                arm.tcp, arm.tcp_at = msg, time.monotonic()

    def on_joints(self, arm, msg):
        # The adapter can republish retained q while feedback_valid=false.
        # Forward only valid telemetry, preserving the adapter's ROS stamp.
        with self.lock:
            if not self.fresh(arm):
                return
            expected = [arm.side + '_joint' + str(i) for i in range(1, 7)]
            if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
                return
            positions = dict(zip(msg.name, msg.position))
            if not all(n in positions and math.isfinite(positions[n]) for n in expected):
                return
            out = JointState(header=msg.header, name=expected,
                             position=[positions[n] for n in expected])
            self.joints.publish(out)

    def on_result(self, arm, msg):
        with self.lock:
            self.command_id = max(self.command_id, msg.command_id)
            arm.note = '{}: {}'.format(msg.outcome, msg.reason)
            if msg.command_id == arm.pending and msg.outcome != MotionResult.ACCEPTED:
                arm.pending = None
            rospy.loginfo('%s result id=%s outcome=%s: %s', arm.side,
                          msg.command_id, msg.outcome, msg.reason)

    def transform(self, pose, frame):
        # Latest TF is intentional: both arm bases are fixed to base_link.
        # Zero is for TF lookup only; outgoing commands receive a fresh stamp.
        source = copy.deepcopy(pose)
        source.header.stamp = rospy.Time(0)
        return self.buffer.transform(source, frame, rospy.Duration(0.05))

    def reset(self, arm):
        if not self.fresh(arm):
            raise ValueError('fresh status and TCP feedback required')
        arm.target = self.transform(arm.tcp, self.frame)
        arm.dirty = False
        if arm.created:
            self.server.setPose(arm.name, arm.target.pose, arm.target.header)
            self.server.applyChanges()
        arm.note = 'Target reset to measured TCP'

    def send(self, arm):
        if not self.fresh(arm):
            raise ValueError('stale/invalid status or TCP feedback; target not sent')
        if arm.status.mode != Status.HOLDING or arm.pending is not None:
            raise ValueError('requires Holding and no pending goal; target not queued')
        if arm.target is None or not valid_pose(arm.target.pose):
            raise ValueError('invalid target pose')
        if arm.goal_pub.get_num_connections() == 0:
            raise ValueError('no pose_goal subscriber')
        target = self.transform(arm.target, arm.base)
        msg = PoseGoal()
        msg.header = Header(stamp=rospy.Time.now(), frame_id=arm.base)
        msg.command_id, msg.pose, msg.speed_scale = self.next_id(), target.pose, self.speed
        arm.pending = msg.command_id
        arm.goal_pub.publish(msg)
        arm.note = 'Sent id={} in {}'.format(msg.command_id, arm.base)
        rospy.loginfo('%s %s: xyz=(%.4f, %.4f, %.4f)', arm.side, arm.note,
                      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def operate(self, arm, operation):
        # STOP/DISABLE remain available even when telemetry is absent.
        if operation not in (ControlCommand.STOP, ControlCommand.DISABLE) and not self.fresh(arm):
            raise ValueError('fresh status and TCP feedback required for this operation')
        if arm.control_pub.get_num_connections() == 0:
            raise ValueError('no control subscriber')
        msg = ControlCommand()
        msg.header.stamp, msg.command_id, msg.operation = rospy.Time.now(), self.next_id(), operation
        arm.control_pub.publish(msg)
        arm.note = 'Control operation {} requested'.format(operation)

    def handle(self, arm, action):
        with self.lock:
            try:
                action()
            except (ValueError, tf2_ros.TransformException) as error:
                arm.note = str(error)
                rospy.logwarn('%s: %s', arm.side, error)

    def feedback(self, arm, msg):
        def action():
            if msg.event_type in (InteractiveMarkerFeedback.POSE_UPDATE, InteractiveMarkerFeedback.MOUSE_UP):
                if msg.header.frame_id != self.frame or not valid_pose(msg.pose):
                    raise ValueError('invalid marker frame or pose')
                arm.target = PoseStamped(header=copy.deepcopy(msg.header), pose=copy.deepcopy(msg.pose))
                if msg.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
                    arm.dirty = True
                elif arm.dirty:
                    arm.dirty = False
                    if self.send_on_release:
                        self.send(arm)
        self.handle(arm, action)

    def create_marker(self, arm):
        self.reset(arm)
        marker = InteractiveMarker()
        marker.header.frame_id, marker.name = self.frame, arm.name
        trigger = 'release to send' if self.send_on_release else 'menu: Send target once'
        marker.description = arm.side + ' TCP target (' + trigger + ')'
        marker.pose, marker.scale = arm.target.pose, 0.22
        center = InteractiveMarkerControl(name='menu', always_visible=True)
        center.orientation.w = 1.0
        center.interaction_mode = InteractiveMarkerControl.MENU
        sphere = Marker(type=Marker.SPHERE)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.045
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (0.2, 0.8, 1.0, 0.9)
        center.markers.append(sphere)
        marker.controls.append(center)
        for axis, xyz in (('x', (1, 0, 0)), ('y', (0, 0, 1)), ('z', (0, 1, 0))):
            for label, mode in (('rotate', InteractiveMarkerControl.ROTATE_AXIS),
                                ('move', InteractiveMarkerControl.MOVE_AXIS)):
                control = InteractiveMarkerControl(name=label + '_' + axis, interaction_mode=mode)
                control.orientation.w = math.sqrt(0.5)
                control.orientation.x, control.orientation.y, control.orientation.z = [v * math.sqrt(0.5) for v in xyz]
                control.orientation_mode = InteractiveMarkerControl.FIXED
                marker.controls.append(control)
        self.server.insert(marker, lambda msg: self.feedback(arm, msg))
        menu = MenuHandler()
        for label, action in (
                ('Send target once', lambda: self.send(arm)),
                ('Reset target to measured TCP', lambda: self.reset(arm)),
                ('ENABLE (Disabled -> Holding)', lambda: self.operate(arm, ControlCommand.ENABLE)),
                ('STOP', lambda: self.operate(arm, ControlCommand.STOP)),
                ('HOLD after stop (AwaitResume -> Holding)', lambda: self.operate(arm, ControlCommand.HOLD)),
                ('Acknowledge FAULT (then ENABLE)', lambda: self.operate(arm, ControlCommand.RESUME)),
                ('DISABLE (passive output)', lambda: self.operate(arm, ControlCommand.DISABLE))):
            menu.insert(label, callback=lambda msg, a=action: self.handle(arm, a))
        menu.apply(self.server, arm.name)
        arm.menu, arm.created = menu, True
        self.server.applyChanges()

    def tick(self, _event):
        with self.lock:
            self.joints.publish(JointState(header=Header(stamp=rospy.Time.now()),
                                           name=list(STATIC_DISPLAY_JOINTS),
                                           position=[0.0] * len(STATIC_DISPLAY_JOINTS)))
            labels = MarkerArray()
            for index, arm in enumerate(self.arms.values()):
                if not arm.created and self.fresh(arm):
                    self.handle(arm, lambda a=arm: self.create_marker(a))
                label = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, id=index, ns='controller_status')
                label.header.frame_id, label.header.stamp = self.frame, rospy.Time.now()
                label.pose.orientation.w, label.pose.position.z = 1.0, 1.45 + index * 0.1
                label.scale.z = 0.045
                ready = self.fresh(arm)
                holding = ready and arm.status.mode == Status.HOLDING
                label.color.r = 0.2 if holding else 1.0
                label.color.g, label.color.a = (1.0 if ready else 0.2), 1.0
                code = arm.status.mode if arm.status is not None else -1
                mode = MODE_NAMES[code] if 0 <= code < len(MODE_NAMES) else '?'
                reason = arm.status.reason if arm.status is not None else 'no controller'
                label.text = ('{}: {} {} | {}'.format(arm.side, mode, reason, arm.note)
                              if ready else arm.side + ': FEEDBACK STALE / INVALID')
                if not arm.collision_check_enabled:
                    label.text += ' | GEOMETRIC CHECKS OFF'
                labels.markers.append(label)
            self.labels.publish(labels)


if __name__ == '__main__':
    rospy.init_node('rviz_tcp_test')
    node = TcpTest()
    rospy.spin()
