#!/usr/bin/env python3
"""Adapt MoveIt's goal editor to SDK previews and explicit PoseGoal commands."""
import copy
import math
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import rospy
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from geometry_msgs.msg import PoseStamped
from unidingo_rviz_test.msg import EditorCommand, EditorTarget, PreviewStatus
from unidingo_rviz_test.srv import PreviewPose, PreviewPoseRequest
from z1_msgs.msg import ControlCommand, MotionResult, PoseGoal, Status

MODE_NAMES = ('Disabled', 'Holding', 'JointGoal', 'PoseGoal', 'JointStream',
              'PoseStream', 'Impedance', 'Stopping', 'AwaitResume', 'Fault',
              'JointTrajectory', 'CartesianPath')
STATIC_DISPLAY_JOINTS = (
    'front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel',
    'pan_tilt_yaw_joint', 'pan_tilt_pitch_joint', 'left_jointGripper', 'right_jointGripper')


def valid_pose(pose):
    q = pose.orientation
    values = (pose.position.x, pose.position.y, pose.position.z, q.x, q.y, q.z, q.w)
    return (all(math.isfinite(v) for v in values)
            and abs(math.sqrt(sum(v * v for v in values[3:])) - 1.0) <= .001)


class Arm:
    def __init__(self, side):
        self.side, self.base = side, side + '_link00'
        self.names = [side + '_joint' + str(i) for i in range(1, 7)]
        self.status = self.tcp = self.target = None
        self.status_at = self.tcp_at = self.positions_at = self.reference_at = 0.0
        self.positions = self.reference = None
        self.following = True
        self.version = self.epoch = self.sent_id = 0
        self.editor_revision = self.sent_editor_revision = 0
        self.requested = False
        self.future = self.preview = None
        self.note = 'Waiting for controller feedback'
        self.goal_pub = rospy.Publisher('/' + side + '_arm/pose_goal', PoseGoal, queue_size=1)
        self.control_pub = rospy.Publisher('/' + side + '_arm/control', ControlCommand, queue_size=4)
        self.preview_client = rospy.ServiceProxy('/rviz_tcp_test/' + side + '_preview/preview', PreviewPose)


class TcpTest:
    def __init__(self):
        self.lock = threading.RLock()
        self.speed = float(rospy.get_param('~speed_scale'))
        self.timeout = float(rospy.get_param('~telemetry_timeout'))
        if not (0 < self.speed <= 1 and math.isfinite(self.timeout) and self.timeout > 0):
            raise ValueError('speed_scale must be in (0,1]; telemetry_timeout must be positive')
        self.command_id = time.time_ns()
        self.selected = 'left'
        self.executor = ThreadPoolExecutor(max_workers=2)
        rospy.on_shutdown(lambda: self.executor.shutdown(wait=False))
        self.joints = rospy.Publisher('~joint_states', JointState, queue_size=10)
        self.paths = rospy.Publisher('~preview_trajectory', DisplayTrajectory, queue_size=1, latch=True)
        self.editor = rospy.Publisher('~editor_action', EditorCommand, queue_size=10)
        self.ui_status = rospy.Publisher('~ui_status', PreviewStatus, queue_size=10)
        self.arms = {side: Arm(side) for side in ('left', 'right')}
        self.subscriptions = [rospy.Subscriber('~ui_action', String, self.on_action, queue_size=10)]
        for side, arm in self.arms.items():
            for topic, msg_type, callback in (
                    ('status', Status, self.on_status), ('tcp_pose', PoseStamped, self.on_tcp),
                    ('result', MotionResult, self.on_result), ('joint_states', JointState, self.on_joints),
                    ('commanded_joint_states', JointState, self.on_reference)):
                self.subscriptions.append(rospy.Subscriber('/' + side + '_arm/' + topic, msg_type,
                    lambda msg, a=arm, cb=callback: cb(a, msg), queue_size=10))
            self.subscriptions.append(rospy.Subscriber('~' + side + '/editor_target', EditorTarget,
                lambda msg, a=arm: self.on_target(a, msg), queue_size=1))
        self.timer = rospy.Timer(rospy.Duration(.05), self.tick)

    def editor_action(self, value, expected_revision=0):
        arm, separator, operation = value.partition('/')
        self.editor.publish(EditorCommand(arm=arm if separator else '',
            operation=operation if separator else arm, expected_revision=expected_revision))

    def next_id(self):
        self.command_id = max(self.command_id + 1, time.time_ns())
        return self.command_id

    def current_stamp(self, header):
        age = (rospy.Time.now() - header.stamp).to_sec()
        return not header.stamp.is_zero() and -.02 <= age <= self.timeout

    def fresh(self, arm):
        if arm.status is None or arm.tcp is None or not arm.status.feedback_valid:
            return False
        return all(self.current_stamp(msg.header) and time.monotonic() - received <= self.timeout
                   for msg, received in ((arm.status, arm.status_at), (arm.tcp, arm.tcp_at)))

    def on_status(self, arm, msg):
        with self.lock:
            if msg.header.frame_id == arm.base:
                arm.status, arm.status_at = msg, time.monotonic()
                self.command_id = max(self.command_id, msg.active_command_id)

    def on_tcp(self, arm, msg):
        with self.lock:
            if msg.header.frame_id == arm.base and valid_pose(msg.pose):
                arm.tcp, arm.tcp_at = msg, time.monotonic()

    def joint_positions(self, arm, msg):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return None
        values = dict(zip(msg.name, msg.position))
        if all(n in values and math.isfinite(values[n]) for n in arm.names):
            return [values[n] for n in arm.names]
        return None

    def joint_receipt(self, msg):
        # Receipt does not renew the lifetime of a delayed source sample.
        return time.monotonic() - max(0., (rospy.Time.now() - msg.header.stamp).to_sec())

    def on_joints(self, arm, msg):
        with self.lock:
            values = self.joint_positions(arm, msg)
            if self.fresh(arm) and self.current_stamp(msg.header) and values is not None:
                arm.positions, arm.positions_at = values, self.joint_receipt(msg)
                self.joints.publish(JointState(header=msg.header, name=arm.names, position=values))

    def on_reference(self, arm, msg):
        with self.lock:
            values = self.joint_positions(arm, msg)
            if self.current_stamp(msg.header) and values is not None:
                arm.reference, arm.reference_at = values, self.joint_receipt(msg)

    def on_target(self, arm, msg):
        with self.lock:
            if msg.arm != arm.side or msg.header.frame_id != arm.base or not valid_pose(msg.pose):
                return
            arm.version += 1
            arm.editor_revision = msg.revision
            self.clear_preview(arm)
            arm.target = PoseStamped(header=msg.header, pose=copy.deepcopy(msg.pose)) if msg.valid else None
            arm.following = not msg.editing
            arm.requested = msg.editing and msg.valid
            arm.epoch += 1
            arm.note = ('Following measured joints' if arm.following else
                        'Preparing SDK reference' if msg.valid else 'MoveIt IK cannot reach this target')

    def start_positions(self, arm):
        if not self.fresh(arm):
            raise ValueError('Fresh controller feedback required for preview')
        values, received = ((arm.reference, arm.reference_at) if arm.status.mode == Status.HOLDING
                            else (arm.positions, arm.positions_at))
        if values is None or time.monotonic() - received > self.timeout:
            raise ValueError('Waiting for current joint/reference positions')
        return list(values)

    def show_path(self, arm):
        if arm.side != self.selected:
            return
        trajectory = arm.preview.trajectory
        state = RobotState()
        state.joint_state = JointState(name=list(STATIC_DISPLAY_JOINTS), position=[0.] * len(STATIC_DISPLAY_JOINTS))
        for other in self.arms.values():
            state.joint_state.name.extend(other.names)
            state.joint_state.position.extend(trajectory.points[0].positions if other is arm else
                                             other.positions if other.positions is not None else [0.] * 6)
        path = DisplayTrajectory()
        path.model_id = 'uni_dingo_dual_arm'
        path.trajectory_start = state
        path.trajectory = [RobotTrajectory(joint_trajectory=copy.deepcopy(trajectory))]
        self.paths.publish(path)

    def clear_preview(self, arm):
        arm.preview = None
        if arm.side == self.selected:
            # Upstream ignores empty DisplayTrajectory messages; use its reset API.
            self.editor_action('clear_path')
            self.paths.publish(DisplayTrajectory())

    def preview_matches_start(self, arm):
        return max(abs(a - b) for a, b in zip(self.start_positions(arm),
            arm.preview.trajectory.points[0].positions)) <= arm.preview.start_tolerance

    def ready_to_send(self, arm):
        return (not arm.sent_id and arm.preview is not None and self.fresh(arm)
                and arm.status.mode in (Status.DISABLED, Status.HOLDING)
                and arm.preview.model_revision == arm.status.model_revision)

    def send(self, arm):
        if not self.ready_to_send(arm):
            raise ValueError('Wait for SDK preview in Disabled/Holding and any pending command to finish')
        if not self.preview_matches_start(arm):
            self.clear_preview(arm)
            arm.version += 1
            arm.requested = True
            raise ValueError('Start position changed; refreshing SDK reference')
        if arm.goal_pub.get_num_connections() == 0:
            raise ValueError('No pose_goal subscriber')
        msg = PoseGoal()
        msg.header = Header(stamp=rospy.Time.now(), frame_id=arm.base)
        msg.command_id, msg.pose, msg.speed_scale = self.next_id(), copy.deepcopy(arm.target.pose), self.speed
        arm.goal_pub.publish(msg)
        arm.sent_id = msg.command_id
        arm.sent_editor_revision = arm.editor_revision
        arm.note = 'Sent pose id={}'.format(msg.command_id)

    def on_result(self, arm, msg):
        with self.lock:
            self.command_id = max(self.command_id, msg.command_id)
            if not arm.sent_id or msg.command_id != arm.sent_id:
                return
            arm.note = '{}: {}'.format(msg.outcome, msg.reason)
            if msg.outcome == MotionResult.ACCEPTED:
                return
            arm.sent_id = 0
            if msg.outcome == MotionResult.SUCCEEDED and arm.editor_revision == arm.sent_editor_revision:
                self.request_editor(arm, 'reset', arm.sent_editor_revision)
                arm.note = 'Reached target; restoring measured goal state'

    def operate(self, arm, operation):
        if arm.control_pub.get_num_connections() == 0:
            raise ValueError('No control subscriber')
        msg = ControlCommand()
        msg.header.stamp, msg.command_id, msg.operation = rospy.Time.now(), self.next_id(), operation
        arm.control_pub.publish(msg)
        arm.note = 'Control operation {} requested'.format(operation)

    def handle(self, arm, action):
        with self.lock:
            try:
                action()
            except (ValueError, rospy.ServiceException) as error:
                arm.note = str(error)
                rospy.logwarn('%s: %s', arm.side, error)

    def refresh_start(self, arm):
        if arm.preview is not None and self.fresh(arm) and arm.status.mode in (Status.DISABLED, Status.HOLDING):
            if not self.preview_matches_start(arm):
                self.clear_preview(arm)
                arm.version += 1
                arm.requested = True
                arm.note = 'Start position changed; refreshing SDK reference'

    def update_preview(self, arm):
        if arm.future is not None and arm.future[0].done():
            future, version, epoch = arm.future
            arm.future = None
            if epoch == arm.epoch and not arm.following:
                try:
                    response = future.result()
                    if not self.fresh(arm):
                        arm.requested = True
                        return
                    if not response.success:
                        raise ValueError(response.reason)
                    if response.model_revision != arm.status.model_revision:
                        raise ValueError('Preview model differs from controller; restart with matching configuration')
                    if version == arm.version:
                        arm.preview = response
                        self.refresh_start(arm)
                        if arm.preview is not None:
                            self.show_path(arm)
                            arm.note = 'Ready: {:.2f} s SDK reference'.format(response.trajectory.points[-1].time_from_start.to_sec())
                except (ValueError, rospy.ServiceException) as error:
                    self.clear_preview(arm)
                    arm.note = 'Preview: ' + str(error)
        if arm.requested and arm.future is None and self.fresh(arm):
            request = PreviewPoseRequest()
            request.header = Header(stamp=rospy.Time.now(), frame_id=arm.base)
            request.start_positions = self.start_positions(arm)
            request.target_pose = copy.deepcopy(arm.target.pose)
            request.speed_scale = self.speed
            arm.requested = False
            arm.future = (self.executor.submit(arm.preview_client, request), arm.version, arm.epoch)

    def request_editor(self, arm, operation, expected_revision=0):
        arm.version += 1
        arm.epoch += 1
        arm.requested = False
        self.clear_preview(arm)
        self.editor_action(arm.side + '/' + operation, expected_revision)
        arm.note = 'Waiting for MoveIt goal state'

    def on_action(self, msg):
        with self.lock:
            if msg.data == 'both/home':
                for arm in self.arms.values():
                    self.request_editor(arm, 'home')
                return
            side, _, operation = msg.data.partition('/')
            if side not in self.arms:
                rospy.logwarn('Unknown panel action: %s', msg.data)
                return
            arm = self.arms[side]
            if operation == 'select':
                self.selected = side
                # The editor acknowledges the selected group with a target.
                # Regenerate after that acknowledgement, not before its path reset.
                self.request_editor(arm, 'select')
            elif operation in ('home', 'reset'):
                self.request_editor(arm, operation)
            elif operation == 'send':
                self.handle(arm, lambda: self.send(arm))
            elif operation in ('stop', 'hold', 'enable', 'disable', 'resume'):
                code = getattr(ControlCommand, operation.upper())
                self.handle(arm, lambda: self.operate(arm, code))
            else:
                rospy.logwarn('Unknown panel action: %s', msg.data)

    def tick(self, _event):
        with self.lock:
            self.joints.publish(JointState(header=Header(stamp=rospy.Time.now()), name=list(STATIC_DISPLAY_JOINTS),
                                           position=[0.] * len(STATIC_DISPLAY_JOINTS)))
            for arm in self.arms.values():
                self.handle(arm, lambda a=arm: self.refresh_start(a))
                self.handle(arm, lambda a=arm: self.update_preview(a))
                if not self.fresh(arm) and arm.preview is not None:
                    self.clear_preview(arm)
                    arm.requested = not arm.following
                info = PreviewStatus()
                info.header = Header(stamp=rospy.Time.now(), frame_id=arm.base)
                info.arm = arm.side
                info.mode = MODE_NAMES[arm.status.mode] if arm.status is not None and 0 <= arm.status.mode < len(MODE_NAMES) else '?'
                info.detail = arm.note if self.fresh(arm) else 'Feedback stale or invalid'
                info.ready, info.editing = self.ready_to_send(arm), not arm.following
                info.target_available = arm.target is not None and self.fresh(arm)
                if info.target_available:
                    info.target_pose = arm.target.pose
                self.ui_status.publish(info)


if __name__ == '__main__':
    rospy.init_node('rviz_tcp_test')
    node = TcpTest()
    rospy.spin()
