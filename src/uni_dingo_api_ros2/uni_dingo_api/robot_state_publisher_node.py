#!/usr/bin/env python3
"""Robot state publisher node - publishes unified RobotState at 10 Hz."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from uni_dingo_msgs.msg import (
    RobotState,
    BaseState,
    ArmState,
    HeadState,
    GripperState,
    SubsystemStatus,
)


class RobotStatePublisherNode(Node):
    def __init__(self):
        super().__init__('uni_dingo_state_publisher')

        self.declare_parameter('mode', 'simulation')
        self.declare_parameter('publish_rate', 10.0)
        self._mode = self.get_parameter('mode').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self._publish_rate = publish_rate if publish_rate > 0.0 else 10.0

        self._state_pub = self.create_publisher(RobotState, '/uni_dingo/state', 10)

        self._joint_state = None
        self._odom = None

        self._left_arm_joints = [f'left_joint{i}' for i in range(1, 7)]
        self._right_arm_joints = [f'right_joint{i}' for i in range(1, 7)]
        self._head_joints = ['pan_tilt_yaw_joint', 'pan_tilt_pitch_joint']

        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)

        self.create_timer(1.0 / self._publish_rate, self._publish_state)
        self.get_logger().info(
            f'Robot state publisher started (mode: {self._mode}, rate: {self._publish_rate:.1f} Hz)'
        )

    def _joint_state_cb(self, msg):
        self._joint_state = msg

    def _odom_cb(self, msg):
        self._odom = msg

    def _publish_state(self):
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.mode = self._mode
        msg.last_update = msg.header.stamp

        msg.base = self._build_base_state()
        msg.left_arm = self._build_arm_state('left')
        msg.right_arm = self._build_arm_state('right')
        msg.head = self._build_head_state()
        msg.subsystem_statuses = self._build_subsystem_statuses()
        msg.all_ready = all(status.ready for status in msg.subsystem_statuses)
        msg.emergency_stopped = msg.base.emergency_stopped

        self._state_pub.publish(msg)

    def _build_base_state(self) -> BaseState:
        base = BaseState()
        base.enabled = True
        base.emergency_stopped = False

        if self._odom:
            base.header.stamp = self._odom.header.stamp
            base.pose = self._odom.pose.pose
            base.velocity = self._odom.twist.twist
            vel = self._odom.twist.twist
            base.moving = (
                abs(vel.linear.x) > 0.01 or
                abs(vel.linear.y) > 0.01 or
                abs(vel.angular.z) > 0.01
            )

        return base

    def _build_arm_state(self, side: str) -> ArmState:
        arm = ArmState()
        arm.side = side
        arm.enabled = True
        arm.in_collision = False
        arm.at_limit = False
        arm.current_named_pose = ''

        joint_names = self._left_arm_joints if side == 'left' else self._right_arm_joints
        arm.joint_names = joint_names
        arm.gripper = self._build_gripper_state(side)

        if self._joint_state:
            arm.header.stamp = self._joint_state.header.stamp
            positions = []
            velocities = []
            efforts = []

            for joint_name in joint_names:
                if joint_name in self._joint_state.name:
                    idx = self._joint_state.name.index(joint_name)
                    positions.append(self._joint_state.position[idx])
                    velocities.append(
                        self._joint_state.velocity[idx]
                        if idx < len(self._joint_state.velocity) else 0.0
                    )
                    efforts.append(
                        self._joint_state.effort[idx]
                        if idx < len(self._joint_state.effort) else 0.0
                    )

            arm.joint_positions = positions
            arm.joint_velocities = velocities
            arm.joint_efforts = efforts
            arm.moving = any(abs(velocity) > 0.01 for velocity in velocities)

        return arm

    def _build_gripper_state(self, side: str) -> GripperState:
        gripper = GripperState()
        gripper_joint = f'{side}_gripper'

        if self._joint_state and gripper_joint in self._joint_state.name:
            idx = self._joint_state.name.index(gripper_joint)
            gripper.position = self._joint_state.position[idx]
            gripper.velocity = (
                self._joint_state.velocity[idx]
                if idx < len(self._joint_state.velocity) else 0.0
            )
            gripper.effort = (
                self._joint_state.effort[idx]
                if idx < len(self._joint_state.effort) else 0.0
            )

        return gripper

    def _build_head_state(self) -> HeadState:
        head = HeadState()
        head.enabled = True
        head.pan_min = -1.047
        head.pan_max = 1.047
        head.tilt_min = -1.047
        head.tilt_max = 1.047

        if self._joint_state:
            head.header.stamp = self._joint_state.header.stamp
            if self._head_joints[0] in self._joint_state.name:
                idx = self._joint_state.name.index(self._head_joints[0])
                head.pan = self._joint_state.position[idx]
                head.pan_velocity = (
                    self._joint_state.velocity[idx]
                    if idx < len(self._joint_state.velocity) else 0.0
                )
            if self._head_joints[1] in self._joint_state.name:
                idx = self._joint_state.name.index(self._head_joints[1])
                head.tilt = self._joint_state.position[idx]
                head.tilt_velocity = (
                    self._joint_state.velocity[idx]
                    if idx < len(self._joint_state.velocity) else 0.0
                )

            head.moving = (
                abs(head.pan_velocity) > 0.01 or
                abs(head.tilt_velocity) > 0.01
            )

        return head

    def _build_subsystem_statuses(self):
        statuses = []

        statuses.append(self._make_status(
            name='base',
            enabled=True,
            ready=self._odom is not None,
            ok_message='OK',
            waiting_message='Waiting for odometry',
        ))
        statuses.append(self._make_status(
            name='left_arm',
            enabled=True,
            ready=self._joint_state is not None,
            ok_message='OK',
            waiting_message='Waiting for joint states',
        ))
        statuses.append(self._make_status(
            name='right_arm',
            enabled=True,
            ready=self._joint_state is not None,
            ok_message='OK',
            waiting_message='Waiting for joint states',
        ))
        statuses.append(self._make_status(
            name='head',
            enabled=True,
            ready=True,
            ok_message='OK',
            waiting_message='Waiting for head state',
        ))
        statuses.append(self._make_status(
            name='perception',
            enabled=True,
            ready=True,
            ok_message='OK',
            waiting_message='Waiting for perception data',
        ))

        return statuses

    @staticmethod
    def _make_status(name: str, enabled: bool, ready: bool,
                     ok_message: str, waiting_message: str) -> SubsystemStatus:
        status = SubsystemStatus()
        status.name = name
        status.enabled = enabled
        status.ready = ready
        status.status = SubsystemStatus.STATUS_OK
        status.message = ok_message if ready else waiting_message
        return status


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
