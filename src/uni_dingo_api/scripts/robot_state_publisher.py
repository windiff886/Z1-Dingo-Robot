#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot State Publisher - Publishes unified robot state with detailed subsystem information
"""

import rospy
import math
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

from uni_dingo_msgs.msg import (
    RobotState, BaseState, ArmState, HeadState,
    GripperState, SubsystemStatus
)


class RobotStatePublisher:
    """Publishes unified robot state at regular intervals."""

    def __init__(self):
        rospy.init_node('uni_dingo_state_publisher', anonymous=True)

        self._rate = rospy.get_param('~publish_rate', 10.0)
        self._mode = rospy.get_param('/uni_dingo/robot/mode', 'simulation')

        # Publisher
        self._pub = rospy.Publisher(
            '/uni_dingo/state', RobotState, queue_size=10
        )

        # Data storage
        self._joint_states = None
        self._odom = None

        # Subscribers
        self._joint_sub = rospy.Subscriber(
            '/joint_states', JointState, self._joint_callback, queue_size=10
        )
        self._odom_sub = rospy.Subscriber(
            '/odometry/filtered', Odometry, self._odom_callback, queue_size=10
        )

        # Joint name mappings
        self._left_arm_joints = [
            'left_joint1', 'left_joint2', 'left_joint3',
            'left_joint4', 'left_joint5', 'left_joint6'
        ]
        self._right_arm_joints = [
            'right_joint1', 'right_joint2', 'right_joint3',
            'right_joint4', 'right_joint5', 'right_joint6'
        ]
        self._head_joints = ['pan_tilt_yaw_joint', 'pan_tilt_pitch_joint']

        rospy.loginfo(f"RobotStatePublisher started (mode: {self._mode})")

    def _joint_callback(self, msg: JointState):
        self._joint_states = msg

    def _odom_callback(self, msg: Odometry):
        self._odom = msg

    def run(self):
        """Main loop."""
        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            try:
                state = self._build_state()
                self._pub.publish(state)
            except Exception as e:
                rospy.logwarn_throttle(5.0, f"State publish error: {e}")

            rate.sleep()

    def _build_state(self) -> RobotState:
        """Build robot state message."""
        state = RobotState()
        state.header.stamp = rospy.Time.now()
        state.header.frame_id = 'base_link'
        state.mode = self._mode
        state.last_update = rospy.Time.now()

        # Build base state
        state.base = self._build_base_state()

        # Build arm states
        state.left_arm = self._build_arm_state('left')
        state.right_arm = self._build_arm_state('right')

        # Build head state
        state.head = self._build_head_state()

        # Build subsystem statuses
        state.subsystem_statuses = self._build_subsystem_statuses()

        # Check if all ready
        state.all_ready = all(s.ready for s in state.subsystem_statuses)
        state.emergency_stopped = False

        return state

    def _build_base_state(self) -> BaseState:
        """Build base state from odometry."""
        base = BaseState()
        # Use original odometry timestamp, not current time
        if self._odom:
            base.header.stamp = self._odom.header.stamp
        else:
            base.header.stamp = rospy.Time(0)
        base.enabled = True
        base.moving = False
        base.emergency_stopped = False

        if self._odom:
            base.pose = self._odom.pose.pose
            base.velocity = self._odom.twist.twist

            # Check if moving
            vel = self._odom.twist.twist
            base.moving = (abs(vel.linear.x) > 0.01 or
                          abs(vel.linear.y) > 0.01 or
                          abs(vel.angular.z) > 0.01)

        return base

    def _build_arm_state(self, side: str) -> ArmState:
        """Build arm state from joint states."""
        arm = ArmState()
        # Use original joint states timestamp, not current time
        if self._joint_states:
            arm.header.stamp = self._joint_states.header.stamp
        else:
            arm.header.stamp = rospy.Time(0)
        arm.side = side
        arm.enabled = True
        arm.moving = False
        arm.in_collision = False
        arm.at_limit = False

        if side == 'left':
            joint_names = self._left_arm_joints
        else:
            joint_names = self._right_arm_joints

        arm.joint_names = joint_names

        if self._joint_states:
            positions = []
            velocities = []
            efforts = []

            for name in joint_names:
                if name in self._joint_states.name:
                    idx = self._joint_states.name.index(name)
                    positions.append(self._joint_states.position[idx])
                    if idx < len(self._joint_states.velocity):
                        velocities.append(self._joint_states.velocity[idx])
                    else:
                        velocities.append(0.0)
                    if idx < len(self._joint_states.effort):
                        efforts.append(self._joint_states.effort[idx])
                    else:
                        efforts.append(0.0)

            arm.joint_positions = positions
            arm.joint_velocities = velocities
            arm.joint_efforts = efforts

            # Check if moving
            arm.moving = any(abs(v) > 0.01 for v in velocities)

        # Gripper state
        arm.gripper = GripperState()
        gripper_joint = f'{side}_gripper'
        if self._joint_states and gripper_joint in self._joint_states.name:
            idx = self._joint_states.name.index(gripper_joint)
            arm.gripper.position = self._joint_states.position[idx]

        return arm

    def _build_head_state(self) -> HeadState:
        """Build head state from joint states."""
        head = HeadState()
        # Use original joint states timestamp, not current time
        if self._joint_states:
            head.header.stamp = self._joint_states.header.stamp
        else:
            head.header.stamp = rospy.Time(0)
        head.enabled = True
        head.moving = False

        # Limits
        head.pan_min = -1.047
        head.pan_max = 1.047
        head.tilt_min = -1.047
        head.tilt_max = 1.047

        if self._joint_states:
            # Pan (yaw)
            if 'pan_tilt_yaw_joint' in self._joint_states.name:
                idx = self._joint_states.name.index('pan_tilt_yaw_joint')
                head.pan = self._joint_states.position[idx]
                if idx < len(self._joint_states.velocity):
                    head.pan_velocity = self._joint_states.velocity[idx]

            # Tilt (pitch)
            if 'pan_tilt_pitch_joint' in self._joint_states.name:
                idx = self._joint_states.name.index('pan_tilt_pitch_joint')
                head.tilt = self._joint_states.position[idx]
                if idx < len(self._joint_states.velocity):
                    head.tilt_velocity = self._joint_states.velocity[idx]

            # Check if moving
            head.moving = (abs(head.pan_velocity) > 0.01 or
                          abs(head.tilt_velocity) > 0.01)

        return head

    def _build_subsystem_statuses(self) -> list:
        """Build subsystem status list."""
        statuses = []

        # Base status
        base_status = SubsystemStatus()
        base_status.name = 'base'
        base_status.status = SubsystemStatus.STATUS_OK
        base_status.enabled = True
        base_status.ready = self._odom is not None
        base_status.message = "OK" if base_status.ready else "Waiting for odometry"
        statuses.append(base_status)

        # Left arm status
        left_status = SubsystemStatus()
        left_status.name = 'left_arm'
        left_status.status = SubsystemStatus.STATUS_OK
        left_status.enabled = True
        left_status.ready = self._joint_states is not None
        left_status.message = "OK" if left_status.ready else "Waiting for joint states"
        statuses.append(left_status)

        # Right arm status
        right_status = SubsystemStatus()
        right_status.name = 'right_arm'
        right_status.status = SubsystemStatus.STATUS_OK
        right_status.enabled = True
        right_status.ready = self._joint_states is not None
        right_status.message = "OK" if right_status.ready else "Waiting for joint states"
        statuses.append(right_status)

        # Head status
        head_status = SubsystemStatus()
        head_status.name = 'head'
        head_status.status = SubsystemStatus.STATUS_OK
        head_status.enabled = True
        head_status.ready = True
        head_status.message = "OK"
        statuses.append(head_status)

        # Perception status
        perception_status = SubsystemStatus()
        perception_status.name = 'perception'
        perception_status.status = SubsystemStatus.STATUS_OK
        perception_status.enabled = True
        perception_status.ready = True
        perception_status.message = "OK"
        statuses.append(perception_status)

        return statuses


if __name__ == '__main__':
    try:
        node = RobotStatePublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
