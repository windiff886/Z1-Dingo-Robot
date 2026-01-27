#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot State Publisher - Publishes unified robot state
"""

import rospy
from uni_dingo_msgs.msg import RobotState, BaseState, ArmState, SubsystemStatus


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

        rospy.loginfo(f"RobotStatePublisher started (mode: {self._mode})")

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

        # Subsystem statuses
        state.subsystem_statuses = []

        # Base status
        base_status = SubsystemStatus()
        base_status.name = 'base'
        base_status.status = SubsystemStatus.STATUS_OK
        base_status.enabled = True
        base_status.ready = True
        state.subsystem_statuses.append(base_status)

        # Left arm status
        left_arm_status = SubsystemStatus()
        left_arm_status.name = 'left_arm'
        left_arm_status.status = SubsystemStatus.STATUS_OK
        left_arm_status.enabled = True
        left_arm_status.ready = True
        state.subsystem_statuses.append(left_arm_status)

        # Right arm status
        right_arm_status = SubsystemStatus()
        right_arm_status.name = 'right_arm'
        right_arm_status.status = SubsystemStatus.STATUS_OK
        right_arm_status.enabled = True
        right_arm_status.ready = True
        state.subsystem_statuses.append(right_arm_status)

        # Check if all ready
        state.all_ready = all(s.ready for s in state.subsystem_statuses)
        state.emergency_stopped = False

        return state


if __name__ == '__main__':
    try:
        node = RobotStatePublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
