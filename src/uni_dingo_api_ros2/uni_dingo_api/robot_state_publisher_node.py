#!/usr/bin/env python3
"""Robot state publisher node - publishes unified RobotState at 10 Hz."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from uni_dingo_msgs.msg import RobotState, BaseState, ArmState, HeadState, GripperState


class RobotStatePublisherNode(Node):
    def __init__(self):
        super().__init__('uni_dingo_state_publisher')

        self._state_pub = self.create_publisher(RobotState, '/uni_dingo/state', 10)

        self._joint_state = None
        self._odom = None

        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)

        self.create_timer(0.1, self._publish_state)
        self.get_logger().info('Robot state publisher started')

    def _joint_state_cb(self, msg):
        self._joint_state = msg

    def _odom_cb(self, msg):
        self._odom = msg

    def _publish_state(self):
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self._odom:
            msg.base.velocity = self._odom.twist.twist
            msg.base.pose = self._odom.pose.pose

        if self._joint_state:
            for side in ['left', 'right']:
                arm_state = ArmState()
                arm_state.joint_names = [f'{side}_joint{i}' for i in range(1, 7)]
                positions = []
                for name in arm_state.joint_names:
                    if name in self._joint_state.name:
                        idx = self._joint_state.name.index(name)
                        positions.append(self._joint_state.position[idx])
                    else:
                        positions.append(0.0)
                arm_state.joint_positions = positions

                if side == 'left':
                    msg.left_arm = arm_state
                else:
                    msg.right_arm = arm_state

        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
