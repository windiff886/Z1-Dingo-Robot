#!/usr/bin/env python3
"""ROS2 joystick bridge for the pan-tilt gimbal."""

from sensor_msgs.msg import Joy
import rclpy
from rclpy.node import Node

from pan_tilt_msgs.msg import PanTiltCmdDeg


class PanTiltControlNode(Node):
    def __init__(self):
        super().__init__('pan_tilt_control_node')

        self.declare_parameter('delta_deg', 2.0)
        self.declare_parameter('max_angle_deg', 60.0)
        self.declare_parameter('speed_deg_per_sec', 20.0)
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('command_topic', 'pan_tilt_cmd_deg')

        self.delta_deg = self.get_parameter('delta_deg').value
        self.max_angle_deg = self.get_parameter('max_angle_deg').value
        self.speed_deg_per_sec = self.get_parameter('speed_deg_per_sec').value
        joy_topic = self.get_parameter('joy_topic').value
        command_topic = self.get_parameter('command_topic').value

        self.pan_tilt_yaw = 0.0
        self.pan_tilt_pitch = 0.0

        self.publisher = self.create_publisher(PanTiltCmdDeg, command_topic, 10)
        self.create_subscription(Joy, joy_topic, self.joy_callback, 10)

        self.get_logger().info('PanTilt Control Start')

    def joy_callback(self, data: Joy):
        if data.buttons[0] == 1:
            self.pan_tilt_pitch -= self.delta_deg
        if data.buttons[1] == 1:
            self.pan_tilt_yaw -= self.delta_deg
        if data.buttons[2] == 1:
            self.pan_tilt_pitch += self.delta_deg
        if data.buttons[3] == 1:
            self.pan_tilt_yaw += self.delta_deg
        if data.buttons[5] == 1:
            self.pan_tilt_pitch = 0.0
            self.pan_tilt_yaw = 0.0

        self.pan_tilt_pitch = max(-self.max_angle_deg, min(self.max_angle_deg, self.pan_tilt_pitch))
        self.pan_tilt_yaw = max(-self.max_angle_deg, min(self.max_angle_deg, self.pan_tilt_yaw))

        command = PanTiltCmdDeg()
        command.speed = float(self.speed_deg_per_sec)
        command.yaw = float(self.pan_tilt_yaw)
        command.pitch = float(self.pan_tilt_pitch)
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
