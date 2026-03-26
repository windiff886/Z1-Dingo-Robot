#!/usr/bin/env python3
"""BaseAPI - Unified interface for mobile base control (ROS2)."""

import math
import threading
from typing import Optional

from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
import tf2_ros

from .action_future import ActionFuture


class BaseAPI:
    """Unified API for mobile base control."""

    def __init__(self, node: Node, mode: str = 'simulation', config: dict = None):
        self._node = node
        self._mode = mode
        self._config = config or {}
        self._enabled = True
        self._emergency_stopped = False

        self._cmd_vel_topic = self._config.get('cmd_vel_topic', '/cmd_vel')
        self._odom_topic = self._config.get('odom_topic', '/odometry/filtered')
        self._max_linear_vel = self._config.get('max_linear_vel', 1.0)
        self._max_angular_vel = self._config.get('max_angular_vel', 2.0)

        self._cmd_vel_pub = node.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._current_odom = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        self._odom_sub = node.create_subscription(
            Odometry, self._odom_topic, self._odom_callback, 10)

        node.get_logger().info(f'BaseAPI initialized in {mode} mode')

    def _odom_callback(self, msg: Odometry):
        self._current_odom = msg

    def move(self, linear_x: float = 0.0, linear_y: float = 0.0,
             angular_z: float = 0.0) -> None:
        if not self._enabled or self._emergency_stopped:
            self._node.get_logger().warn('BaseAPI: Cannot move - disabled or emergency stopped')
            return

        twist = Twist()
        twist.linear.x = self._clamp(linear_x, -self._max_linear_vel, self._max_linear_vel)
        twist.linear.y = self._clamp(linear_y, -self._max_linear_vel, self._max_linear_vel)
        twist.angular.z = self._clamp(angular_z, -self._max_angular_vel, self._max_angular_vel)
        self._cmd_vel_pub.publish(twist)

    def stop(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def move_to(self, x: float, y: float, theta: float,
                linear_tolerance: float = 0.05,
                angular_tolerance: float = 0.1) -> ActionFuture:
        if not self._enabled or self._emergency_stopped:
            return ActionFuture.failure('Base disabled or emergency stopped')

        future = ActionFuture()

        def control_loop():
            while not future._cancelled:
                current = self.get_pose_2d()
                if current is None:
                    threading.Event().wait(0.05)
                    continue

                dx = x - current.x
                dy = y - current.y
                dist = math.sqrt(dx * dx + dy * dy)
                target_angle = math.atan2(dy, dx)
                angle_diff = self._normalize_angle(target_angle - current.theta)
                final_angle_diff = self._normalize_angle(theta - current.theta)

                if dist > linear_tolerance:
                    if abs(angle_diff) > 0.3:
                        self.move(angular_z=1.5 * angle_diff)
                    else:
                        speed = min(0.5, dist)
                        self.move(linear_x=speed, angular_z=0.5 * angle_diff)
                elif abs(final_angle_diff) > angular_tolerance:
                    self.move(angular_z=1.0 * final_angle_diff)
                else:
                    self.stop()
                    future._complete(True)
                    return

                threading.Event().wait(0.05)

            self.stop()
            future._complete(False, error_msg='Cancelled')

        threading.Thread(target=control_loop, daemon=True).start()
        return future

    def get_odom(self) -> Optional[Odometry]:
        return self._current_odom

    def get_pose_2d(self) -> Optional[Pose2D]:
        if self._current_odom is None:
            return None

        pose = Pose2D()
        pose.x = self._current_odom.pose.pose.position.x
        pose.y = self._current_odom.pose.pose.position.y

        q = self._current_odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        pose.theta = math.atan2(siny_cosp, cosy_cosp)
        return pose

    def get_velocity(self) -> Optional[Twist]:
        if self._current_odom is None:
            return None
        return self._current_odom.twist.twist

    def is_moving(self) -> bool:
        vel = self.get_velocity()
        if vel is None:
            return False
        threshold = 0.01
        return (abs(vel.linear.x) > threshold or
                abs(vel.linear.y) > threshold or
                abs(vel.angular.z) > threshold)

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled
        if not enabled:
            self.stop()

    def is_enabled(self) -> bool:
        return self._enabled

    def emergency_stop(self) -> None:
        self._emergency_stopped = True
        self.stop()

    def recover(self) -> bool:
        self._emergency_stopped = False
        return True

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
