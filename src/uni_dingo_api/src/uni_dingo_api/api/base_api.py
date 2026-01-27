#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BaseAPI - Unified interface for mobile base control
"""

import rospy
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from nav_msgs.msg import Odometry
from typing import Optional, Callable
import tf2_ros
import tf2_geometry_msgs
import math

from .action_future import ActionFuture


class BaseAPI:
    """
    Unified API for mobile base control.

    Supports both simulation (Gazebo) and real robot (Dingo driver).
    The backend is selected based on configuration.

    Usage:
        base = BaseAPI(mode='simulation')
        base.move(linear_x=0.5)  # Move forward
        base.move_to(x=1.0, y=0.0, theta=0.0).wait()  # Go to pose
    """

    def __init__(self, mode: str = 'simulation', config: dict = None):
        """
        Initialize BaseAPI.

        Args:
            mode: 'simulation' or 'real'
            config: Configuration dictionary (from robot_config.yaml)
        """
        self._mode = mode
        self._config = config or {}
        self._enabled = True
        self._emergency_stopped = False

        # Get topics from config or use defaults
        self._cmd_vel_topic = self._config.get('cmd_vel_topic', '/cmd_vel')
        self._odom_topic = self._config.get('odom_topic', '/odometry/filtered')

        # Velocity limits
        self._max_linear_vel = self._config.get('max_linear_vel', 1.0)
        self._max_angular_vel = self._config.get('max_angular_vel', 2.0)

        # Publishers and subscribers
        self._cmd_vel_pub = rospy.Publisher(self._cmd_vel_topic, Twist, queue_size=10)
        self._odom_sub = None
        self._current_odom = None

        # TF buffer for transforms
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # Subscribe to odometry
        self._subscribe_odom()

        rospy.loginfo(f"BaseAPI initialized in {mode} mode")

    def _subscribe_odom(self):
        """Subscribe to odometry topic."""
        def odom_callback(msg):
            self._current_odom = msg

        self._odom_sub = rospy.Subscriber(
            self._odom_topic, Odometry, odom_callback, queue_size=10
        )

    # ==================== Motion Control ====================

    def move(self, linear_x: float = 0.0, linear_y: float = 0.0,
             angular_z: float = 0.0) -> None:
        """
        Send velocity command to the base.

        Args:
            linear_x: Forward/backward velocity (m/s)
            linear_y: Left/right velocity (m/s, omni only)
            angular_z: Rotational velocity (rad/s)
        """
        if not self._enabled or self._emergency_stopped:
            rospy.logwarn("BaseAPI: Cannot move - disabled or emergency stopped")
            return

        twist = Twist()
        twist.linear.x = self._clamp(linear_x, -self._max_linear_vel, self._max_linear_vel)
        twist.linear.y = self._clamp(linear_y, -self._max_linear_vel, self._max_linear_vel)
        twist.angular.z = self._clamp(angular_z, -self._max_angular_vel, self._max_angular_vel)

        self._cmd_vel_pub.publish(twist)

    def stop(self) -> None:
        """Stop the base immediately."""
        twist = Twist()
        self._cmd_vel_pub.publish(twist)

    def move_to(self, x: float, y: float, theta: float,
                frame: str = 'odom',
                linear_tolerance: float = 0.05,
                angular_tolerance: float = 0.1) -> ActionFuture:
        """
        Move to a target pose using simple proportional control.

        Args:
            x: Target x position
            y: Target y position
            theta: Target orientation (rad)
            frame: Reference frame
            linear_tolerance: Position tolerance (m)
            angular_tolerance: Orientation tolerance (rad)

        Returns:
            ActionFuture that completes when pose is reached
        """
        if not self._enabled or self._emergency_stopped:
            return ActionFuture.failure("Base disabled or emergency stopped")

        future = ActionFuture()
        future._success = False

        def control_loop():
            rate = rospy.Rate(20)
            while not rospy.is_shutdown() and not future._cancelled:
                current = self.get_pose_2d()
                if current is None:
                    rate.sleep()
                    continue

                # Calculate errors
                dx = x - current.x
                dy = y - current.y
                dist = math.sqrt(dx*dx + dy*dy)

                # Angle to target
                target_angle = math.atan2(dy, dx)
                angle_diff = self._normalize_angle(target_angle - current.theta)

                # Final orientation error
                final_angle_diff = self._normalize_angle(theta - current.theta)

                # State machine: rotate -> translate -> rotate
                if dist > linear_tolerance:
                    if abs(angle_diff) > 0.3:
                        # Rotate towards target
                        self.move(angular_z=1.5 * angle_diff)
                    else:
                        # Move towards target
                        speed = min(0.5, dist)
                        self.move(linear_x=speed, angular_z=0.5 * angle_diff)
                elif abs(final_angle_diff) > angular_tolerance:
                    # Final rotation
                    self.move(angular_z=1.0 * final_angle_diff)
                else:
                    # Done
                    self.stop()
                    future._success = True
                    future._done.set()
                    return

                rate.sleep()

            self.stop()
            future._done.set()

        import threading
        thread = threading.Thread(target=control_loop, daemon=True)
        thread.start()

        return future

    # ==================== State Query ====================

    def get_odom(self) -> Optional[Odometry]:
        """Get current odometry."""
        return self._current_odom

    def get_pose_2d(self) -> Optional[Pose2D]:
        """Get current pose as Pose2D."""
        if self._current_odom is None:
            return None

        pose = Pose2D()
        pose.x = self._current_odom.pose.pose.position.x
        pose.y = self._current_odom.pose.pose.position.y

        # Extract yaw from quaternion
        q = self._current_odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        pose.theta = math.atan2(siny_cosp, cosy_cosp)

        return pose

    def get_velocity(self) -> Optional[Twist]:
        """Get current velocity."""
        if self._current_odom is None:
            return None
        return self._current_odom.twist.twist

    def is_moving(self) -> bool:
        """Check if base is currently moving."""
        vel = self.get_velocity()
        if vel is None:
            return False
        threshold = 0.01
        return (abs(vel.linear.x) > threshold or
                abs(vel.linear.y) > threshold or
                abs(vel.angular.z) > threshold)

    # ==================== Enable/Disable ====================

    def set_enabled(self, enabled: bool) -> None:
        """Enable or disable the base."""
        self._enabled = enabled
        if not enabled:
            self.stop()

    def is_enabled(self) -> bool:
        """Check if base is enabled."""
        return self._enabled

    def emergency_stop(self) -> None:
        """Activate emergency stop."""
        self._emergency_stopped = True
        self.stop()

    def recover(self) -> bool:
        """Recover from emergency stop."""
        self._emergency_stopped = False
        return True

    # ==================== Utilities ====================

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        """Clamp value to range."""
        return max(min_val, min(max_val, value))

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
