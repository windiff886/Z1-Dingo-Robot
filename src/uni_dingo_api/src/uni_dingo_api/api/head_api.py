#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HeadAPI - Unified interface for pan-tilt head control
"""

import rospy
import math
import threading
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from typing import Optional, Tuple

from .action_future import ActionFuture

# Try to import pan_tilt_msgs (may not be available in all setups)
try:
    from pan_tilt_msgs.msg import PanTiltCmdDeg, PanTiltStatus
    HAS_PAN_TILT_MSGS = True
except ImportError:
    HAS_PAN_TILT_MSGS = False


class HeadAPI:
    """
    Unified API for pan-tilt head control.

    Supports both simulation (Gazebo joint controllers) and real robot (pan_tilt driver).

    Usage:
        head = HeadAPI(mode='simulation')
        head.set_angles(pan=0.5, tilt=-0.3).wait()  # radians
        head.look_at(point).wait()
        head.home().wait()
    """

    def __init__(self, mode: str = 'simulation', config: dict = None):
        """
        Initialize HeadAPI.

        Args:
            mode: 'simulation' or 'real'
            config: Configuration dictionary
        """
        self._mode = mode
        self._config = config or {}
        self._enabled = True

        # Joint names
        self._pan_joint = self._config.get('pan_joint', 'pan_tilt_yaw_joint')
        self._tilt_joint = self._config.get('tilt_joint', 'pan_tilt_pitch_joint')

        # Limits (in radians)
        self._pan_min = self._config.get('pan_min', -math.pi / 3)  # -60 deg
        self._pan_max = self._config.get('pan_max', math.pi / 3)   # +60 deg
        self._tilt_min = self._config.get('tilt_min', -math.pi / 3)
        self._tilt_max = self._config.get('tilt_max', math.pi / 3)

        # Current state
        self._current_pan = 0.0
        self._current_tilt = 0.0
        self._state_lock = threading.Lock()

        # Topics based on mode
        if mode == 'simulation':
            # Gazebo uses position controllers
            self._pan_pub = rospy.Publisher(
                '/pan_tilt/pan_controller/command', Float64, queue_size=10
            )
            self._tilt_pub = rospy.Publisher(
                '/pan_tilt/tilt_controller/command', Float64, queue_size=10
            )
            # Subscribe to joint states
            self._joint_state_sub = rospy.Subscriber(
                '/joint_states', JointState, self._joint_state_callback, queue_size=10
            )
        else:
            # Real robot uses pan_tilt_msgs
            if HAS_PAN_TILT_MSGS:
                self._cmd_pub = rospy.Publisher(
                    '/pan_tilt_cmd_deg', PanTiltCmdDeg, queue_size=10
                )
                self._status_sub = rospy.Subscriber(
                    '/pan_tilt_status', PanTiltStatus, self._status_callback, queue_size=10
                )
            else:
                rospy.logwarn("pan_tilt_msgs not available, HeadAPI in limited mode")
                self._cmd_pub = None

        # TF buffer for look_at functionality
        try:
            import tf2_ros
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        except Exception:
            self._tf_buffer = None

        rospy.loginfo(f"HeadAPI initialized in {mode} mode")

    def _joint_state_callback(self, msg: JointState):
        """Callback for joint state updates (simulation)."""
        with self._state_lock:
            if self._pan_joint in msg.name:
                idx = msg.name.index(self._pan_joint)
                self._current_pan = msg.position[idx]
            if self._tilt_joint in msg.name:
                idx = msg.name.index(self._tilt_joint)
                self._current_tilt = msg.position[idx]

    def _status_callback(self, msg):
        """Callback for pan_tilt status updates (real robot)."""
        with self._state_lock:
            # Convert from degrees to radians
            self._current_pan = math.radians(msg.yaw_now)
            self._current_tilt = math.radians(msg.pitch_now)

    # ==================== Motion Control ====================

    def set_angles(self, pan: float, tilt: float, speed: float = 0.5) -> ActionFuture:
        """
        Set pan and tilt angles.

        Args:
            pan: Pan angle in radians (yaw)
            tilt: Tilt angle in radians (pitch)
            speed: Movement speed (0.0 to 1.0, used for real robot)

        Returns:
            ActionFuture
        """
        if not self._enabled:
            return ActionFuture.failure("Head disabled")

        # Clamp to limits
        pan = self._clamp(pan, self._pan_min, self._pan_max)
        tilt = self._clamp(tilt, self._tilt_min, self._tilt_max)

        future = ActionFuture()

        if self._mode == 'simulation':
            # Publish to Gazebo controllers
            self._pan_pub.publish(Float64(pan))
            self._tilt_pub.publish(Float64(tilt))

            # Wait for motion to complete
            def wait_for_position():
                rate = rospy.Rate(20)
                timeout = rospy.Time.now() + rospy.Duration(5.0)
                tolerance = 0.05  # radians

                while rospy.Time.now() < timeout and not rospy.is_shutdown():
                    with self._state_lock:
                        pan_error = abs(self._current_pan - pan)
                        tilt_error = abs(self._current_tilt - tilt)

                    if pan_error < tolerance and tilt_error < tolerance:
                        future._success = True
                        future._done.set()
                        return

                    rate.sleep()

                # Timeout
                future._success = False
                future._error_msg = "Timeout waiting for head position"
                future._done.set()

            thread = threading.Thread(target=wait_for_position, daemon=True)
            thread.start()

        else:
            # Real robot
            if self._cmd_pub and HAS_PAN_TILT_MSGS:
                cmd = PanTiltCmdDeg()
                cmd.yaw = math.degrees(pan)
                cmd.pitch = math.degrees(tilt)
                cmd.speed = int(speed * 60)  # Convert to deg/s (max ~60)
                self._cmd_pub.publish(cmd)

                # Simple wait (real robot feedback is async)
                def wait_real():
                    rospy.sleep(2.0)  # Estimate motion time
                    future._success = True
                    future._done.set()

                thread = threading.Thread(target=wait_real, daemon=True)
                thread.start()
            else:
                return ActionFuture.failure("Pan-tilt driver not available")

        return future

    def look_at(self, point: PointStamped, frame: str = 'base_link') -> ActionFuture:
        """
        Point the head to look at a specific point in space.

        Args:
            point: Target point to look at
            frame: Reference frame for calculations

        Returns:
            ActionFuture
        """
        if not self._enabled:
            return ActionFuture.failure("Head disabled")

        if self._tf_buffer is None:
            return ActionFuture.failure("TF not available")

        try:
            # Transform point to head base frame
            head_frame = 'pan_tilt_base_link'

            # Get transform from point frame to head frame
            transform = self._tf_buffer.lookup_transform(
                head_frame, point.header.frame_id,
                rospy.Time(0), rospy.Duration(1.0)
            )

            # Apply transform to point
            import tf2_geometry_msgs
            point_transformed = tf2_geometry_msgs.do_transform_point(point, transform)

            # Calculate angles
            x = point_transformed.point.x
            y = point_transformed.point.y
            z = point_transformed.point.z

            # Pan angle (yaw) - rotation around Z axis
            pan = math.atan2(y, x)

            # Tilt angle (pitch) - rotation around Y axis
            horizontal_dist = math.sqrt(x*x + y*y)
            tilt = -math.atan2(z, horizontal_dist)  # Negative because looking down is positive pitch

            return self.set_angles(pan, tilt)

        except Exception as e:
            return ActionFuture.failure(f"look_at failed: {e}")

    def home(self) -> ActionFuture:
        """
        Move head to home position (centered).

        Returns:
            ActionFuture
        """
        return self.set_angles(0.0, 0.0)

    def stop(self) -> None:
        """Stop head motion (sets to current position)."""
        with self._state_lock:
            pan = self._current_pan
            tilt = self._current_tilt

        if self._mode == 'simulation':
            self._pan_pub.publish(Float64(pan))
            self._tilt_pub.publish(Float64(tilt))

    # ==================== State Query ====================

    def get_angles(self) -> Tuple[float, float]:
        """
        Get current pan and tilt angles.

        Returns:
            (pan, tilt) in radians
        """
        with self._state_lock:
            return (self._current_pan, self._current_tilt)

    def get_angles_deg(self) -> Tuple[float, float]:
        """
        Get current pan and tilt angles in degrees.

        Returns:
            (pan, tilt) in degrees
        """
        pan, tilt = self.get_angles()
        return (math.degrees(pan), math.degrees(tilt))

    def get_limits(self) -> dict:
        """
        Get joint limits.

        Returns:
            Dictionary with pan_min, pan_max, tilt_min, tilt_max
        """
        return {
            'pan_min': self._pan_min,
            'pan_max': self._pan_max,
            'tilt_min': self._tilt_min,
            'tilt_max': self._tilt_max
        }

    def is_moving(self) -> bool:
        """Check if head is currently moving."""
        # Simple implementation - could be improved with velocity check
        return False

    # ==================== Enable/Disable ====================

    def set_enabled(self, enabled: bool) -> None:
        """Enable or disable the head."""
        self._enabled = enabled

    def is_enabled(self) -> bool:
        """Check if head is enabled."""
        return self._enabled

    # ==================== Utilities ====================

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        """Clamp value to range."""
        return max(min_val, min(max_val, value))
