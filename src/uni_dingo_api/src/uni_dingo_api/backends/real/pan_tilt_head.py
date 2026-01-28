#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PanTiltHeadBackend - Real pan-tilt head backend for HeadAPI
"""

import rospy
import math
import threading
from typing import Optional, Tuple, Dict

from ..api.head_api import HeadAPI
from ..api.action_future import ActionFuture

# Import pan_tilt_msgs
try:
    from pan_tilt_msgs.msg import PanTiltCmdDeg, PanTiltStatus
    HAS_PAN_TILT_MSGS = True
except ImportError:
    HAS_PAN_TILT_MSGS = False
    rospy.logwarn("pan_tilt_msgs not available")


class PanTiltHeadBackend(HeadAPI):
    """
    Real pan-tilt head backend.

    Interfaces with the PanTiltDriverNode through ROS topics.
    Provides hardware monitoring, temperature feedback, and error handling.
    """

    def __init__(self, config: dict = None):
        super().__init__(mode='real', config=config)

        # Hardware state
        self._hardware_ready = False
        self._last_status = None
        self._status_lock = threading.Lock()

        # Temperature monitoring
        self._yaw_temp = 0.0
        self._pitch_temp = 0.0
        self._temp_warning_threshold = 60.0  # Celsius

        # Error state
        self._driver_error = 0
        self._encoder_error = 0
        self._loop_error = 0

        # Device info
        self._serial_num = ""
        self._hw_version = ""
        self._sw_version = ""

        # Override topics from config
        self._cmd_topic = config.get('cmd_topic', '/pan_tilt_cmd_deg') if config else '/pan_tilt_cmd_deg'
        self._status_topic = config.get('status_topic', '/pan_tilt_status') if config else '/pan_tilt_status'
        self._port = config.get('port', '/dev/pan_tilt') if config else '/dev/pan_tilt'

        # Set up subscribers and publishers
        if HAS_PAN_TILT_MSGS:
            self._cmd_pub = rospy.Publisher(
                self._cmd_topic, PanTiltCmdDeg, queue_size=10
            )
            self._status_sub = rospy.Subscriber(
                self._status_topic, PanTiltStatus, self._status_callback_real, queue_size=10
            )
        else:
            self._cmd_pub = None
            self._status_sub = None
            rospy.logerr("PanTiltHeadBackend: pan_tilt_msgs not available!")

        rospy.loginfo("PanTiltHeadBackend initialized")

    def _status_callback_real(self, msg: 'PanTiltStatus'):
        """Callback for pan_tilt status updates."""
        with self._status_lock:
            self._last_status = msg
            self._hardware_ready = True

            # Update current angles (convert degrees to radians)
            self._current_pan = math.radians(msg.yaw_now)
            self._current_tilt = math.radians(msg.pitch_now)

            # Update temperatures
            self._yaw_temp = msg.yaw_temp
            self._pitch_temp = msg.pitch_temp

            # Update error codes
            self._driver_error = msg.driver_ec
            self._encoder_error = msg.encoder_ec
            self._loop_error = msg.loop_ec

            # Update device info (only on first message)
            if not self._serial_num:
                self._serial_num = msg.serial_num
                self._hw_version = msg.hw_version
                self._sw_version = msg.sw_version

            # Temperature warning
            if self._yaw_temp > self._temp_warning_threshold or \
               self._pitch_temp > self._temp_warning_threshold:
                rospy.logwarn_throttle(10.0,
                    f"Pan-tilt temperature warning: yaw={self._yaw_temp}°C, pitch={self._pitch_temp}°C")

    # ==================== Hardware Status ====================

    def is_hardware_ready(self) -> bool:
        """Check if pan-tilt hardware is connected and responding."""
        with self._status_lock:
            return self._hardware_ready

    def get_temperatures(self) -> Dict[str, float]:
        """
        Get motor temperatures.

        Returns:
            Dict with 'yaw' and 'pitch' temperatures in Celsius
        """
        with self._status_lock:
            return {
                'yaw': self._yaw_temp,
                'pitch': self._pitch_temp
            }

    def is_overheating(self, threshold: float = None) -> bool:
        """
        Check if any motor is overheating.

        Args:
            threshold: Temperature threshold in Celsius (default: 60°C)

        Returns:
            True if any motor exceeds threshold
        """
        threshold = threshold or self._temp_warning_threshold
        temps = self.get_temperatures()
        return temps['yaw'] > threshold or temps['pitch'] > threshold

    def has_errors(self) -> bool:
        """Check if any errors are present."""
        with self._status_lock:
            return (self._driver_error != 0 or
                    self._encoder_error != 0 or
                    self._loop_error != 0)

    def get_errors(self) -> Dict[str, int]:
        """
        Get error codes.

        Returns:
            Dict with 'driver', 'encoder', 'loop' error codes
        """
        with self._status_lock:
            return {
                'driver': self._driver_error,
                'encoder': self._encoder_error,
                'loop': self._loop_error
            }

    def get_device_info(self) -> Dict[str, str]:
        """
        Get device information.

        Returns:
            Dict with 'serial', 'hw_version', 'sw_version'
        """
        with self._status_lock:
            return {
                'serial': self._serial_num,
                'hw_version': self._hw_version,
                'sw_version': self._sw_version,
                'port': self._port
            }

    # ==================== Motion Control ====================

    def set_angles(self, pan: float, tilt: float, speed: float = 0.5) -> ActionFuture:
        """
        Set pan and tilt angles.

        Args:
            pan: Pan angle in radians (yaw)
            tilt: Tilt angle in radians (pitch)
            speed: Movement speed (0.0 to 1.0)

        Returns:
            ActionFuture
        """
        if not self._enabled:
            return ActionFuture.failure("Head disabled")

        if not HAS_PAN_TILT_MSGS or self._cmd_pub is None:
            return ActionFuture.failure("Pan-tilt driver not available")

        if not self.is_hardware_ready():
            rospy.logwarn("Pan-tilt hardware not ready, sending command anyway")

        # Check for errors
        if self.has_errors():
            errors = self.get_errors()
            rospy.logwarn(f"Pan-tilt has errors: {errors}")

        # Check temperature
        if self.is_overheating():
            return ActionFuture.failure("Pan-tilt overheating, motion blocked")

        # Clamp to limits
        pan = self._clamp(pan, self._pan_min, self._pan_max)
        tilt = self._clamp(tilt, self._tilt_min, self._tilt_max)

        # Convert to degrees for command
        pan_deg = math.degrees(pan)
        tilt_deg = math.degrees(tilt)

        # Convert speed (0-1) to deg/s (max ~60 deg/s)
        speed_deg_s = int(speed * 60)
        speed_deg_s = max(1, min(60, speed_deg_s))

        # Send command
        cmd = PanTiltCmdDeg()
        cmd.yaw = pan_deg
        cmd.pitch = tilt_deg
        cmd.speed = speed_deg_s
        self._cmd_pub.publish(cmd)

        future = ActionFuture()

        def wait_for_position():
            rate = rospy.Rate(20)
            # Estimate time based on distance and speed
            pan_dist = abs(pan_deg - math.degrees(self._current_pan))
            tilt_dist = abs(tilt_deg - math.degrees(self._current_tilt))
            max_dist = max(pan_dist, tilt_dist)
            estimated_time = max_dist / speed_deg_s if speed_deg_s > 0 else 5.0
            timeout = rospy.Time.now() + rospy.Duration(estimated_time + 2.0)

            tolerance = 0.05  # radians (~2.8 degrees)

            while rospy.Time.now() < timeout and not rospy.is_shutdown():
                with self._state_lock:
                    pan_error = abs(self._current_pan - pan)
                    tilt_error = abs(self._current_tilt - tilt)

                if pan_error < tolerance and tilt_error < tolerance:
                    future._success = True
                    future._result = {
                        'pan': pan,
                        'tilt': tilt,
                        'pan_deg': pan_deg,
                        'tilt_deg': tilt_deg
                    }
                    future._done.set()
                    return

                rate.sleep()

            # Timeout - but might still be close enough
            with self._state_lock:
                pan_error = abs(self._current_pan - pan)
                tilt_error = abs(self._current_tilt - tilt)

            if pan_error < tolerance * 2 and tilt_error < tolerance * 2:
                future._success = True
                future._result = {'pan': self._current_pan, 'tilt': self._current_tilt}
            else:
                future._success = False
                future._error_msg = f"Timeout: pan_error={math.degrees(pan_error):.1f}°, tilt_error={math.degrees(tilt_error):.1f}°"
            future._done.set()

        thread = threading.Thread(target=wait_for_position, daemon=True)
        thread.start()

        return future

    def set_angles_deg(self, pan_deg: float, tilt_deg: float,
                        speed: int = 30) -> ActionFuture:
        """
        Set pan and tilt angles in degrees (direct hardware interface).

        Args:
            pan_deg: Pan angle in degrees
            tilt_deg: Tilt angle in degrees
            speed: Speed in deg/s (1-60)

        Returns:
            ActionFuture
        """
        return self.set_angles(
            math.radians(pan_deg),
            math.radians(tilt_deg),
            speed / 60.0  # Convert to 0-1 scale
        )

    def stop(self) -> None:
        """Stop head motion by commanding current position."""
        if not HAS_PAN_TILT_MSGS or self._cmd_pub is None:
            return

        with self._state_lock:
            pan = self._current_pan
            tilt = self._current_tilt

        cmd = PanTiltCmdDeg()
        cmd.yaw = math.degrees(pan)
        cmd.pitch = math.degrees(tilt)
        cmd.speed = 60  # Maximum speed for quick stop
        self._cmd_pub.publish(cmd)

    # ==================== Calibration ====================

    def set_zero(self) -> bool:
        """
        Set current position as zero reference.

        Note: This requires appropriate hardware support.

        Returns:
            True if command was sent successfully
        """
        rospy.loginfo("Requesting pan-tilt zero calibration")
        # The actual zero setting would require a service call or
        # specific message to the driver
        # For now, log the request
        rospy.logwarn("set_zero: Not implemented - requires driver support")
        return False

    # ==================== State Query ====================

    def get_raw_angles(self) -> Tuple[int, int]:
        """
        Get raw encoder values.

        Returns:
            (yaw_raw, pitch_raw) encoder counts
        """
        with self._status_lock:
            if self._last_status is None:
                return (0, 0)
            return (self._last_status.yaw_raw, self._last_status.pitch_raw)

    def get_goal_angles(self) -> Tuple[float, float]:
        """
        Get the commanded goal angles.

        Returns:
            (yaw_goal, pitch_goal) in radians
        """
        with self._status_lock:
            if self._last_status is None:
                return (0.0, 0.0)
            return (
                math.radians(self._last_status.yaw_goal),
                math.radians(self._last_status.pitch_goal)
            )

    def is_at_goal(self, tolerance: float = 0.05) -> bool:
        """
        Check if head has reached its goal position.

        Args:
            tolerance: Tolerance in radians

        Returns:
            True if at goal
        """
        current = self.get_angles()
        goal = self.get_goal_angles()

        pan_error = abs(current[0] - goal[0])
        tilt_error = abs(current[1] - goal[1])

        return pan_error < tolerance and tilt_error < tolerance

    def get_timing_info(self) -> Dict[str, int]:
        """
        Get driver timing information.

        Returns:
            Dict with 'loop_error', 'loop_time'
        """
        with self._status_lock:
            if self._last_status is None:
                return {'loop_error': 0, 'loop_time': 0}
            return {
                'loop_error': self._last_status.loop_ec,
                'loop_time': self._last_status.loop_time
            }

    def get_full_status(self) -> Dict:
        """
        Get complete status information.

        Returns:
            Dict with all status fields
        """
        with self._status_lock:
            if self._last_status is None:
                return {'available': False}

            return {
                'available': True,
                'hardware_ready': self._hardware_ready,
                'angles': self.get_angles(),
                'angles_deg': self.get_angles_deg(),
                'goal': self.get_goal_angles(),
                'temperatures': self.get_temperatures(),
                'errors': self.get_errors(),
                'device_info': self.get_device_info(),
                'raw': self.get_raw_angles(),
                'timing': self.get_timing_info()
            }
