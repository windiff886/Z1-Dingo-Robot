#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DingoBaseBackend - Real Dingo robot backend for BaseAPI
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray
import threading

from ..api.base_api import BaseAPI
from ..api.action_future import ActionFuture


class DingoBaseBackend(BaseAPI):
    """
    Real Dingo robot backend.

    Interfaces with the dingo_base driver through ROS topics.
    Provides battery monitoring, diagnostic access, and safety features.
    """

    def __init__(self, config: dict = None):
        super().__init__(mode='real', config=config)

        # Battery state
        self._battery_voltage = 0.0
        self._battery_percentage = 0.0
        self._battery_lock = threading.Lock()

        # Diagnostics
        self._diagnostics = None
        self._diagnostics_lock = threading.Lock()

        # Safety
        self._estop_active = False

        # Subscribe to real robot topics
        self._battery_sub = rospy.Subscriber(
            '/dingo/battery', BatteryState, self._battery_callback, queue_size=10
        )
        self._diagnostics_sub = rospy.Subscriber(
            '/diagnostics_agg', DiagnosticArray, self._diagnostics_callback, queue_size=10
        )
        self._estop_sub = rospy.Subscriber(
            '/dingo/e_stop', Bool, self._estop_callback, queue_size=10
        )

        rospy.loginfo("DingoBaseBackend initialized")

    def _battery_callback(self, msg: BatteryState):
        """Callback for battery state updates."""
        with self._battery_lock:
            self._battery_voltage = msg.voltage
            self._battery_percentage = msg.percentage * 100  # Convert to percentage

    def _diagnostics_callback(self, msg: DiagnosticArray):
        """Callback for diagnostic updates."""
        with self._diagnostics_lock:
            self._diagnostics = msg

    def _estop_callback(self, msg: Bool):
        """Callback for e-stop state."""
        self._estop_active = msg.data
        if self._estop_active:
            rospy.logwarn("E-STOP ACTIVE on Dingo base")

    # ==================== Battery Info ====================

    def get_battery_voltage(self) -> float:
        """Get current battery voltage."""
        with self._battery_lock:
            return self._battery_voltage

    def get_battery_percentage(self) -> float:
        """Get battery percentage (0-100)."""
        with self._battery_lock:
            return self._battery_percentage

    def is_battery_low(self, threshold: float = 20.0) -> bool:
        """Check if battery is below threshold percentage."""
        return self.get_battery_percentage() < threshold

    # ==================== Safety ====================

    def is_estop_active(self) -> bool:
        """Check if e-stop is active."""
        return self._estop_active

    def move(self, linear_x: float = 0.0, linear_y: float = 0.0,
             angular_z: float = 0.0) -> None:
        """
        Send velocity command to the base.
        Overridden to check e-stop state.
        """
        if self._estop_active:
            rospy.logwarn("Cannot move - E-STOP active")
            return

        super().move(linear_x, linear_y, angular_z)

    # ==================== Diagnostics ====================

    def get_diagnostics(self) -> dict:
        """
        Get robot diagnostics summary.

        Returns:
            Dictionary with diagnostic information
        """
        with self._diagnostics_lock:
            if self._diagnostics is None:
                return {'available': False}

            result = {
                'available': True,
                'timestamp': self._diagnostics.header.stamp.to_sec(),
                'status': []
            }

            for status in self._diagnostics.status:
                result['status'].append({
                    'name': status.name,
                    'level': status.level,  # 0=OK, 1=WARN, 2=ERROR
                    'message': status.message
                })

            return result

    def has_errors(self) -> bool:
        """Check if any diagnostics report errors."""
        with self._diagnostics_lock:
            if self._diagnostics is None:
                return False

            for status in self._diagnostics.status:
                if status.level == 2:  # ERROR level
                    return True
            return False

    # ==================== Configuration ====================

    def set_max_velocity(self, linear: float = None, angular: float = None) -> None:
        """
        Set maximum velocity limits.

        Args:
            linear: Max linear velocity (m/s)
            angular: Max angular velocity (rad/s)
        """
        if linear is not None:
            self._max_linear_vel = max(0.1, min(1.5, linear))
        if angular is not None:
            self._max_angular_vel = max(0.1, min(3.0, angular))

        rospy.loginfo(f"Velocity limits: linear={self._max_linear_vel}, angular={self._max_angular_vel}")
