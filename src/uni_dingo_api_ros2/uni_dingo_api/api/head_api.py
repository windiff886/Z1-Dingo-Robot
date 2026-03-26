#!/usr/bin/env python3
"""HeadAPI - Unified interface for pan-tilt head control (ROS2)."""

import math
import threading
from typing import Optional, Tuple

from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import tf2_ros

from .action_future import ActionFuture

try:
    from pan_tilt_msgs.msg import PanTiltCmdDeg, PanTiltStatus
    HAS_PAN_TILT_MSGS = True
except ImportError:
    HAS_PAN_TILT_MSGS = False


class HeadAPI:
    """Unified API for pan-tilt head control."""

    def __init__(self, node: Node, mode: str = 'simulation', config: dict = None):
        self._node = node
        self._mode = mode
        self._config = config or {}
        self._enabled = True

        self._pan_joint = self._config.get('pan_joint', 'pan_tilt_yaw_joint')
        self._tilt_joint = self._config.get('tilt_joint', 'pan_tilt_pitch_joint')

        self._pan_min = self._config.get('pan_min', -math.pi / 3)
        self._pan_max = self._config.get('pan_max', math.pi / 3)
        self._tilt_min = self._config.get('tilt_min', -math.pi / 3)
        self._tilt_max = self._config.get('tilt_max', math.pi / 3)

        self._current_pan = 0.0
        self._current_tilt = 0.0
        self._state_lock = threading.Lock()

        if mode == 'simulation':
            self._pan_pub = node.create_publisher(
                Float64, '/pan_tilt/pan_controller/command', 10)
            self._tilt_pub = node.create_publisher(
                Float64, '/pan_tilt/tilt_controller/command', 10)
            self._joint_state_sub = node.create_subscription(
                JointState, '/joint_states', self._joint_state_callback, 10)
        else:
            if HAS_PAN_TILT_MSGS:
                self._cmd_pub = node.create_publisher(
                    PanTiltCmdDeg, '/pan_tilt_cmd_deg', 10)
                self._status_sub = node.create_subscription(
                    PanTiltStatus, '/pan_tilt_status', self._status_callback, 10)
            else:
                node.get_logger().warn('pan_tilt_msgs not available')
                self._cmd_pub = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        node.get_logger().info(f'HeadAPI initialized in {mode} mode')

    def _joint_state_callback(self, msg: JointState):
        with self._state_lock:
            if self._pan_joint in msg.name:
                idx = msg.name.index(self._pan_joint)
                self._current_pan = msg.position[idx]
            if self._tilt_joint in msg.name:
                idx = msg.name.index(self._tilt_joint)
                self._current_tilt = msg.position[idx]

    def _status_callback(self, msg):
        with self._state_lock:
            self._current_pan = math.radians(msg.yaw_now)
            self._current_tilt = math.radians(msg.pitch_now)

    def set_angles(self, pan: float, tilt: float, speed: float = 0.5) -> ActionFuture:
        if not self._enabled:
            return ActionFuture.failure('Head disabled')

        pan = self._clamp(pan, self._pan_min, self._pan_max)
        tilt = self._clamp(tilt, self._tilt_min, self._tilt_max)

        future = ActionFuture()

        if self._mode == 'simulation':
            pan_msg = Float64()
            pan_msg.data = pan
            tilt_msg = Float64()
            tilt_msg.data = tilt
            self._pan_pub.publish(pan_msg)
            self._tilt_pub.publish(tilt_msg)

            def wait_for_position():
                tolerance = 0.05
                for _ in range(100):  # 5s timeout at 20Hz
                    with self._state_lock:
                        pan_error = abs(self._current_pan - pan)
                        tilt_error = abs(self._current_tilt - tilt)
                    if pan_error < tolerance and tilt_error < tolerance:
                        future._complete(True)
                        return
                    threading.Event().wait(0.05)
                future._complete(False, error_msg='Timeout waiting for head position')

            threading.Thread(target=wait_for_position, daemon=True).start()
        else:
            if self._cmd_pub and HAS_PAN_TILT_MSGS:
                cmd = PanTiltCmdDeg()
                cmd.yaw = math.degrees(pan)
                cmd.pitch = math.degrees(tilt)
                cmd.speed = int(speed * 60)
                self._cmd_pub.publish(cmd)

                def wait_real():
                    threading.Event().wait(2.0)
                    future._complete(True)

                threading.Thread(target=wait_real, daemon=True).start()
            else:
                return ActionFuture.failure('Pan-tilt driver not available')

        return future

    def look_at(self, point: PointStamped) -> ActionFuture:
        if not self._enabled:
            return ActionFuture.failure('Head disabled')

        try:
            from rclpy.time import Time
            head_frame = 'pan_tilt_base_link'
            transform = self._tf_buffer.lookup_transform(
                head_frame, point.header.frame_id, Time())

            import tf2_geometry_msgs
            point_transformed = tf2_geometry_msgs.do_transform_point(point, transform)

            x = point_transformed.point.x
            y = point_transformed.point.y
            z = point_transformed.point.z

            pan = math.atan2(y, x)
            horizontal_dist = math.sqrt(x * x + y * y)
            tilt = -math.atan2(z, horizontal_dist)

            return self.set_angles(pan, tilt)
        except Exception as e:
            return ActionFuture.failure(f'look_at failed: {e}')

    def home(self) -> ActionFuture:
        return self.set_angles(0.0, 0.0)

    def stop(self) -> None:
        with self._state_lock:
            pan = self._current_pan
            tilt = self._current_tilt
        if self._mode == 'simulation':
            pan_msg = Float64()
            pan_msg.data = pan
            tilt_msg = Float64()
            tilt_msg.data = tilt
            self._pan_pub.publish(pan_msg)
            self._tilt_pub.publish(tilt_msg)

    def get_angles(self) -> Tuple[float, float]:
        with self._state_lock:
            return (self._current_pan, self._current_tilt)

    def get_angles_deg(self) -> Tuple[float, float]:
        pan, tilt = self.get_angles()
        return (math.degrees(pan), math.degrees(tilt))

    def get_limits(self) -> dict:
        return {
            'pan_min': self._pan_min, 'pan_max': self._pan_max,
            'tilt_min': self._tilt_min, 'tilt_max': self._tilt_max,
        }

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled

    def is_enabled(self) -> bool:
        return self._enabled

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))
