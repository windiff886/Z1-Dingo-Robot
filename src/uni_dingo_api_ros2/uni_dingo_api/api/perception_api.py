#!/usr/bin/env python3
"""PerceptionAPI - Unified interface for robot perception (ROS2)."""

import threading
from typing import Optional, Tuple

import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Pose, TransformStamped
import tf2_ros

from .action_future import ActionFuture

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False


class PerceptionAPI:
    """Unified API for robot perception."""

    def __init__(self, node: Node, mode: str = 'simulation', config: dict = None):
        self._node = node
        self._mode = mode
        self._config = config or {}
        self._enabled = True

        if mode == 'simulation':
            self._rgb_topic = self._config.get('rgb_topic', '/camera/color/image_raw')
            self._depth_topic = self._config.get('depth_topic', '/camera/depth/image_raw')
            self._points_topic = self._config.get('points_topic', '/camera/depth/points')
            self._camera_info_topic = self._config.get('camera_info_topic', '/camera/color/camera_info')
        else:
            self._rgb_topic = self._config.get('rgb_topic', '/d455/color/image_raw')
            self._depth_topic = self._config.get('depth_topic', '/d455/depth/image_raw')
            self._points_topic = self._config.get('points_topic', '/d455/depth/color/points')
            self._camera_info_topic = self._config.get('camera_info_topic', '/d455/color/camera_info')

        self._camera_frame = self._config.get('camera_frame', 'camera_link')

        self._latest_rgb = None
        self._latest_depth = None
        self._latest_points = None
        self._latest_camera_info = None
        self._data_lock = threading.Lock()

        if HAS_CV_BRIDGE:
            self._cv_bridge = CvBridge()
        else:
            self._cv_bridge = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        self._rgb_sub = None
        self._depth_sub = None
        self._points_sub = None
        self._camera_info_sub = None

        node.get_logger().info(f'PerceptionAPI initialized in {mode} mode')

    def _ensure_subscribers(self):
        if self._rgb_sub is None:
            self._rgb_sub = self._node.create_subscription(
                Image, self._rgb_topic, self._rgb_callback, 1)
        if self._depth_sub is None:
            self._depth_sub = self._node.create_subscription(
                Image, self._depth_topic, self._depth_callback, 1)
        if self._points_sub is None:
            self._points_sub = self._node.create_subscription(
                PointCloud2, self._points_topic, self._points_callback, 1)
        if self._camera_info_sub is None:
            self._camera_info_sub = self._node.create_subscription(
                CameraInfo, self._camera_info_topic, self._camera_info_callback, 1)

    def _rgb_callback(self, msg):
        with self._data_lock:
            self._latest_rgb = msg

    def _depth_callback(self, msg):
        with self._data_lock:
            self._latest_depth = msg

    def _points_callback(self, msg):
        with self._data_lock:
            self._latest_points = msg

    def _camera_info_callback(self, msg):
        with self._data_lock:
            self._latest_camera_info = msg

    def get_rgb(self, timeout: float = 1.0) -> Optional[Image]:
        if not self._enabled:
            return None
        self._ensure_subscribers()
        if self._latest_rgb is None:
            end = self._node.get_clock().now().nanoseconds + int(timeout * 1e9)
            while self._node.get_clock().now().nanoseconds < end and self._latest_rgb is None:
                threading.Event().wait(0.05)
        with self._data_lock:
            return self._latest_rgb

    def get_rgb_numpy(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        msg = self.get_rgb(timeout)
        if msg is None or not self._cv_bridge:
            return None
        try:
            return self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return None

    def get_depth(self, timeout: float = 1.0) -> Optional[Image]:
        if not self._enabled:
            return None
        self._ensure_subscribers()
        if self._latest_depth is None:
            end = self._node.get_clock().now().nanoseconds + int(timeout * 1e9)
            while self._node.get_clock().now().nanoseconds < end and self._latest_depth is None:
                threading.Event().wait(0.05)
        with self._data_lock:
            return self._latest_depth

    def get_depth_numpy(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        msg = self.get_depth(timeout)
        if msg is None or not self._cv_bridge:
            return None
        try:
            if msg.encoding == '16UC1':
                return self._cv_bridge.imgmsg_to_cv2(msg, '16UC1').astype(np.float32) / 1000.0
            elif msg.encoding == '32FC1':
                return self._cv_bridge.imgmsg_to_cv2(msg, '32FC1')
            else:
                return self._cv_bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception:
            return None

    def get_pointcloud(self, timeout: float = 1.0) -> Optional[PointCloud2]:
        if not self._enabled:
            return None
        self._ensure_subscribers()
        if self._latest_points is None:
            end = self._node.get_clock().now().nanoseconds + int(timeout * 1e9)
            while self._node.get_clock().now().nanoseconds < end and self._latest_points is None:
                threading.Event().wait(0.05)
        with self._data_lock:
            return self._latest_points

    def get_camera_info(self, timeout: float = 1.0) -> Optional[CameraInfo]:
        if not self._enabled:
            return None
        self._ensure_subscribers()
        if self._latest_camera_info is None:
            end = self._node.get_clock().now().nanoseconds + int(timeout * 1e9)
            while self._node.get_clock().now().nanoseconds < end and self._latest_camera_info is None:
                threading.Event().wait(0.05)
        with self._data_lock:
            return self._latest_camera_info

    def get_intrinsics(self) -> Optional[dict]:
        info = self.get_camera_info()
        if info is None:
            return None
        return {
            'fx': info.k[0], 'fy': info.k[4],
            'cx': info.k[2], 'cy': info.k[5],
            'width': info.width, 'height': info.height,
        }

    def get_camera_pose(self, frame: str = 'base_link',
                        timeout: float = 1.0) -> Optional[Pose]:
        try:
            transform = self._tf_buffer.lookup_transform(
                frame, self._camera_frame, Time())
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            return pose
        except Exception:
            return None

    def lookup_transform(self, target_frame: str, source_frame: str,
                         timeout: float = 1.0) -> Optional[TransformStamped]:
        try:
            return self._tf_buffer.lookup_transform(
                target_frame, source_frame, Time())
        except Exception:
            return None

    def pixel_to_point(self, u: int, v: int, depth: float) -> Optional[Tuple[float, float, float]]:
        intrinsics = self.get_intrinsics()
        if intrinsics is None:
            return None
        fx, fy = intrinsics['fx'], intrinsics['fy']
        cx, cy = intrinsics['cx'], intrinsics['cy']
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        return (x, y, depth)

    def has_data(self) -> bool:
        with self._data_lock:
            return (self._latest_rgb is not None or
                    self._latest_depth is not None or
                    self._latest_points is not None)

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled

    def is_enabled(self) -> bool:
        return self._enabled

    def get_topics(self) -> dict:
        return {
            'rgb': self._rgb_topic, 'depth': self._depth_topic,
            'points': self._points_topic, 'camera_info': self._camera_info_topic,
        }
