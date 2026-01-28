#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PerceptionAPI - Unified interface for robot perception (cameras, sensors)
"""

import rospy
import threading
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Pose, TransformStamped
from typing import Optional, Tuple
import numpy as np

# Try to import CV bridge
try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False


class PerceptionAPI:
    """
    Unified API for robot perception.

    Provides access to cameras, depth sensors, and point clouds.
    Supports both simulation (Gazebo camera plugins) and real robot (RealSense).

    Usage:
        perception = PerceptionAPI(mode='simulation')
        rgb = perception.get_rgb()           # Get RGB image
        depth = perception.get_depth()       # Get depth image
        points = perception.get_pointcloud() # Get point cloud
    """

    def __init__(self, mode: str = 'simulation', config: dict = None):
        """
        Initialize PerceptionAPI.

        Args:
            mode: 'simulation' or 'real'
            config: Configuration dictionary
        """
        self._mode = mode
        self._config = config or {}
        self._enabled = True

        # Topic configuration
        if mode == 'simulation':
            # Gazebo camera topics
            self._rgb_topic = self._config.get(
                'rgb_topic', '/camera/color/image_raw')
            self._depth_topic = self._config.get(
                'depth_topic', '/camera/depth/image_raw')
            self._points_topic = self._config.get(
                'points_topic', '/camera/depth/points')
            self._camera_info_topic = self._config.get(
                'camera_info_topic', '/camera/color/camera_info')
        else:
            # RealSense D455 topics
            self._rgb_topic = self._config.get(
                'rgb_topic', '/d455/color/image_raw')
            self._depth_topic = self._config.get(
                'depth_topic', '/d455/depth/image_raw')
            self._points_topic = self._config.get(
                'points_topic', '/d455/depth/color/points')
            self._camera_info_topic = self._config.get(
                'camera_info_topic', '/d455/color/camera_info')

        # Frame IDs
        self._camera_frame = self._config.get('camera_frame', 'camera_link')
        self._optical_frame = self._config.get('optical_frame', 'camera_color_optical_frame')

        # Data buffers
        self._latest_rgb = None
        self._latest_depth = None
        self._latest_points = None
        self._latest_camera_info = None
        self._data_lock = threading.Lock()

        # CV Bridge for image conversion
        if HAS_CV_BRIDGE:
            self._cv_bridge = CvBridge()
        else:
            self._cv_bridge = None
            rospy.logwarn("cv_bridge not available, image conversion limited")

        # TF buffer
        try:
            import tf2_ros
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        except Exception:
            self._tf_buffer = None

        # Subscribers (lazy initialization to avoid errors if topics don't exist)
        self._rgb_sub = None
        self._depth_sub = None
        self._points_sub = None
        self._camera_info_sub = None

        rospy.loginfo(f"PerceptionAPI initialized in {mode} mode")

    def _ensure_subscribers(self):
        """Initialize subscribers if not already done."""
        if self._rgb_sub is None:
            self._rgb_sub = rospy.Subscriber(
                self._rgb_topic, Image, self._rgb_callback, queue_size=1
            )
        if self._depth_sub is None:
            self._depth_sub = rospy.Subscriber(
                self._depth_topic, Image, self._depth_callback, queue_size=1
            )
        if self._points_sub is None:
            self._points_sub = rospy.Subscriber(
                self._points_topic, PointCloud2, self._points_callback, queue_size=1
            )
        if self._camera_info_sub is None:
            self._camera_info_sub = rospy.Subscriber(
                self._camera_info_topic, CameraInfo, self._camera_info_callback, queue_size=1
            )

    def _rgb_callback(self, msg: Image):
        """Callback for RGB image."""
        with self._data_lock:
            self._latest_rgb = msg

    def _depth_callback(self, msg: Image):
        """Callback for depth image."""
        with self._data_lock:
            self._latest_depth = msg

    def _points_callback(self, msg: PointCloud2):
        """Callback for point cloud."""
        with self._data_lock:
            self._latest_points = msg

    def _camera_info_callback(self, msg: CameraInfo):
        """Callback for camera info."""
        with self._data_lock:
            self._latest_camera_info = msg

    # ==================== Image Access ====================

    def get_rgb(self, timeout: float = 1.0) -> Optional[Image]:
        """
        Get the latest RGB image.

        Args:
            timeout: Max wait time for new image

        Returns:
            sensor_msgs/Image or None
        """
        if not self._enabled:
            return None

        self._ensure_subscribers()

        # Wait for image if none available
        if self._latest_rgb is None:
            end_time = rospy.Time.now() + rospy.Duration(timeout)
            rate = rospy.Rate(20)
            while rospy.Time.now() < end_time and self._latest_rgb is None:
                rate.sleep()

        with self._data_lock:
            return self._latest_rgb

    def get_rgb_numpy(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        """
        Get RGB image as numpy array.

        Args:
            timeout: Max wait time

        Returns:
            numpy array (H, W, 3) BGR format, or None
        """
        msg = self.get_rgb(timeout)
        if msg is None:
            return None

        if self._cv_bridge:
            try:
                return self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                rospy.logwarn(f"Image conversion failed: {e}")
                return None
        return None

    def get_depth(self, timeout: float = 1.0) -> Optional[Image]:
        """
        Get the latest depth image.

        Args:
            timeout: Max wait time

        Returns:
            sensor_msgs/Image or None
        """
        if not self._enabled:
            return None

        self._ensure_subscribers()

        if self._latest_depth is None:
            end_time = rospy.Time.now() + rospy.Duration(timeout)
            rate = rospy.Rate(20)
            while rospy.Time.now() < end_time and self._latest_depth is None:
                rate.sleep()

        with self._data_lock:
            return self._latest_depth

    def get_depth_numpy(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        """
        Get depth image as numpy array.

        Args:
            timeout: Max wait time

        Returns:
            numpy array (H, W) with depth values in meters, or None
        """
        msg = self.get_depth(timeout)
        if msg is None:
            return None

        if self._cv_bridge:
            try:
                # Depth images are typically 16UC1 (mm) or 32FC1 (m)
                if msg.encoding == '16UC1':
                    depth = self._cv_bridge.imgmsg_to_cv2(msg, "16UC1")
                    return depth.astype(np.float32) / 1000.0  # Convert mm to m
                elif msg.encoding == '32FC1':
                    return self._cv_bridge.imgmsg_to_cv2(msg, "32FC1")
                else:
                    return self._cv_bridge.imgmsg_to_cv2(msg, "passthrough")
            except Exception as e:
                rospy.logwarn(f"Depth conversion failed: {e}")
                return None
        return None

    def get_pointcloud(self, timeout: float = 1.0) -> Optional[PointCloud2]:
        """
        Get the latest point cloud.

        Args:
            timeout: Max wait time

        Returns:
            sensor_msgs/PointCloud2 or None
        """
        if not self._enabled:
            return None

        self._ensure_subscribers()

        if self._latest_points is None:
            end_time = rospy.Time.now() + rospy.Duration(timeout)
            rate = rospy.Rate(20)
            while rospy.Time.now() < end_time and self._latest_points is None:
                rate.sleep()

        with self._data_lock:
            return self._latest_points

    # ==================== Camera Info ====================

    def get_camera_info(self, timeout: float = 1.0) -> Optional[CameraInfo]:
        """
        Get camera intrinsic parameters.

        Args:
            timeout: Max wait time

        Returns:
            sensor_msgs/CameraInfo or None
        """
        if not self._enabled:
            return None

        self._ensure_subscribers()

        if self._latest_camera_info is None:
            end_time = rospy.Time.now() + rospy.Duration(timeout)
            rate = rospy.Rate(20)
            while rospy.Time.now() < end_time and self._latest_camera_info is None:
                rate.sleep()

        with self._data_lock:
            return self._latest_camera_info

    def get_intrinsics(self) -> Optional[dict]:
        """
        Get camera intrinsic parameters as dictionary.

        Returns:
            Dictionary with fx, fy, cx, cy, width, height
        """
        info = self.get_camera_info()
        if info is None:
            return None

        return {
            'fx': info.K[0],
            'fy': info.K[4],
            'cx': info.K[2],
            'cy': info.K[5],
            'width': info.width,
            'height': info.height
        }

    # ==================== Transform Queries ====================

    def get_camera_pose(self, frame: str = 'base_link',
                        timeout: float = 1.0) -> Optional[Pose]:
        """
        Get camera pose relative to specified frame.

        Args:
            frame: Reference frame
            timeout: TF lookup timeout

        Returns:
            geometry_msgs/Pose or None
        """
        if self._tf_buffer is None:
            return None

        try:
            transform = self._tf_buffer.lookup_transform(
                frame, self._camera_frame,
                rospy.Time(0), rospy.Duration(timeout)
            )

            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation

            return pose

        except Exception as e:
            rospy.logwarn(f"Camera pose lookup failed: {e}")
            return None

    def lookup_transform(self, target_frame: str, source_frame: str,
                         timeout: float = 1.0) -> Optional[TransformStamped]:
        """
        Look up transform between frames.

        Args:
            target_frame: Target frame
            source_frame: Source frame
            timeout: Lookup timeout

        Returns:
            geometry_msgs/TransformStamped or None
        """
        if self._tf_buffer is None:
            return None

        try:
            return self._tf_buffer.lookup_transform(
                target_frame, source_frame,
                rospy.Time(0), rospy.Duration(timeout)
            )
        except Exception as e:
            rospy.logwarn(f"Transform lookup failed: {e}")
            return None

    # ==================== Utility Methods ====================

    def pixel_to_point(self, u: int, v: int, depth: float) -> Optional[Tuple[float, float, float]]:
        """
        Convert pixel coordinates to 3D point.

        Args:
            u: Pixel x coordinate
            v: Pixel y coordinate
            depth: Depth value in meters

        Returns:
            (x, y, z) in camera frame, or None
        """
        intrinsics = self.get_intrinsics()
        if intrinsics is None:
            return None

        fx = intrinsics['fx']
        fy = intrinsics['fy']
        cx = intrinsics['cx']
        cy = intrinsics['cy']

        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        return (x, y, z)

    def has_data(self) -> bool:
        """Check if any sensor data is available."""
        with self._data_lock:
            return (self._latest_rgb is not None or
                    self._latest_depth is not None or
                    self._latest_points is not None)

    # ==================== Enable/Disable ====================

    def set_enabled(self, enabled: bool) -> None:
        """Enable or disable perception."""
        self._enabled = enabled

    def is_enabled(self) -> bool:
        """Check if perception is enabled."""
        return self._enabled

    # ==================== Topic Info ====================

    def get_topics(self) -> dict:
        """Get configured topic names."""
        return {
            'rgb': self._rgb_topic,
            'depth': self._depth_topic,
            'points': self._points_topic,
            'camera_info': self._camera_info_topic
        }
