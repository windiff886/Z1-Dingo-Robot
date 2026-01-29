#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D455CameraBackend - Real Intel RealSense D455 camera backend for PerceptionAPI
"""

import rospy
import threading
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu
from std_srvs.srv import SetBool, SetBoolRequest
from dynamic_reconfigure.client import Client as DynamicReconfigureClient
from typing import Optional, Dict, Tuple
import numpy as np

from ...api.perception_api import PerceptionAPI


class D455CameraBackend(PerceptionAPI):
    """
    Real Intel RealSense D455 camera backend.

    Extends PerceptionAPI with D455-specific features:
    - IMU data access
    - Depth post-processing control
    - Exposure and white balance control
    - Spatial/temporal filter settings
    - Hardware diagnostics
    """

    def __init__(self, config: dict = None):
        super().__init__(mode='real', config=config)

        self._config = config or {}

        # Override topics for D455
        self._rgb_topic = self._config.get('rgb_topic', '/d455/color/image_raw')
        self._depth_topic = self._config.get('depth_topic', '/d455/aligned_depth_to_color/image_raw')
        self._points_topic = self._config.get('points_topic', '/d455/depth/color/points')
        self._camera_info_topic = self._config.get('camera_info_topic', '/d455/color/camera_info')
        self._depth_info_topic = self._config.get('depth_info_topic', '/d455/depth/camera_info')

        # Additional D455-specific topics
        self._imu_topic = self._config.get('imu_topic', '/d455/imu')
        self._infra1_topic = self._config.get('infra1_topic', '/d455/infra1/image_rect_raw')
        self._infra2_topic = self._config.get('infra2_topic', '/d455/infra2/image_rect_raw')

        # Frame IDs
        self._camera_frame = self._config.get('camera_frame', 'd455_link')
        self._optical_frame = self._config.get('optical_frame', 'd455_color_optical_frame')
        self._depth_optical_frame = self._config.get('depth_optical_frame', 'd455_depth_optical_frame')

        # Hardware state
        self._camera_ready = False
        self._serial_number = ""
        self._firmware_version = ""

        # IMU data
        self._latest_imu = None
        self._imu_lock = threading.Lock()

        # Infrared images
        self._latest_infra1 = None
        self._latest_infra2 = None

        # Depth camera info
        self._depth_camera_info = None

        # Subscribe to additional topics
        self._imu_sub = rospy.Subscriber(
            self._imu_topic, Imu, self._imu_callback, queue_size=10
        )
        self._infra1_sub = None  # Lazy init
        self._infra2_sub = None

        # Dynamic reconfigure client (for runtime parameter adjustment)
        self._dyn_reconfig_client = None

        # Camera statistics
        self._frame_count = {'rgb': 0, 'depth': 0}
        self._last_frame_time = {'rgb': None, 'depth': None}
        self._fps_estimate = {'rgb': 0.0, 'depth': 0.0}

        # Override callbacks to track statistics
        self._original_rgb_callback = self._rgb_callback
        self._rgb_sub = rospy.Subscriber(
            self._rgb_topic, Image, self._rgb_callback_with_stats, queue_size=1
        )
        self._depth_sub = rospy.Subscriber(
            self._depth_topic, Image, self._depth_callback_with_stats, queue_size=1
        )

        rospy.loginfo("D455CameraBackend initialized")

    def _rgb_callback_with_stats(self, msg: Image):
        """RGB callback with statistics tracking."""
        self._frame_count['rgb'] += 1
        now = rospy.Time.now()
        if self._last_frame_time['rgb'] is not None:
            dt = (now - self._last_frame_time['rgb']).to_sec()
            if dt > 0:
                # Exponential moving average
                self._fps_estimate['rgb'] = 0.9 * self._fps_estimate['rgb'] + 0.1 * (1.0 / dt)
        self._last_frame_time['rgb'] = now

        with self._data_lock:
            self._latest_rgb = msg
            self._camera_ready = True

    def _depth_callback_with_stats(self, msg: Image):
        """Depth callback with statistics tracking."""
        self._frame_count['depth'] += 1
        now = rospy.Time.now()
        if self._last_frame_time['depth'] is not None:
            dt = (now - self._last_frame_time['depth']).to_sec()
            if dt > 0:
                self._fps_estimate['depth'] = 0.9 * self._fps_estimate['depth'] + 0.1 * (1.0 / dt)
        self._last_frame_time['depth'] = now

        with self._data_lock:
            self._latest_depth = msg

    def _imu_callback(self, msg: Imu):
        """Callback for IMU data."""
        with self._imu_lock:
            self._latest_imu = msg

    # ==================== Hardware Status ====================

    def is_camera_ready(self) -> bool:
        """Check if camera is connected and streaming."""
        return self._camera_ready

    def get_camera_info_d455(self) -> Dict:
        """
        Get D455-specific camera information.

        Returns:
            Dict with camera details
        """
        return {
            'ready': self._camera_ready,
            'serial_number': self._serial_number,
            'firmware_version': self._firmware_version,
            'frame_count': dict(self._frame_count),
            'fps': {
                'rgb': round(self._fps_estimate['rgb'], 1),
                'depth': round(self._fps_estimate['depth'], 1)
            }
        }

    def get_stream_statistics(self) -> Dict:
        """
        Get streaming statistics.

        Returns:
            Dict with frame counts and FPS estimates
        """
        return {
            'rgb': {
                'frame_count': self._frame_count['rgb'],
                'fps': round(self._fps_estimate['rgb'], 1)
            },
            'depth': {
                'frame_count': self._frame_count['depth'],
                'fps': round(self._fps_estimate['depth'], 1)
            }
        }

    # ==================== IMU Access ====================

    def get_imu(self, timeout: float = 0.1) -> Optional[Imu]:
        """
        Get the latest IMU reading.

        Args:
            timeout: Max wait time

        Returns:
            sensor_msgs/Imu or None
        """
        if self._latest_imu is None:
            end_time = rospy.Time.now() + rospy.Duration(timeout)
            rate = rospy.Rate(100)
            while rospy.Time.now() < end_time and self._latest_imu is None:
                rate.sleep()

        with self._imu_lock:
            return self._latest_imu

    def get_imu_data(self) -> Optional[Dict]:
        """
        Get IMU data as dictionary.

        Returns:
            Dict with 'linear_acceleration', 'angular_velocity', 'orientation'
        """
        imu = self.get_imu()
        if imu is None:
            return None

        return {
            'linear_acceleration': {
                'x': imu.linear_acceleration.x,
                'y': imu.linear_acceleration.y,
                'z': imu.linear_acceleration.z
            },
            'angular_velocity': {
                'x': imu.angular_velocity.x,
                'y': imu.angular_velocity.y,
                'z': imu.angular_velocity.z
            },
            'orientation': {
                'x': imu.orientation.x,
                'y': imu.orientation.y,
                'z': imu.orientation.z,
                'w': imu.orientation.w
            }
        }

    # ==================== Infrared Streams ====================

    def get_infrared(self, camera: int = 1, timeout: float = 1.0) -> Optional[Image]:
        """
        Get infrared image from stereo pair.

        Args:
            camera: 1 for left IR, 2 for right IR
            timeout: Max wait time

        Returns:
            sensor_msgs/Image or None
        """
        if camera == 1:
            if self._infra1_sub is None:
                self._infra1_sub = rospy.Subscriber(
                    self._infra1_topic, Image, self._infra1_callback, queue_size=1
                )

            if self._latest_infra1 is None:
                end_time = rospy.Time.now() + rospy.Duration(timeout)
                rate = rospy.Rate(20)
                while rospy.Time.now() < end_time and self._latest_infra1 is None:
                    rate.sleep()

            return self._latest_infra1

        elif camera == 2:
            if self._infra2_sub is None:
                self._infra2_sub = rospy.Subscriber(
                    self._infra2_topic, Image, self._infra2_callback, queue_size=1
                )

            if self._latest_infra2 is None:
                end_time = rospy.Time.now() + rospy.Duration(timeout)
                rate = rospy.Rate(20)
                while rospy.Time.now() < end_time and self._latest_infra2 is None:
                    rate.sleep()

            return self._latest_infra2

        return None

    def _infra1_callback(self, msg: Image):
        """Callback for left infrared image."""
        self._latest_infra1 = msg

    def _infra2_callback(self, msg: Image):
        """Callback for right infrared image."""
        self._latest_infra2 = msg

    # ==================== Depth Processing ====================

    def get_aligned_depth(self, timeout: float = 1.0) -> Optional[Image]:
        """
        Get depth image aligned to color frame.

        This is the same as get_depth() when using aligned_depth_to_color topic.

        Args:
            timeout: Max wait time

        Returns:
            sensor_msgs/Image or None
        """
        return self.get_depth(timeout)

    def get_depth_in_meters(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        """
        Get depth image converted to meters.

        Args:
            timeout: Max wait time

        Returns:
            numpy array (H, W) with depth in meters, or None
        """
        return self.get_depth_numpy(timeout)

    def get_depth_colorized(self, timeout: float = 1.0,
                            min_depth: float = 0.1,
                            max_depth: float = 10.0) -> Optional[np.ndarray]:
        """
        Get colorized depth image for visualization.

        Args:
            timeout: Max wait time
            min_depth: Minimum depth for colormap (meters)
            max_depth: Maximum depth for colormap (meters)

        Returns:
            numpy array (H, W, 3) with colorized depth
        """
        depth = self.get_depth_numpy(timeout)
        if depth is None:
            return None

        try:
            import cv2

            # Clip to range
            depth_clipped = np.clip(depth, min_depth, max_depth)

            # Normalize to 0-255
            depth_norm = ((depth_clipped - min_depth) / (max_depth - min_depth) * 255).astype(np.uint8)

            # Apply colormap
            depth_colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

            # Set invalid depth (0) to black
            depth_colored[depth == 0] = [0, 0, 0]

            return depth_colored

        except ImportError:
            rospy.logwarn("OpenCV not available for depth colorization")
            return None

    # ==================== Depth Camera Info ====================

    def get_depth_intrinsics(self) -> Optional[Dict]:
        """
        Get depth camera intrinsic parameters.

        Returns:
            Dict with fx, fy, cx, cy, width, height
        """
        if self._depth_camera_info is None:
            # Try to get depth camera info
            try:
                msg = rospy.wait_for_message(
                    self._depth_info_topic, CameraInfo, timeout=1.0
                )
                self._depth_camera_info = msg
            except rospy.ROSException:
                return None

        info = self._depth_camera_info
        return {
            'fx': info.K[0],
            'fy': info.K[4],
            'cx': info.K[2],
            'cy': info.K[5],
            'width': info.width,
            'height': info.height
        }

    # ==================== Configuration ====================

    def set_exposure(self, exposure_us: int, auto: bool = False) -> bool:
        """
        Set color camera exposure.

        Args:
            exposure_us: Exposure time in microseconds (1-10000)
            auto: Enable auto exposure

        Returns:
            True if successful
        """
        try:
            if self._dyn_reconfig_client is None:
                self._dyn_reconfig_client = DynamicReconfigureClient(
                    '/d455/rgb_camera', timeout=5.0
                )

            if auto:
                self._dyn_reconfig_client.update_configuration({
                    'enable_auto_exposure': True
                })
            else:
                self._dyn_reconfig_client.update_configuration({
                    'enable_auto_exposure': False,
                    'exposure': exposure_us
                })
            return True

        except Exception as e:
            rospy.logwarn(f"Failed to set exposure: {e}")
            return False

    def set_white_balance(self, temperature: int, auto: bool = False) -> bool:
        """
        Set white balance.

        Args:
            temperature: Color temperature (2800-6500K)
            auto: Enable auto white balance

        Returns:
            True if successful
        """
        try:
            if self._dyn_reconfig_client is None:
                self._dyn_reconfig_client = DynamicReconfigureClient(
                    '/d455/rgb_camera', timeout=5.0
                )

            if auto:
                self._dyn_reconfig_client.update_configuration({
                    'enable_auto_white_balance': True
                })
            else:
                self._dyn_reconfig_client.update_configuration({
                    'enable_auto_white_balance': False,
                    'white_balance': temperature
                })
            return True

        except Exception as e:
            rospy.logwarn(f"Failed to set white balance: {e}")
            return False

    def set_laser_power(self, power: int) -> bool:
        """
        Set depth sensor laser power.

        Args:
            power: Laser power (0-360 mW)

        Returns:
            True if successful
        """
        try:
            if self._dyn_reconfig_client is None:
                self._dyn_reconfig_client = DynamicReconfigureClient(
                    '/d455/stereo_module', timeout=5.0
                )

            self._dyn_reconfig_client.update_configuration({
                'laser_power': power
            })
            return True

        except Exception as e:
            rospy.logwarn(f"Failed to set laser power: {e}")
            return False

    # ==================== Point Cloud Processing ====================

    def get_pointcloud_numpy(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        """
        Get point cloud as numpy array.

        Args:
            timeout: Max wait time

        Returns:
            numpy array (N, 3) with XYZ points, or None
        """
        pc = self.get_pointcloud(timeout)
        if pc is None:
            return None

        try:
            import sensor_msgs.point_cloud2 as pc2

            # Convert to numpy
            points = []
            for p in pc2.read_points(pc, field_names=("x", "y", "z"), skip_nans=True):
                points.append([p[0], p[1], p[2]])

            return np.array(points, dtype=np.float32)

        except Exception as e:
            rospy.logwarn(f"Point cloud conversion failed: {e}")
            return None

    def get_pointcloud_with_color(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        """
        Get point cloud with color as numpy array.

        Args:
            timeout: Max wait time

        Returns:
            numpy array (N, 6) with XYZRGB, or None
        """
        pc = self.get_pointcloud(timeout)
        if pc is None:
            return None

        try:
            import sensor_msgs.point_cloud2 as pc2
            import struct

            points = []
            for p in pc2.read_points(pc, field_names=("x", "y", "z", "rgb"), skip_nans=True):
                # Unpack RGB from float
                rgb_int = struct.unpack('I', struct.pack('f', p[3]))[0]
                r = (rgb_int >> 16) & 0xFF
                g = (rgb_int >> 8) & 0xFF
                b = rgb_int & 0xFF
                points.append([p[0], p[1], p[2], r, g, b])

            return np.array(points, dtype=np.float32)

        except Exception as e:
            rospy.logwarn(f"Colored point cloud conversion failed: {e}")
            return None

    # ==================== RGBD Frame ====================

    def get_rgbd_frame(self, timeout: float = 1.0) -> Optional[Dict]:
        """
        Get synchronized RGBD frame.

        Returns:
            Dict with 'rgb', 'depth', 'camera_info', 'timestamp'
        """
        rgb = self.get_rgb_numpy(timeout)
        depth = self.get_depth_numpy(timeout)
        info = self.get_intrinsics()

        if rgb is None or depth is None:
            return None

        # Use original image timestamp, not current time
        with self._data_lock:
            if self._latest_rgb is not None:
                timestamp = self._latest_rgb.header.stamp.to_sec()
            else:
                timestamp = 0.0

        return {
            'rgb': rgb,
            'depth': depth,
            'camera_info': info,
            'timestamp': timestamp
        }

    # ==================== Full Status ====================

    def get_full_status(self) -> Dict:
        """
        Get complete camera status.

        Returns:
            Dict with all status information
        """
        return {
            'ready': self._camera_ready,
            'mode': self._mode,
            'topics': self.get_topics(),
            'statistics': self.get_stream_statistics(),
            'camera_info': self.get_camera_info_d455(),
            'has_rgb': self._latest_rgb is not None,
            'has_depth': self._latest_depth is not None,
            'has_pointcloud': self._latest_points is not None,
            'has_imu': self._latest_imu is not None
        }
