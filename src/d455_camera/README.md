# D455 Camera

Intel RealSense D455 深度相机的 ROS 驱动封装。

> 说明：本目录是 **ROS1 参考实现**。当前 ROS2 对应入口位于 `src/d455_camera_ros2/`，启动包名为 `d455_camera_bringup`。

## 原理

基于 Intel RealSense SDK 2.0 和 librealsense，通过 USB3.0 连接 D455 相机，获取 RGB 图像、深度图像、IMU 数据和点云。

使用 nodelet 架构实现零拷贝图像传输，降低 CPU 开销。

## 节点

### d455_camera_nodelet

通过 nodelet manager 运行的相机节点。

**发布话题：**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/d455/color/image_raw` | `sensor_msgs/Image` | RGB 图像 (30Hz) |
| `/d455/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 对齐到彩色的深度图 |
| `/d455/depth/color/points` | `sensor_msgs/PointCloud2` | 彩色点云 |
| `/d455/color/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |
| `/d455/imu` | `sensor_msgs/Imu` | IMU 数据 (200Hz) |

## 启动

```bash
roslaunch d455_camera d455_camera_nodelet.launch
```

## 参数

| 参数 | 默认值 | 说明 |
|------|-------|------|
| `serial_no` | "" | 指定相机序列号 |
| `color_width` | 640 | 彩色图像宽度 |
| `color_height` | 480 | 彩色图像高度 |
| `color_fps` | 30 | 彩色图像帧率 |
| `depth_width` | 640 | 深度图像宽度 |
| `depth_height` | 480 | 深度图像高度 |
| `depth_fps` | 30 | 深度图像帧率 |
| `enable_pointcloud` | true | 启用点云发布 |
| `enable_imu` | true | 启用 IMU |

## 依赖

- librealsense2
- realsense2_camera (ROS 包)
