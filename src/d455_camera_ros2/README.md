# d455_camera_ros2 (d455_camera)

Intel RealSense D455 深度相机的 ROS2 驱动包，负责采集 RGB 彩色图像和深度图像并发布为标准 `sensor_msgs` 话题。

## 架构概览

```
┌─────────────────────────────────────────────────────────────┐
│                     d455_camera_node                        │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐   │
│  │ RealSense    │───▶│ Capture Loop │───▶│ ROS2         │   │
│  │ Pipeline     │    │ (独立线程)    │    │ Publishers   │   │
│  │ (librealsense2)   │              │    │              │   │
│  └──────────────┘    └──────────────┘    └──────────────┘   │
└─────────────────────────────────────────────────────────────┘
      ▼ 发布                                    ▼ 发布
 color/image_raw                          depth/image_raw
 color/camera_info                        depth/camera_info
```

### 设计要点

- **独立采集线程**：使用 `std::thread` 在独立线程中执行 `pipeline_.wait_for_frames()` 阻塞式采集，避免阻塞 ROS2 主循环。
- **按需发布**：发布前检查 `get_subscription_count()`，无订阅者时跳过序列化与发布，节省 CPU 和带宽。
- **硬件时间同步**：默认启用 `RS2_OPTION_GLOBAL_TIME_ENABLED` 以获取硬件时间戳，失败时自动回退到 ROS 系统时间。
- **优雅降级编译**：`CMakeLists.txt` 中使用 `find_path/find_library` 检测 librealsense2，未安装时跳过编译并输出警告。

## 目录结构

```
d455_camera_ros2/
├── CMakeLists.txt          # 构建配置（ament_cmake）
├── package.xml             # 包描述与依赖声明
├── src/
│   └── d455_camera_node.cpp  # 核心节点实现
├── launch/
│   ├── d455_camera.launch.py   # 相机启动（仅驱动）
│   └── view_d455.launch.py     # 相机 + URDF 模型 + RViz 可视化
├── rviz/
│   └── view_d455.rviz          # RViz 预设配置
└── test/
```

## 节点说明

### d455_camera_node

| 属性 | 值 |
|------|------|
| 包名 | `d455_camera` |
| 可执行文件 | `d455_camera_node` |
| 节点名 | `d455_camera_node` |

#### 发布话题

| 话题 | 类型 | QoS Depth | 说明 |
|------|------|-----------|------|
| `color/image_raw` | `sensor_msgs/msg/Image` | 1 | BGR8 格式彩色图像 |
| `color/camera_info` | `sensor_msgs/msg/CameraInfo` | 1 | 彩色相机内参 |
| `depth/image_raw` | `sensor_msgs/msg/Image` | 1 | 16UC1 格式深度图像（单位 mm） |
| `depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 1 | 深度相机内参 |

#### 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `serial_no` | string | `""` | 相机序列号（空字符串则使用第一台） |
| `enable_color` | bool | `true` | 是否启用彩色流 |
| `enable_depth` | bool | `true` | 是否启用深度流 |
| `color_width` | int | `640` | 彩色图像宽度 |
| `color_height` | int | `480` | 彩色图像高度 |
| `color_fps` | int | `30` | 彩色流帧率 |
| `depth_width` | int | `640` | 深度图像宽度 |
| `depth_height` | int | `480` | 深度图像高度 |
| `depth_fps` | int | `30` | 深度流帧率 |
| `color_frame_id` | string | `d455_color_optical_frame` | 彩色图像 TF frame |
| `depth_frame_id` | string | `d455_depth_optical_frame` | 深度图像 TF frame |
| `use_hardware_time` | bool | `true` | 是否使用硬件时间戳 |

## 使用方法

### 前置条件

```bash
# 安装 librealsense2 开发包
sudo apt install librealsense2-dev librealsense2-utils
```

### 仅启动相机驱动

```bash
ros2 launch d455_camera d455_camera.launch.py \
  camera_serial:= \
  color_fps:=30 \
  depth_fps:=30
```

### 启动相机 + RViz 可视化

```bash
ros2 launch d455_camera view_d455.launch.py
```

该 launch 文件会同时启动：
1. `d455_camera_node` 采集节点
2. `unidingo_description` 中的 D455 URDF 模型（提供 TF）
3. RViz2 可视化界面

### 通过统一入口启动（推荐）

```bash
ros2 launch uni_dingo_api bringup.launch.py mode:=real perception:=true
```

## 依赖

- `rclcpp`
- `sensor_msgs`
- `librealsense2` (Intel RealSense SDK 2.0)
- `unidingo_description` (用于 TF 模型显示)
