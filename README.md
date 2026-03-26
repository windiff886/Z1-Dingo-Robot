# Z1-Dingo-Robot

Clearpath Dingo 全向底盘 + 双 Unitree Z1 六自由度机械臂 + IQR Pan-Tilt 云台 + Intel RealSense D455 深度相机的 **ROS2 Humble** 集成工作区。

## 系统概览

| 子系统 | 硬件 | ROS2 包 |
|--------|------|---------|
| 移动底盘 | Clearpath Dingo-O (全向) | `clearpath_common` (官方) |
| 机械臂 ×2 | Unitree Z1 (6-DOF) | `z1_description`, `z1_hardware`, `z1_bringup` |
| 云台 | IQR Pan-Tilt (Modbus RTU) | `pan_tilt_driver`, `pan_tilt_bringup` |
| 深度相机 | Intel RealSense D455 | `d455_camera_bringup` (封装 `realsense2_camera`) |
| 统一接口 | — | `uni_dingo_api` |

## 环境要求

- Ubuntu 22.04
- ROS2 Humble
- colcon build tool
- MoveIt2 (`ros-humble-moveit`)
- ros2_control (`ros-humble-ros2-control`, `ros-humble-ros2-controllers`)
- RealSense SDK + ROS2 wrapper (`ros-humble-realsense2-camera`)

## 快速启动

### 编译

```bash
source /opt/ros/humble/setup.bash
cd Z1-Dingo-Robot
colcon build
source install/setup.bash
```

### 仿真模式（URDF 可视化）

```bash
ros2 launch uni_dingo_api bringup.launch.py mode:=simulation use_rviz:=true
```

### 真机模式

```bash
ros2 launch uni_dingo_api bringup.launch.py mode:=real \
  left_controller_ip:=192.168.123.110 \
  right_controller_ip:=192.168.123.111
```

### 键盘遥控

```bash
ros2 run whole_body_example keyboard_teleop --ros-args -p mode:=simulation
```

按键说明见 [键盘遥控文档](#键盘遥控按键)。

## 独立组件启动

### 双臂 (Z1)

```bash
# 单臂
ros2 launch z1_bringup z1_bringup.launch.py controller_ip:=192.168.123.110

# 双臂
ros2 launch z1_bringup z1_dual_bringup.launch.py \
  left_controller_ip:=192.168.123.110 \
  right_controller_ip:=192.168.123.111
```

### 云台 (Pan-Tilt)

```bash
ros2 launch pan_tilt_bringup pan_tilt_bringup.launch.py port:=/dev/ttyUSB0
```

### 相机 (D455)

```bash
ros2 launch d455_camera_bringup d455_camera.launch.py
```

### URDF 可视化

```bash
ros2 launch unidingo_description display.launch.py
```

## Python API 使用

```python
import rclpy
rclpy.init()

from uni_dingo_api import RobotAPI

robot = RobotAPI(mode='simulation')

# 底盘控制
robot.base.move(linear_x=0.3, linear_y=0.0, angular_z=0.0)

# 机械臂控制
robot.left_arm.move_to_named('home').wait()

# 夹爪
robot.left_arm.gripper_open().wait()
robot.left_arm.gripper_close().wait()

# 云台
robot.head.set_angles(pan=0.5, tilt=-0.2).wait()

# 感知
rgb_msg = robot.perception.get_rgb()
depth_np = robot.perception.get_depth_numpy()

# 急停与恢复
robot.emergency_stop()
robot.recover()

robot.destroy()
rclpy.shutdown()
```

## 键盘遥控按键

| 功能 | 按键 | 说明 |
|------|------|------|
| 底盘前进/后退 | `w` / `s` | Shift 加速 |
| 底盘左移/右移 | `a` / `d` | Shift 加速 |
| 底盘左转/右转 | `q` / `e` | Shift 加速 |
| 急停 | `Space` | 停止所有运动 |
| 双臂 Home | `1` | |
| 左臂 Home | `2` | |
| 右臂 Home | `3` | |
| 末端上/下 | `t` / `g` | Shift 加速 |
| 末端左/右 | `f` / `h` | |
| 末端前/后 | `r` / `y` | Shift 加速 |
| 切换控制臂 | `Tab` | 左臂 ↔ 右臂 |
| 左夹爪 开/关 | `o` / `p` | |
| 右夹爪 开/关 | `[` / `]` | |
| 云台上仰/下俯 | `i` / `k` | Shift 加速 |
| 云台左转/右转 | `j` / `l` | Shift 加速 |
| 云台归零 | `H` (大写) | |
| 退出 | `ESC` | |

## 文档

- [项目架构](docs/architecture.md) — 系统架构、目录结构、话题列表
- [API 框架](docs/api_framework.md) — Python API 接口说明

## 目录结构

```
src/
├── uni_dingo_msgs_ros2/        # 消息/服务/动作定义
├── pan_tilt_ros2/
│   ├── pan_tilt_msgs/          # 云台消息定义
│   ├── pan_tilt_driver/        # 云台驱动 (rclcpp + Modbus RTU)
│   ├── pan_tilt_description/   # 云台 URDF + 网格
│   └── pan_tilt_bringup/       # 云台启动文件
├── d455_camera_ros2/           # D455 相机启动封装
├── z1_ros2/
│   ├── z1_description/         # Z1 臂 URDF (支持 prefix) + ros2_control 配置
│   ├── z1_hardware/            # ros2_control 硬件接口 (z1_sdk)
│   └── z1_bringup/             # 单臂/双臂启动文件
├── unidingo_description_ros2/  # 组合机器人 URDF (底盘+双臂+云台+相机)
├── uni_dingo_api_ros2/         # 统一 Python API + 系统启动文件
└── whole_body_example_ros2/    # 键盘遥控等示例
```

> 旧 ROS1 包 (`src/dingo/`, `src/z1_ros/`, `src/pan_tilt_ros/` 等) 已通过 `COLCON_IGNORE` 排除，保留作为参考。
