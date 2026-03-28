# Z1-Dingo-Robot

Clearpath Dingo 全向底盘 + 双 Unitree Z1 六自由度机械臂 + IQR Pan-Tilt 云台 + Intel RealSense D455 深度相机的 **ROS2 Humble** 集成工作区。

## 系统概览

| 子系统 | 硬件 | ROS2 包 |
|--------|------|---------|
| 移动底盘 | Clearpath Dingo-O (全向) | 外部 `clearpath_robot` / `clearpath_common` |
| 机械臂 ×2 | Unitree Z1 (6-DOF) | `z1_description`, `z1_hardware`, `z1_bringup`, `z1_moveit_config` |
| 云台 | IQR Pan-Tilt (Modbus RTU) | `pan_tilt_driver`, `pan_tilt_bringup`, `pan_tilt_description` |
| 深度相机 | Intel RealSense D455 | `d455_camera_bringup` |
| 组合描述 | Dingo + 双臂 + 云台 + D455 | `unidingo_description` |
| 统一接口 | 整机 Python API + 系统启动 | `uni_dingo_api` |
| 示例工具 | 键盘遥控、位姿可视化 | `whole_body_example` |

## 环境要求

- Ubuntu 22.04
- ROS2 Humble
- `colcon`
- MoveIt2
- `ros2_control` / `ros2_controllers`
- `realsense2_camera`
- 如需底盘接入：Clearpath ROS2 官方包

## 编译

```bash
source /opt/ros/humble/setup.bash
cd /path/to/Z1-Dingo-Robot
colcon build
source install/setup.bash
```

## 统一启动

### 总入口

```bash
ros2 launch uni_dingo_api bringup.launch.py
```

`bringup.launch.py` 已恢复为 ROS1 风格总入口，默认等价于：

```bash
ros2 launch uni_dingo_api bringup.launch.py mode:=simulation
```

当前顶层关键参数：

- `mode:=simulation|real`
- `gui:=true|false`
- `rviz:=true|false`
- `world:=<gazebo_world>`
- `paused:=true|false`
- `teleop:=keyboard|joystick|none`
- `moveit:=true|false`
- `auto_safe_pose:=true|false`
- `base:=true|false`
- `left_arm:=true|false`
- `right_arm:=true|false`
- `head:=true|false`
- `perception:=true|false`
- `state_publisher:=true|false`
- `has_gripper:=true|false`

### Gazebo 仿真

```bash
ros2 launch uni_dingo_api bringup.launch.py \
  mode:=simulation \
  teleop:=keyboard \
  rviz:=true
```

默认 world 为 `dingo_race.world`，默认启动 MoveIt2，并按需接入底盘控制、双臂控制器、safe pose、云台仿真控制链和统一状态发布。

### 真机模式

```bash
ros2 launch uni_dingo_api bringup.launch.py \
  mode:=real \
  left_arm_ip:=192.168.123.110 \
  left_port_to_mcu:=8881 \
  left_port_to_sdk:=8871 \
  left_own_port:=8872 \
  right_arm_ip:=192.168.123.111 \
  right_port_to_mcu:=8882 \
  right_port_to_sdk:=8873 \
  right_own_port:=8874
```

如果只想直接走真机子入口，也可以使用：

```bash
ros2 launch uni_dingo_api real.launch.py \
  left_arm_ip:=192.168.123.110 \
  right_arm_ip:=192.168.123.111
```

真机入口的核心连接参数为：

- 左臂：`left_arm_ip`、`left_port_to_mcu`、`left_port_to_sdk`、`left_own_port`
- 右臂：`right_arm_ip`、`right_port_to_mcu`、`right_port_to_sdk`、`right_own_port`
- 云台：`pan_tilt_port`
- 相机：`camera_serial`、`color_fps`、`depth_fps`

`base:=true` 时仍通过 Clearpath ROS2 官方包接入底盘，但外部适配细节不再暴露给顶层用户。

## 模型可视化

滑块与 RViz 已从统一 bringup 中拆出，单独用于模型查看：

```bash
ros2 launch uni_dingo_api visualization.launch.py
```

常用参数：

- `gui:=true|false`
  控制是否启用 `joint_state_publisher_gui`
- `use_rviz:=true|false`
  控制是否启动 RViz2
- `head:=true|false`
- `perception:=true|false`
- `has_gripper:=true|false`

## 独立组件启动

### 双臂硬件与规划

```bash
ros2 launch z1_bringup z1_dual_bringup.launch.py \
  left_arm_ip:=192.168.123.110 \
  left_port_to_mcu:=8881 \
  left_port_to_sdk:=8871 \
  left_own_port:=8872 \
  right_arm_ip:=192.168.123.111 \
  right_port_to_mcu:=8882 \
  right_port_to_sdk:=8873 \
  right_own_port:=8874

ros2 launch z1_moveit_config move_group.launch.py
ros2 launch z1_moveit_config moveit_rviz.launch.py
```

### 组合模型与视图

```bash
ros2 launch uni_dingo_api visualization.launch.py
ros2 launch unidingo_description dual_arm_display.launch.py
ros2 launch unidingo_description dual_arm_hw.launch.py
ros2 launch unidingo_description dual_arm_world.launch.py
ros2 launch unidingo_description view_d455_only.launch.py
```

### 云台

```bash
ros2 launch pan_tilt_bringup pan_tilt_bringup.launch.py pan_tilt_port:=/dev/ttyUSB0
ros2 launch pan_tilt_description view_model.launch.py
ros2 launch pan_tilt_description view_pan_tilt_only.launch.py
ros2 run pan_tilt_driver pan_tilt_control.py
```

### 相机

```bash
ros2 launch d455_camera_bringup d455_camera.launch.py \
  camera_serial:= \
  color_fps:=30 \
  depth_fps:=30

ros2 launch d455_camera_bringup view_d455.launch.py
```

### 示例工具

```bash
ros2 run whole_body_example keyboard_teleop --ros-args -p mode:=simulation
ros2 launch whole_body_example pose_visualizer.launch.py
```

## Python API 使用

```python
import rclpy
from geometry_msgs.msg import Pose
from uni_dingo_api import RobotAPI

rclpy.init()
robot = RobotAPI(mode='simulation')

if robot.is_ready():
    robot.base.move(linear_x=0.3, linear_y=0.0, angular_z=0.0)
    robot.left_arm.move_to_named('home').wait()

target = Pose()
target.position.x = 0.4
target.position.z = 0.3
robot.left_arm.move_to_pose(target).wait()

robot.whole_body_move(
    base_goal=(0.5, 0.0, 0.0),
    left_arm_goal=[0.0, 0.2, -0.2, 0.0, 0.0, 0.0],
).wait()

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
| 云台归零 | `H` | |
| 退出 | `ESC` | |

## 文档

- [项目架构](docs/architecture.md)
- [API 框架](docs/api_framework.md)
- [ROS2 严格对齐状态](docs/ros2_strict_alignment_status.md)
- [ROS2 完全对齐实施计划](docs/superpowers/plans/2026-03-27-ros2-full-alignment-plan.md)

## 目录结构

```text
src/
├── uni_dingo_msgs_ros2/
├── pan_tilt_ros2/
│   ├── pan_tilt_msgs/
│   ├── pan_tilt_driver/
│   ├── pan_tilt_description/
│   └── pan_tilt_bringup/
├── d455_camera_ros2/
├── z1_ros2/
│   ├── z1_description/
│   ├── z1_hardware/
│   ├── z1_bringup/
│   └── z1_moveit_config/
├── unidingo_description_ros2/
├── uni_dingo_api_ros2/
└── whole_body_example_ros2/
```

> 旧 ROS1 包（如 `src/dingo/`、`src/z1_ros/`、`src/pan_tilt_ros/`、`src/d455_camera/`）保留为迁移参考，不参与 ROS2 构建。
