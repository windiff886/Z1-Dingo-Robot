# Phase 3 实施记录：真机后端适配

**完成日期**: 2026-01-28
**状态**: ✅ 已完成

---

## 1. 概述

Phase 3 完成了所有子系统的真机后端实现，包括 Dingo 底盘、Z1 机械臂、Pan-Tilt 云台和 D455 相机。

### 1.1 完成的目标

| 任务 | 状态 | 说明 |
|------|------|------|
| Dingo 底盘真机后端 | ✅ | 电池监控、诊断、急停支持 |
| Z1 机械臂真机后端 | ✅ | 双臂支持、硬件状态监控、关节限制 |
| Pan-Tilt 云台真机后端 | ✅ | 温度监控、错误检测、设备信息 |
| D455 相机真机后端 | ✅ | IMU、红外、深度处理、配置控制 |
| 真机启动文件 | ✅ | real_robot.launch 完整配置 |

---

## 2. 新增文件清单

```
src/uni_dingo_api/
├── src/uni_dingo_api/backends/real/
│   ├── __init__.py              # [更新] 导出所有后端
│   ├── dingo_base.py            # [新增] Dingo 底盘后端
│   ├── z1_arm.py                # [新增] Z1 机械臂后端
│   ├── pan_tilt_head.py         # [新增] Pan-Tilt 云台后端
│   └── d455_camera.py           # [新增] D455 相机后端
├── launch/
│   ├── bringup.launch           # [更新] 支持真机模式
│   └── real_robot.launch        # [新增] 真机专用启动文件
```

---

## 3. 后端接口说明

### 3.1 DingoBaseBackend - Dingo 底盘

```python
from uni_dingo_api.backends.real import DingoBaseBackend

base = DingoBaseBackend(config)

# 电池状态
voltage = base.get_battery_voltage()          # 电池电压
percentage = base.get_battery_percentage()    # 电池百分比 (0-100)
low = base.is_battery_low(threshold=20.0)     # 电量是否过低

# 安全状态
estop = base.is_estop_active()                # 急停是否激活
base.move(0.5, 0.0, 0.0)                      # 会自动检查急停状态

# 诊断信息
diag = base.get_diagnostics()                 # 获取诊断信息
has_err = base.has_errors()                   # 是否有错误

# 速度限制
base.set_max_velocity(linear=1.0, angular=2.0)
```

### 3.2 Z1ArmBackend - Z1 机械臂

```python
from uni_dingo_api.backends.real import Z1ArmBackend

arm = Z1ArmBackend(side='left', config=config)  # 或 'right', 'both'

# 硬件状态
ready = arm.is_hardware_ready()               # 硬件是否就绪
temps = arm.get_motor_temperatures()          # 电机温度
hot = arm.is_overheating(threshold=70.0)      # 是否过热
errors = arm.has_motor_errors()               # 是否有电机错误

# 运动控制（绕过 MoveIt，直接使用轨迹控制器）
arm.move_to_joint_direct(positions=[0, 0, 0, 0, 0, 0], duration=2.0).wait()

# 夹爪控制（支持速度和力矩控制）
arm.gripper_open(speed=0.5).wait()
arm.gripper_close(max_torque=10.0).wait()
state = arm.get_gripper_state()               # 夹爪状态（位置、速度、力矩、是否卡住）

# 安全
arm.emergency_stop()                          # 紧急停止
arm.clear_errors()                            # 清除错误

# 硬件信息
info = arm.get_arm_info()
# {'ip': '192.168.123.110', 'ports': {...}, 'hardware_ready': True, ...}
```

### 3.3 PanTiltHeadBackend - Pan-Tilt 云台

```python
from uni_dingo_api.backends.real import PanTiltHeadBackend

head = PanTiltHeadBackend(config)

# 硬件状态
ready = head.is_hardware_ready()              # 硬件是否就绪
temps = head.get_temperatures()               # {'yaw': 45.0, 'pitch': 42.0}
hot = head.is_overheating()                   # 是否过热
errors = head.has_errors()                    # 是否有错误
err_codes = head.get_errors()                 # 错误码详情

# 运动控制
head.set_angles(pan=0.5, tilt=-0.3, speed=0.5).wait()
head.set_angles_deg(pan_deg=30, tilt_deg=-20, speed=30).wait()

# 状态查询
pan, tilt = head.get_angles()                 # 当前角度（弧度）
goal = head.get_goal_angles()                 # 目标角度
at_goal = head.is_at_goal()                   # 是否到达目标
raw = head.get_raw_angles()                   # 原始编码器值

# 设备信息
info = head.get_device_info()
# {'serial': '...', 'hw_version': '...', 'sw_version': '...', 'port': '/dev/pan_tilt'}

# 完整状态
status = head.get_full_status()
```

### 3.4 D455CameraBackend - D455 相机

```python
from uni_dingo_api.backends.real import D455CameraBackend

cam = D455CameraBackend(config)

# 硬件状态
ready = cam.is_camera_ready()                 # 相机是否就绪
info = cam.get_camera_info_d455()             # D455 特定信息
stats = cam.get_stream_statistics()           # 帧率统计

# 图像获取
rgb = cam.get_rgb_numpy()                     # RGB 图像
depth = cam.get_depth_in_meters()             # 深度图（米）
depth_color = cam.get_depth_colorized()       # 彩色深度图
aligned = cam.get_aligned_depth()             # 对齐到RGB的深度

# 点云
points = cam.get_pointcloud_numpy()           # (N, 3) XYZ
points_color = cam.get_pointcloud_with_color()  # (N, 6) XYZRGB

# IMU
imu = cam.get_imu()                           # sensor_msgs/Imu
imu_data = cam.get_imu_data()                 # dict格式

# 红外图像
ir_left = cam.get_infrared(camera=1)
ir_right = cam.get_infrared(camera=2)

# RGBD 帧
frame = cam.get_rgbd_frame()
# {'rgb': ..., 'depth': ..., 'camera_info': ..., 'timestamp': ...}

# 配置（需要 dynamic_reconfigure）
cam.set_exposure(exposure_us=5000, auto=False)
cam.set_white_balance(temperature=5000, auto=False)
cam.set_laser_power(power=200)

# 深度相机内参
depth_intrinsics = cam.get_depth_intrinsics()

# 完整状态
status = cam.get_full_status()
```

---

## 4. 启动方式

### 4.1 仿真模式（默认）

```bash
# 启动仿真
roslaunch uni_dingo_api bringup.launch

# 禁用某些子系统
roslaunch uni_dingo_api bringup.launch head:=false perception:=false
```

### 4.2 真机模式

```bash
# 启动所有真机子系统
roslaunch uni_dingo_api bringup.launch mode:=real

# 或使用专用启动文件
roslaunch uni_dingo_api real_robot.launch

# 禁用某些子系统
roslaunch uni_dingo_api real_robot.launch head:=false

# 仅启动机械臂（不启动底盘）
roslaunch uni_dingo_api real_robot.launch base:=false

# 启用 MoveIt
roslaunch uni_dingo_api real_robot.launch moveit:=true rviz:=true

# 自定义臂IP
roslaunch uni_dingo_api real_robot.launch left_arm_ip:=192.168.123.110 right_arm_ip:=192.168.123.111
```

---

## 5. 硬件连接要求

### 5.1 Dingo 底盘
- 以太网连接
- 确保 dingo_base 驱动已安装

### 5.2 Z1 机械臂
- 左臂: `192.168.123.110`
  - MCU 端口: 8881, SDK 端口: 8871
- 右臂: `192.168.123.111`
  - MCU 端口: 8882, SDK 端口: 8873
- 确保网络配置正确（主机IP在同一网段）

### 5.3 Pan-Tilt 云台
- USB 串口连接
- 默认设备: `/dev/pan_tilt`
- 需要 udev 规则或手动指定端口

### 5.4 D455 相机
- USB 3.0 连接
- 需要 librealsense 驱动

---

## 6. 架构图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                            RobotAPI                                       │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌────────────────┐  │
│  │   BaseAPI    │ │    ArmAPI    │ │   HeadAPI    │ │ PerceptionAPI  │  │
│  └──────┬───────┘ └──────┬───────┘ └──────┬───────┘ └───────┬────────┘  │
└─────────┼────────────────┼────────────────┼─────────────────┼───────────┘
          │                │                │                 │
          ▼                ▼                ▼                 ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         Backend Layer                                     │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌────────────────┐  │
│  │ Simulation   │ │ Simulation   │ │ Simulation   │ │  Simulation    │  │
│  │ GazeboBase   │ │ MoveItArm    │ │   HeadAPI    │ │ PerceptionAPI  │  │
│  └──────────────┘ └──────────────┘ └──────────────┘ └────────────────┘  │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌────────────────┐  │
│  │    Real      │ │    Real      │ │    Real      │ │     Real       │  │
│  │ DingoBase    │ │  Z1Arm       │ │ PanTiltHead  │ │  D455Camera    │  │
│  └──────┬───────┘ └──────┬───────┘ └──────┬───────┘ └───────┬────────┘  │
└─────────┼────────────────┼────────────────┼─────────────────┼───────────┘
          │                │                │                 │
          ▼                ▼                ▼                 ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         Hardware Layer                                    │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌────────────────┐  │
│  │ dingo_node   │ │  z1_hw_node  │ │ PanTiltDriver│ │ D455 Nodelet   │  │
│  │   (ROS)      │ │   (ROS)      │ │   (Serial)   │ │   (USB)        │  │
│  └──────────────┘ └──────────────┘ └──────────────┘ └────────────────┘  │
│                                                                          │
│  Dingo Base       Z1 Arms          Pan-Tilt Unit   RealSense D455       │
│  (Ethernet)       (UDP/Ethernet)   (USB Serial)    (USB 3.0)            │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 7. ROS 话题映射

### 7.1 Dingo 底盘（真机）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/dingo/battery` | `sensor_msgs/BatteryState` | 电池状态 |
| `/dingo/e_stop` | `std_msgs/Bool` | 急停状态 |
| `/diagnostics_agg` | `diagnostic_msgs/DiagnosticArray` | 诊断信息 |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度命令 |

### 7.2 Z1 机械臂（真机）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/{side}_arm/joint_states` | `sensor_msgs/JointState` | 关节状态 |
| `/{side}_arm/{side}_z1_joint_traj_controller/command` | `trajectory_msgs/JointTrajectory` | 轨迹命令 |
| `/{side}_arm/{side}_z1_joint_traj_controller/follow_joint_trajectory` | `Action` | 轨迹执行 |

### 7.3 Pan-Tilt 云台（真机）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/pan_tilt_cmd_deg` | `pan_tilt_msgs/PanTiltCmdDeg` | 角度命令 |
| `/pan_tilt_status` | `pan_tilt_msgs/PanTiltStatus` | 状态反馈 |

### 7.4 D455 相机（真机）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/d455/color/image_raw` | `sensor_msgs/Image` | RGB 图像 |
| `/d455/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 对齐深度图 |
| `/d455/depth/color/points` | `sensor_msgs/PointCloud2` | 彩色点云 |
| `/d455/imu` | `sensor_msgs/Imu` | IMU 数据 |
| `/d455/infra1/image_rect_raw` | `sensor_msgs/Image` | 左红外 |
| `/d455/infra2/image_rect_raw` | `sensor_msgs/Image` | 右红外 |

---

## 8. 安全特性

### 8.1 底盘安全
- 急停状态检测：`is_estop_active()`
- 低电量警告：`is_battery_low(threshold)`
- 急停时自动阻止运动命令

### 8.2 机械臂安全
- 硬件就绪检查：`is_hardware_ready()`
- 电机温度监控：`is_overheating(threshold)`
- 电机错误检测：`has_motor_errors()`
- 关节限制检查
- 紧急停止：`emergency_stop()`

### 8.3 云台安全
- 温度监控：`is_overheating(threshold)`
- 错误码检测：`has_errors()`
- 过热时自动阻止运动

### 8.4 相机安全
- 流状态监控
- 帧率统计

---

## 9. 后续计划

### Phase 4: 高级功能
- [ ] 全身运动控制优化（协调底盘和机械臂）
- [ ] 视觉伺服
- [ ] 行为树 / 状态机框架
- [ ] 远程监控界面

### 待完善项目
- [ ] 双臂协调控制（MoveIt dual-arm planning）
- [ ] 动态避障
- [ ] 传感器标定工具
- [ ] 硬件诊断仪表盘

---

## 10. 编译验证

```bash
cd /root/Z1-Dingo-Robot
source devel/setup.bash
catkin_make --pkg uni_dingo_api

# 编译结果: 成功 ✅
```

---

## 11. 版本更新

- **API 版本**: 0.2.0 → 0.3.0
- **新增后端**: DingoBaseBackend, Z1ArmBackend, PanTiltHeadBackend, D455CameraBackend
- **新增启动文件**: real_robot.launch
- **更新启动文件**: bringup.launch (完整真机支持)

---

*文档版本: 1.0*
*最后更新: 2026-01-28*
