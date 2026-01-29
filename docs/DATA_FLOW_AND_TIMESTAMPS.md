# Uni-Dingo 数据流与时间戳规范

本文档说明 Uni-Dingo 机器人系统中数据的流动路径、时间戳的产生位置及其正确使用方式。

---  

## 目录

1. [系统架构概览](#1-系统架构概览)
2. [数据流总图](#2-数据流总图)
3. [各子系统数据流详解](#3-各子系统数据流详解)
   - [3.1 底盘 (Dingo Base)](#31-底盘-dingo-base)
   - [3.2 机械臂 (Z1 Arm)](#32-机械臂-z1-arm)
   - [3.3 云台 (Pan-Tilt Head)](#33-云台-pan-tilt-head)
   - [3.4 相机 (D455 Camera)](#34-相机-d455-camera)
4. [时间戳规范](#4-时间戳规范)
5. [完整话题列表](#5-完整话题列表)
6. [当前问题与修复建议](#6-当前问题与修复建议)

---

## 1. 系统架构概览

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              应用层 (Application)                            │
│                      whole_body_example, 用户脚本                            │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           统一接口层 (Unified API)                           │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────┐  │
│  │ RobotAPI │──│ BaseAPI  │──│ ArmAPI   │──│ HeadAPI  │──│PerceptionAPI │  │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                        ┌─────────────┼─────────────┐
                        ▼             ▼             ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         硬件抽象层 (Hardware Abstraction)                    │
│                                                                              │
│   仿真后端 (simulation)              真机后端 (real)                         │
│  ┌────────────────────┐           ┌────────────────────┐                    │
│  │ GazeboBaseBackend  │           │ DingoBaseBackend   │                    │
│  │ MoveItArmBackend   │           │ Z1ArmBackend       │                    │
│  │ (Gazebo controllers)│          │ PanTiltHeadBackend │                    │
│  │                    │           │ D455CameraBackend  │                    │
│  └────────────────────┘           └────────────────────┘                    │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                            硬件驱动层 (Hardware Drivers)                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐│
│  │ dingo_driver │  │   z1_hw      │  │ pan_tilt_    │  │ realsense2_      ││
│  │ ros_control  │  │   z1_sdk     │  │ driver       │  │ camera           ││
│  └──────────────┘  └──────────────┘  └──────────────┘  └──────────────────┘│
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                               物理硬件 (Hardware)                            │
│      Dingo 底盘          Z1 机械臂 x2          Pan-Tilt 云台      D455 相机 │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. 数据流总图

### 2.1 传感器数据流 (上行: 硬件 → 应用)

```
时间戳产生位置
     │
     ▼
┌──────────┐      ┌──────────────┐      ┌──────────────┐      ┌──────────┐
│ 硬件驱动  │ ───→ │ 硬件抽象层   │ ───→ │ 统一接口层   │ ───→ │ 应用层   │
│ (打时间戳)│      │ (透传时间戳) │      │ (透传时间戳) │      │ (消费)   │
└──────────┘      └──────────────┘      └──────────────┘      └──────────┘

示例: 关节状态
┌────────────────┐    ┌─────────────────┐    ┌────────────────┐    ┌──────────┐
│ ros_control    │    │ Z1ArmBackend    │    │ ArmAPI         │    │ 用户脚本 │
│ /joint_states  │───→│ _joint_callback │───→│ get_joint_     │───→│          │
│ header.stamp   │    │ (存储原始msg)   │    │ states()       │    │          │
│ = 采集时刻     │    │                 │    │                │    │          │
└────────────────┘    └─────────────────┘    └────────────────┘    └──────────┘
```

### 2.2 命令数据流 (下行: 应用 → 硬件)

```
                                                              时间戳产生位置
                                                                    │
                                                                    ▼
┌──────────┐      ┌──────────────┐      ┌──────────────┐      ┌──────────┐
│ 应用层   │ ───→ │ 统一接口层   │ ───→ │ 硬件抽象层   │ ───→ │ 硬件驱动 │
│ (发起)   │      │ (转发)       │      │ (打命令时间戳)│      │ (执行)   │
└──────────┘      └──────────────┘      └──────────────┘      └──────────┘

示例: 机械臂轨迹
┌──────────┐    ┌─────────────────┐    ┌─────────────────────┐    ┌──────────┐
│ 用户脚本 │    │ ArmAPI          │    │ Z1ArmBackend        │    │ z1_hw    │
│ move_to_ │───→│ move_to_joint() │───→│ _execute_traj()     │───→│ 执行轨迹 │
│ joint()  │    │                 │    │ traj.header.stamp   │    │          │
│          │    │                 │    │ = rospy.Time.now()  │    │          │
└──────────┘    └─────────────────┘    └─────────────────────┘    └──────────┘
```

---

## 3. 各子系统数据流详解

### 3.1 底盘 (Dingo Base)

#### 传感器数据 (上行)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              底盘传感器数据流                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌───────────────┐                                                          │
│  │ Dingo MCU     │  IMU 原始数据                                            │
│  │ (100Hz)       │───────────────┐                                          │
│  └───────────────┘               │                                          │
│                                  ▼                                          │
│  ┌───────────────┐      ┌───────────────┐                                   │
│  │ 轮式编码器    │      │imu_filter_    │     /imu/data                     │
│  │ (100Hz)       │      │madgwick       │────────────────┐                  │
│  └───────┬───────┘      └───────────────┘                │                  │
│          │                                                │                  │
│          ▼                                                ▼                  │
│  ┌───────────────┐                              ┌───────────────┐           │
│  │ ros_control   │                              │ robot_        │           │
│  │ 轮里程计      │                              │ localization  │           │
│  └───────┬───────┘                              │ (EKF)         │           │
│          │                                      └───────┬───────┘           │
│          │  /dingo_velocity_controller/odom             │                   │
│          └──────────────────────────────────────────────┤                   │
│                                                         ▼                   │
│                                              ┌───────────────────┐          │
│    时间戳: EKF 输出时刻                      │ /odometry/filtered│          │
│    (融合轮式里程计+IMU)                      │ nav_msgs/Odometry │          │
│                                              │ header.stamp      │          │
│                                              └─────────┬─────────┘          │
│                                                        │                    │
│                                                        ▼                    │
│                                              ┌───────────────────┐          │
│                                              │ DingoBaseBackend  │          │
│                                              │ _odom_callback()  │          │
│                                              │ 存储原始 msg      │          │
│                                              └─────────┬─────────┘          │
│                                                        │                    │
│                                                        ▼                    │
│                                              ┌───────────────────┐          │
│                                              │ BaseAPI           │          │
│                                              │ get_odom()        │          │
│                                              │ get_pose_2d()     │          │
│                                              └───────────────────┘          │
└─────────────────────────────────────────────────────────────────────────────┘
```

**其他底盘状态话题:**

| 话题 | 消息类型 | 时间戳来源 | 说明 |
|------|---------|-----------|------|
| `/dingo/battery` | `sensor_msgs/BatteryState` | 驱动 | 电池状态 |
| `/dingo/e_stop` | `std_msgs/Bool` | 驱动 | 急停状态 |
| `/diagnostics_agg` | `diagnostic_msgs/DiagnosticArray` | aggregator | 诊断信息 |

#### 命令数据 (下行)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              底盘命令数据流                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌───────────────┐      ┌───────────────┐      ┌───────────────────────┐   │
│  │ 应用层        │      │ BaseAPI       │      │ /cmd_vel              │   │
│  │ robot.base.   │─────→│ move()        │─────→│ geometry_msgs/Twist   │   │
│  │ move(vx,vy,w) │      │               │      │ (无 header, 无时间戳) │   │
│  └───────────────┘      └───────────────┘      └───────────┬───────────┘   │
│                                                             │               │
│                                                             ▼               │
│                                              ┌───────────────────────┐      │
│                                              │ dingo_velocity_       │      │
│                                              │ controller            │      │
│                                              │ (ros_control)         │      │
│                                              └───────────┬───────────┘      │
│                                                          │                  │
│                                                          ▼                  │
│                                              ┌───────────────────────┐      │
│                                              │ Dingo MCU             │      │
│                                              │ 电机控制              │      │
│                                              └───────────────────────┘      │
└─────────────────────────────────────────────────────────────────────────────┘

注: Twist 消息无 header，命令为实时执行，不需要时间戳
```

---

### 3.2 机械臂 (Z1 Arm)

#### 传感器数据 (上行)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            机械臂传感器数据流                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌───────────────────────────────┐                                          │
│  │ Z1 SDK (UDP)                  │                                          │
│  │ 192.168.123.110 (左臂)        │                                          │
│  │ 192.168.123.111 (右臂)        │                                          │
│  │ 关节角度、速度、力矩 (500Hz)   │                                          │
│  └───────────────┬───────────────┘                                          │
│                  │                                                          │
│                  ▼                                                          │
│  ┌───────────────────────────────┐                                          │
│  │ z1_hw (hardware_interface)    │                                          │
│  │ 时间戳: 读取 SDK 时刻         │                                          │
│  └───────────────┬───────────────┘                                          │
│                  │                                                          │
│                  ▼                                                          │
│  ┌───────────────────────────────┐      ┌───────────────────────────────┐  │
│  │ ros_control                   │      │ /joint_states                 │  │
│  │ joint_state_controller        │─────→│ sensor_msgs/JointState        │  │
│  │                               │      │ header.stamp = 发布时刻       │  │
│  └───────────────────────────────┘      │ 包含 12 个关节 + 2 个夹爪     │  │
│                                         │ name: [left_joint1..6,        │  │
│                                         │        right_joint1..6,       │  │
│                                         │        left_jointGripper,     │  │
│                                         │        right_jointGripper]    │  │
│                                         │ position: [...]               │  │
│                                         │ velocity: [...]               │  │
│                                         │ effort: [...]                 │  │
│                                         └───────────────┬───────────────┘  │
│                                                         │                   │
│                                                         ▼                   │
│                                         ┌───────────────────────────────┐  │
│                                         │ Z1ArmBackend                  │  │
│                                         │ _joint_state_callback()       │  │
│                                         │ 存储原始 msg                  │  │
│                                         └───────────────┬───────────────┘  │
│                                                         │                   │
│                                                         ▼                   │
│                                         ┌───────────────────────────────┐  │
│                                         │ ArmAPI                        │  │
│                                         │ get_joint_states()            │  │
│                                         │ is_moving()                   │  │
│                                         └───────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### 命令数据 (下行)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            机械臂命令数据流                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌───────────────┐                                                          │
│  │ 应用层        │                                                          │
│  │ robot.left_   │                                                          │
│  │ arm.move_to_  │                                                          │
│  │ joint([...])  │                                                          │
│  └───────┬───────┘                                                          │
│          │                                                                   │
│          ▼                                                                   │
│  ┌───────────────────────────────┐                                          │
│  │ ArmAPI.move_to_joint()        │                                          │
│  │ (仿真: 调用 MoveIt)           │                                          │
│  │ (真机: 调用 Backend)          │                                          │
│  └───────────────┬───────────────┘                                          │
│                  │                                                          │
│          ┌───────┴───────────────────────────┐                              │
│          │                                   │                              │
│          ▼ (仿真)                            ▼ (真机)                       │
│  ┌───────────────────┐             ┌───────────────────────────────┐       │
│  │ MoveIt            │             │ Z1ArmBackend                  │       │
│  │ MoveGroupCommander│             │ move_to_joint_direct()        │       │
│  │ 规划+执行         │             │ 或 ActionClient               │       │
│  └─────────┬─────────┘             └───────────────┬───────────────┘       │
│            │                                       │                        │
│            ▼                                       ▼                        │
│  ┌────────────────────────────────────────────────────────────────────┐    │
│  │ /{side}_arm/{side}_z1_joint_traj_controller/follow_joint_trajectory│    │
│  │ control_msgs/FollowJointTrajectoryAction                           │    │
│  │                                                                     │    │
│  │ 或 /{side}_arm/{side}_z1_joint_traj_controller/command             │    │
│  │ trajectory_msgs/JointTrajectory                                    │    │
│  │ header.stamp = 命令发出时刻 (在 Backend 层打)                      │    │
│  └────────────────────────────────────────────────────────────────────┘    │
│            │                                                                │
│            ▼                                                                │
│  ┌───────────────────────────────┐                                         │
│  │ ros_control                   │                                         │
│  │ joint_trajectory_controller   │                                         │
│  └───────────────┬───────────────┘                                         │
│                  │                                                          │
│                  ▼                                                          │
│  ┌───────────────────────────────┐                                         │
│  │ z1_hw → Z1 SDK → 电机         │                                         │
│  └───────────────────────────────┘                                         │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

### 3.3 云台 (Pan-Tilt Head)

#### 传感器数据 (上行)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                             云台传感器数据流                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌───────────────────────────────┐                                          │
│  │ Pan-Tilt 舵机                 │                                          │
│  │ 编码器、温度传感器            │                                          │
│  └───────────────┬───────────────┘                                          │
│                  │                                                          │
│                  ▼                                                          │
│  ┌───────────────────────────────┐      ┌───────────────────────────────┐  │
│  │ pan_tilt_driver               │      │ /pan_tilt_status              │  │
│  │ (串口通信)                    │─────→│ pan_tilt_msgs/PanTiltStatus   │  │
│  │ 时间戳: 驱动读取时刻          │      │ header.stamp = 驱动发布时刻   │  │
│  └───────────────────────────────┘      │                               │  │
│                                         │ 包含:                         │  │
│                                         │  - yaw_now, pitch_now (当前角)│  │
│                                         │  - yaw_goal, pitch_goal (目标)│  │
│                                         │  - yaw_temp, pitch_temp (温度)│  │
│                                         │  - driver_ec, encoder_ec (错误)│  │
│                                         └───────────────┬───────────────┘  │
│                                                         │                   │
│                                                         ▼                   │
│                                         ┌───────────────────────────────┐  │
│                                         │ PanTiltHeadBackend            │  │
│                                         │ _status_callback()            │  │
│                                         │ 存储原始 msg                  │  │
│                                         └───────────────┬───────────────┘  │
│                                                         │                   │
│                                                         ▼                   │
│                                         ┌───────────────────────────────┐  │
│                                         │ HeadAPI                       │  │
│                                         │ get_angles()                  │  │
│                                         │ is_overheating()              │  │
│                                         └───────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### 命令数据 (下行)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                             云台命令数据流                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌───────────────┐      ┌───────────────┐      ┌───────────────────────┐   │
│  │ 应用层        │      │ HeadAPI       │      │ /pan_tilt_cmd_deg     │   │
│  │ robot.head.   │─────→│ set_angles()  │─────→│ pan_tilt_msgs/        │   │
│  │ set_angles()  │      │               │      │ PanTiltCmdDeg         │   │
│  └───────────────┘      └───────────────┘      │ (无 header)           │   │
│                                                │ yaw, pitch, speed     │   │
│                                                └───────────┬───────────┘   │
│                                                            │               │
│                                                            ▼               │
│                                               ┌───────────────────────┐    │
│                                               │ pan_tilt_driver       │    │
│                                               │ 串口发送命令          │    │
│                                               └───────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────────┘

注: PanTiltCmdDeg 消息无 header，命令为实时执行
```

---

### 3.4 相机 (D455 Camera)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                             相机传感器数据流                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌───────────────────────────────┐                                          │
│  │ Intel RealSense D455          │                                          │
│  │ RGB + Depth + IMU (30/60 Hz)  │                                          │
│  └───────────────┬───────────────┘                                          │
│                  │                                                          │
│                  ▼                                                          │
│  ┌───────────────────────────────┐                                          │
│  │ realsense2_camera (驱动)      │                                          │
│  │ 时间戳: 相机硬件时钟          │                                          │
│  └───────────────┬───────────────┘                                          │
│                  │                                                          │
│     ┌────────────┼────────────┬────────────┬────────────┐                   │
│     ▼            ▼            ▼            ▼            ▼                   │
│  ┌────────┐  ┌────────┐  ┌────────┐  ┌────────┐  ┌──────────┐              │
│  │/d455/  │  │/d455/  │  │/d455/  │  │/d455/  │  │/d455/imu │              │
│  │color/  │  │aligned_│  │depth/  │  │color/  │  │sensor_   │              │
│  │image_  │  │depth_to│  │color/  │  │camera_ │  │msgs/Imu  │              │
│  │raw     │  │_color/ │  │points  │  │info    │  │          │              │
│  │        │  │image_  │  │        │  │        │  │          │              │
│  │Image   │  │raw     │  │Point   │  │Camera  │  │          │              │
│  │        │  │Image   │  │Cloud2  │  │Info    │  │          │              │
│  └───┬────┘  └───┬────┘  └───┬────┘  └───┬────┘  └────┬─────┘              │
│      │           │           │           │            │                     │
│      │           │           │           │            │                     │
│      │      header.stamp = 相机硬件采集时刻           │                     │
│      │           │           │           │            │                     │
│      └───────────┴───────────┴───────────┴────────────┘                     │
│                              │                                              │
│                              ▼                                              │
│                  ┌───────────────────────────────┐                          │
│                  │ D455CameraBackend             │                          │
│                  │ _rgb_callback()               │                          │
│                  │ _depth_callback()             │                          │
│                  │ _imu_callback()               │                          │
│                  │ 存储原始 msg (保留时间戳)     │                          │
│                  └───────────────┬───────────────┘                          │
│                                  │                                          │
│                                  ▼                                          │
│                  ┌───────────────────────────────┐                          │
│                  │ PerceptionAPI                 │                          │
│                  │ get_rgb() → 返回原始 msg      │                          │
│                  │ get_depth() → 返回原始 msg    │                          │
│                  │ get_pointcloud()              │                          │
│                  └───────────────────────────────┘                          │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. 时间戳规范

### 4.1 时间戳产生原则

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              时间戳产生原则                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  原则: 时间戳应在数据产生的最近位置打上                                      │
│                                                                              │
│  ┌─────────────────┬─────────────────────────────────────────────────────┐  │
│  │ 数据类型        │ 时间戳产生位置                                      │  │
│  ├─────────────────┼─────────────────────────────────────────────────────┤  │
│  │ 传感器数据      │ 硬件驱动层 (采集时刻)                               │  │
│  │ (关节状态、图像、│ 例: joint_states.header.stamp 由 ros_control 打    │  │
│  │  里程计、IMU)   │      image.header.stamp 由 realsense_camera 打     │  │
│  ├─────────────────┼─────────────────────────────────────────────────────┤  │
│  │ 命令数据        │ 硬件抽象层 (发送时刻)                               │  │
│  │ (轨迹命令、     │ 例: traj.header.stamp 由 Z1ArmBackend 打           │  │
│  │  目标位姿)      │      pose_stamped.header.stamp 由 ArmAPI 打        │  │
│  ├─────────────────┼─────────────────────────────────────────────────────┤  │
│  │ 聚合状态        │ 各子数据保留原始时间戳，聚合消息标记查询时刻        │  │
│  │ (RobotState)    │ 例: robot_state.header.stamp = 查询时刻            │  │
│  │                 │      robot_state.base.header.stamp = odom 原始时间 │  │
│  └─────────────────┴─────────────────────────────────────────────────────┘  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 4.2 各层职责

| 层次 | 对传感器数据 | 对命令数据 |
|------|-------------|-----------|
| **硬件驱动层** | **打时间戳** (采集时刻) | 接收执行 |
| **硬件抽象层** | 透传时间戳 | **打时间戳** (发送时刻) |
| **统一接口层** | 透传时间戳 | 转发 |
| **应用层** | 消费时间戳 | 发起命令 |

### 4.3 正确示例 vs 错误示例

```python
# ==================== 正确示例 ====================

# 1. 传感器数据: 透传原始时间戳
class Z1ArmBackend:
    def get_joint_states(self):
        return self._latest_joint_states  # 直接返回，保留 header.stamp

class RobotStatePublisher:
    def _build_arm_state(self):
        arm = ArmState()
        arm.header.stamp = self._joint_states.header.stamp  # 用原始时间戳
        ...

# 2. 命令数据: 在发送时打时间戳
class Z1ArmBackend:
    def move_to_joint_direct(self, positions):
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()  # 命令发出时刻
        ...


# ==================== 错误示例 ====================

# 1. 在 API 层重新打时间戳 (丢失了原始采集时刻)
class RobotStatePublisher:
    def _build_arm_state(self):
        arm = ArmState()
        arm.header.stamp = rospy.Time.now()  # ✗ 错误! 应该用原始时间戳
        ...

# 2. 在获取数据时重新打时间戳
class D455CameraBackend:
    def get_rgbd_frame(self):
        return {
            'rgb': self._latest_rgb,
            'timestamp': rospy.Time.now().to_sec()  # ✗ 错误! 应该用 rgb.header.stamp
        }
```

---

## 5. 完整话题列表

### 5.1 传感器话题 (上行)

| 子系统 | 话题 | 消息类型 | 频率 | 时间戳来源 |
|--------|------|---------|------|-----------|
| **底盘** | `/odometry/filtered` | `nav_msgs/Odometry` | 50Hz | EKF 输出 |
| | `/dingo/battery` | `sensor_msgs/BatteryState` | 1Hz | 驱动 |
| | `/dingo/e_stop` | `std_msgs/Bool` | 10Hz | 驱动 |
| | `/imu/data` | `sensor_msgs/Imu` | 100Hz | imu_filter |
| | `/diagnostics_agg` | `diagnostic_msgs/DiagnosticArray` | 1Hz | aggregator |
| **机械臂** | `/joint_states` | `sensor_msgs/JointState` | 100Hz | ros_control |
| **云台** | `/pan_tilt_status` | `pan_tilt_msgs/PanTiltStatus` | 20Hz | pan_tilt_driver |
| **相机** | `/d455/color/image_raw` | `sensor_msgs/Image` | 30Hz | realsense2 |
| | `/d455/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 30Hz | realsense2 |
| | `/d455/depth/color/points` | `sensor_msgs/PointCloud2` | 30Hz | realsense2 |
| | `/d455/color/camera_info` | `sensor_msgs/CameraInfo` | 30Hz | realsense2 |
| | `/d455/imu` | `sensor_msgs/Imu` | 200Hz | realsense2 |

### 5.2 命令话题 (下行)

| 子系统 | 话题 | 消息类型 | 时间戳 |
|--------|------|---------|--------|
| **底盘** | `/cmd_vel` | `geometry_msgs/Twist` | 无 (实时) |
| **机械臂** | `/{side}_arm/{side}_z1_joint_traj_controller/command` | `trajectory_msgs/JointTrajectory` | 发送时刻 |
| | `/{side}_arm/{side}_z1_joint_traj_controller/follow_joint_trajectory` | `FollowJointTrajectoryAction` | 发送时刻 |
| **云台** | `/pan_tilt_cmd_deg` | `pan_tilt_msgs/PanTiltCmdDeg` | 无 (实时) |

### 5.3 统一状态话题

| 话题 | 消息类型 | 频率 | 说明 |
|------|---------|------|------|
| `/uni_dingo/state` | `uni_dingo_msgs/RobotState` | 10Hz | 聚合所有子系统状态 |

---

## 6. 当前问题与修复状态

### 6.1 已修复问题

| 文件 | 问题描述 | 状态 |
|------|---------|------|
| `d455_camera_nodelet.cpp` | 相机时间戳未使用 RS2 SDK 全局时间同步 | ✅ 已修复 |
| `robot_state_publisher.py:107` | `base.header.stamp` 使用了 `rospy.Time.now()` | ✅ 已修复 |
| `robot_state_publisher.py:127` | `arm.header.stamp` 使用了 `rospy.Time.now()` | ✅ 已修复 |
| `robot_state_publisher.py:178` | `head.header.stamp` 使用了 `rospy.Time.now()` | ✅ 已修复 |
| `d455_camera.py:541` | `get_rgbd_frame()` 返回的 timestamp 使用了 `rospy.Time.now()` | ✅ 已修复 |
| `robot_api.py:184` | `get_state()` 各子系统缺少独立时间戳 | ✅ 已修复 |
| `arm_api.py` | `get_joint_states()` 返回值缺少时间戳 | ✅ 已修复 |

### 6.2 修复详情

#### D455 相机时间戳对齐

`d455_camera_nodelet.cpp` 现在启用 RS2 SDK 的全局时间同步功能:

```cpp
// 在 pipeline 启动后启用全局时间同步
if (use_hardware_time_) {
    auto device = pipeline_profile_.get_device();
    for (auto &sensor : device.query_sensors()) {
        if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
            sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);
        }
    }
}

// CaptureLoop 中使用帧时间戳
if (use_hardware_time_) {
    double frame_ts_ms = frames.get_timestamp();
    stamp = ros::Time(frame_ts_ms / 1000.0);
} else {
    stamp = ros::Time::now();
}
```

#### robot_state_publisher.py 时间戳透传

```python
def _build_base_state(self) -> BaseState:
    base = BaseState()
    if self._odom:
        base.header.stamp = self._odom.header.stamp  # 使用里程计原始时间戳
    else:
        base.header.stamp = rospy.Time(0)
    ...

def _build_arm_state(self, side: str) -> ArmState:
    arm = ArmState()
    if self._joint_states:
        arm.header.stamp = self._joint_states.header.stamp  # 使用关节状态原始时间戳
    else:
        arm.header.stamp = rospy.Time(0)
    ...

def _build_head_state(self) -> HeadState:
    head = HeadState()
    if self._joint_states:
        head.header.stamp = self._joint_states.header.stamp
    else:
        head.header.stamp = rospy.Time(0)
    ...
```

#### d455_camera.py Backend 时间戳

```python
def get_rgbd_frame(self):
    ...
    with self._data_lock:
        if self._latest_rgb is not None:
            timestamp = self._latest_rgb.header.stamp.to_sec()
        else:
            timestamp = 0.0
    return {
        'rgb': rgb,
        'depth': depth,
        'camera_info': info,
        'timestamp': timestamp  # 使用图像原始时间戳
    }
```

#### robot_api.py 各子系统时间戳

`get_state()` 方法现在为每个子系统提供独立的时间戳:

```python
state['base'] = {
    ...
    'timestamp': odom.header.stamp.to_sec() if odom else 0.0
}

state['left_arm'] = {
    ...
    'timestamp': joints.get('timestamp', 0.0)  # 来自 arm_api.get_joint_states()
}

state['head'] = {
    ...
    'timestamp': head_stamp  # 来自 pan_tilt_status
}

state['perception'] = {
    ...
    'timestamp': perception_stamp  # 来自 latest_rgb.header.stamp
}
```

---

## 附录: 消息类型定义

### uni_dingo_msgs/RobotState

```
std_msgs/Header header          # 查询时刻
string mode                     # "simulation" 或 "real"
uni_dingo_msgs/BaseState base
uni_dingo_msgs/ArmState left_arm
uni_dingo_msgs/ArmState right_arm
uni_dingo_msgs/HeadState head
uni_dingo_msgs/SubsystemStatus[] subsystem_statuses
bool emergency_stopped
bool all_ready
time last_update
```

### uni_dingo_msgs/BaseState

```
std_msgs/Header header          # 里程计原始时间戳
bool enabled
bool moving
bool emergency_stopped
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
```

### uni_dingo_msgs/ArmState

```
std_msgs/Header header          # 关节状态原始时间戳
string side
bool enabled
bool moving
bool in_collision
bool at_limit
string[] joint_names
float64[] joint_positions
float64[] joint_velocities
float64[] joint_efforts
uni_dingo_msgs/GripperState gripper
```
