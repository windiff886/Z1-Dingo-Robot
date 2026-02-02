# Z1-Dingo-Robot 项目架构

## 项目概述

本项目是一个综合性的ROS机器人工作区，集成了Dingo移动底盘、Unitree Z1双机械臂、云台和相机系统。项目支持Gazebo仿真和MoveIt运动规划，可实现复杂的全身操控任务。

**工作区根目录:** `/root/Z1-Dingo-Robot`

**主要功能:**
- Dingo四轮差速/全向移动底盘控制
- Unitree Z1双机械臂协同控制
- MoveIt运动规划与轨迹执行
- Gazebo物理仿真
- Pan-Tilt云台控制
- Intel D455深度相机集成
- 全身操控与遥控

---

## 系统架构

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

## 目录结构

```
Z1-Dingo-Robot/
├── src/
│   ├── d455_camera/          # Intel D455 相机驱动
│   ├── pan_tilt_ros/         # 云台控制包
│   ├── z1_ros/               # Unitree Z1 机械臂相关包
│   ├── dingo/                # Dingo 底盘相关包
│   ├── uni_dingo_api/        # 统一 API 包
│   ├── uni_dingo_msgs/       # 消息定义包
│   ├── whole_body_example/   # 整机示例
│   └── unidingo_description/ # 组合机器人 URDF
├── docs/                     # 文档
│   ├── architecture.md       # 本文档
│   └── api_framework.md      # API 框架说明
└── README.md                 # 快速启动指南
```

---

## 模块划分

### 独立组件

| 模块 | 路径 | 功能 |
|------|------|------|
| D455 相机 | `src/d455_camera/` | Intel RealSense D455 深度相机驱动 |
| 云台 | `src/pan_tilt_ros/` | Pan-Tilt 云台控制（Modbus RTU） |
| 机械臂 | `src/z1_ros/` | Unitree Z1 六自由度机械臂驱动与 MoveIt |
| 底盘 | `src/dingo/` | Clearpath Dingo 移动底盘驱动 |

### 整机组合

| 模块 | 路径 | 功能 |
|------|------|------|
| 统一 API | `src/uni_dingo_api/` | 整合所有子系统的统一接口 |
| 消息定义 | `src/uni_dingo_msgs/` | RobotState, ArmState 等自定义消息 |
| 整机描述 | `src/unidingo_description/` | Dingo + 双 Z1 机械臂的组合 URDF |
| 使用示例 | `src/whole_body_example/` | 键盘遥控等整机示例 |

---

## 数据流说明

### 传感器数据流（上行：硬件 → 应用）

```
时间戳产生位置
     │
     ▼
┌──────────┐      ┌──────────────┐      ┌──────────────┐      ┌──────────┐
│ 硬件驱动  │ ───→ │ 硬件抽象层   │ ───→ │ 统一接口层   │ ───→ │ 应用层   │
│ (打时间戳)│      │ (透传时间戳) │      │ (透传时间戳) │      │ (消费)   │
└──────────┘      └──────────────┘      └──────────────┘      └──────────┘
```

### 命令数据流（下行：应用 → 硬件）

```
                                                              时间戳产生位置
                                                                    │
                                                                    ▼
┌──────────┐      ┌──────────────┐      ┌──────────────┐      ┌──────────┐
│ 应用层   │ ───→ │ 统一接口层   │ ───→ │ 硬件抽象层   │ ───→ │ 硬件驱动 │
│ (发起)   │      │ (转发)       │      │ (打命令时间戳)│      │ (执行)   │
└──────────┘      └──────────────┘      └──────────────┘      └──────────┘
```

### 时间戳规范

| 数据类型 | 时间戳产生位置 | 示例 |
|---------|---------------|------|
| 传感器数据 | 硬件驱动层（采集时刻） | joint_states.header.stamp 由 ros_control 打 |
| 命令数据 | 硬件抽象层（发送时刻） | traj.header.stamp 由 Backend 打 |
| 聚合状态 | 各子数据保留原始时间戳 | robot_state.base.header.stamp = odom 原始时间 |

---

## 话题列表

### 传感器话题（上行）

| 子系统 | 话题 | 消息类型 | 频率 |
|--------|------|---------|------|
| 底盘 | `/odometry/filtered` | `nav_msgs/Odometry` | 50Hz |
| 底盘 | `/imu/data` | `sensor_msgs/Imu` | 100Hz |
| 机械臂 | `/joint_states` | `sensor_msgs/JointState` | 100Hz |
| 云台 | `/pan_tilt_status` | `pan_tilt_msgs/PanTiltStatus` | 20Hz |
| 相机 | `/d455/color/image_raw` | `sensor_msgs/Image` | 30Hz |
| 相机 | `/d455/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 30Hz |
| 相机 | `/d455/depth/color/points` | `sensor_msgs/PointCloud2` | 30Hz |

### 命令话题（下行）

| 子系统 | 话题 | 消息类型 |
|--------|------|---------|
| 底盘 | `/cmd_vel` | `geometry_msgs/Twist` |
| 机械臂 | `/{side}_arm/{side}_z1_joint_traj_controller/command` | `trajectory_msgs/JointTrajectory` |
| 云台 | `/pan_tilt_cmd_deg` | `pan_tilt_msgs/PanTiltCmdDeg` |

### 统一状态话题

| 话题 | 消息类型 | 频率 | 说明 |
|------|---------|------|------|
| `/uni_dingo/state` | `uni_dingo_msgs/RobotState` | 10Hz | 聚合所有子系统状态 |

---

## 控制链路

### Gazebo 仿真控制链路

```
MoveIt Planning → Trajectory Controller →
Gazebo Joint Controllers → Joint State Publisher →
Robot State Publisher → TF Tree
```

### 真机控制链路

```
MoveIt Planning → Trajectory Controller →
z1_hw (Hardware Interface) →
z1_controller (SDK Interface) →
Unitree SDK → Z1 Hardware
```

---

## 参考资料

- [ROS Control](http://wiki.ros.org/ros_control)
- [MoveIt](https://moveit.ros.org/)
- [Unitree Z1 SDK](https://github.com/unitreerobotics/z1_sdk)
- [Clearpath Dingo](https://docs.clearpathrobotics.com/docs/ros/robots/outdoor_robots/dingo)
- [Intel RealSense ROS](https://github.com/IntelRealSense/realsense-ros)
