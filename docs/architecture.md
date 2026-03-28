# Z1-Dingo-Robot 项目架构

## 项目概述

本项目是基于 **ROS2 Humble** 的移动操作机器人工作区，集成 Clearpath Dingo 底盘、双 Unitree Z1 机械臂、IQR Pan-Tilt 云台和 Intel RealSense D455 相机。

截至 **2026-03-27**，ROS2 侧已经形成以下主链：

- 统一系统入口：`uni_dingo_api/launch/bringup.launch.py`
- 仿真子入口：`simulation.launch.py`
- 真机子入口：`real.launch.py`
- 独立模型查看入口：`visualization.launch.py`
- 双臂 MoveIt2 配置：`z1_moveit_config`
- `dual_arm_gazebo_moveit` 风格 Gazebo 仿真链
- 组合模型显示 / 硬件 / world / D455 视图入口
- Pan-Tilt 与 D455 独立视图入口
- 键盘遥控与 pose visualizer 示例

当前仍未完全清零的差异：

- `bringup.launch.py --show-args` 的顶层参数展示面仍不完全等价于 ROS1
- Gazebo 与真机的运行级联调证据仍需补充
- 底盘实现来源仍是 Clearpath ROS2 官方包，而不是仓库内直接复刻 ROS1 驱动链

## 系统分层

```text
应用层
├── whole_body_example
│   ├── keyboard_teleop
│   └── pose_visualizer
└── 用户自定义 rclpy 应用

统一接口层
└── uni_dingo_api
    ├── RobotAPI
    ├── BaseAPI
    ├── ArmAPI
    ├── HeadAPI
    ├── PerceptionAPI
    └── robot_state_publisher_node

系统装配层
└── uni_dingo_api/launch
    ├── bringup.launch.py
    ├── simulation.launch.py
    ├── real.launch.py
    ├── visualization.launch.py
    └── includes/
        ├── base.launch.py
        └── moveit.launch.py

设备与描述层
├── z1_description / z1_hardware / z1_bringup / z1_moveit_config
├── pan_tilt_description / pan_tilt_driver / pan_tilt_bringup
├── d455_camera_bringup
└── unidingo_description
```

## 目录结构

```text
src/
├── uni_dingo_msgs_ros2/
│   ├── msg/
│   ├── srv/
│   └── action/
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
│   ├── urdf/
│   ├── launch/
│   └── rviz/
├── uni_dingo_api_ros2/
│   ├── config/robot_config.yaml
│   ├── launch/
│   └── uni_dingo_api/api/
└── whole_body_example_ros2/
    ├── whole_body_example/
    ├── launch/
    └── rviz/
```

## 关键包关系

```text
uni_dingo_msgs
        │
        ▼
uni_dingo_api ──────────────→ whole_body_example
   │   │   │   │   │
   │   │   │   │   └─→ d455_camera_bringup
   │   │   │   └─────→ pan_tilt_bringup / pan_tilt_description
   │   │   └─────────→ z1_moveit_config
   │   └─────────────→ z1_bringup / z1_hardware / z1_description
   └─────────────────→ unidingo_description
```

## 当前启动结构

### 统一入口

`uni_dingo_api/launch/bringup.launch.py`

- 重新恢复为 ROS1 风格总入口
- 支持 `mode:=simulation|real`
- `simulation` 与 `real` 走独立子入口
- `moveit` 当前默认开启
- 顶层保留统一装配参数：`gui/rviz/world/paused/teleop/moveit/auto_safe_pose/...`

### 仿真模式

`simulation.launch.py`

- include `unidingo_description/dual_arm_gazebo_moveit.launch.py`
- 默认 world 为 `dingo_race.world`
- 仿真链包含 Gazebo、组合模型生成、底盘控制、teleop、双臂控制器、safe pose、云台仿真控制链、MoveIt2 和统一状态发布
- `simulation` 语义已恢复为 Gazebo 仿真，而不是单纯模型查看

### 模型可视化

`visualization.launch.py`

- include `unidingo_description/dual_arm_display.launch.py`
- 可选 `joint_state_publisher_gui`
- 可选 RViz2
- 仅用于模型查看，不属于统一仿真/真机 bringup

### 真机模式

`real.launch.py`

- 既可被 `bringup.launch.py mode:=real` include，也可独立直接调用
- 承接左右臂、云台、相机的真机连接参数
- `robot_state_publisher`
- `ros2_control_node`
- 左右臂 / 夹爪 spawner
- 可选 `pan_tilt_bringup`
- 可选 `d455_camera_bringup`
- 可选外部底盘 include
- 默认 MoveIt2 include
- 默认统一状态聚合
- 机械臂网络参数已恢复为 ROS1 风格：
  `arm_ip + port_to_mcu + port_to_sdk + own_port`

## 话题与接口

### 状态上行

| 子系统 | 话题 | 类型 |
|---|---|---|
| 底盘 | `/odometry/filtered` | `nav_msgs/msg/Odometry` |
| 机械臂 / 组合模型 | `/joint_states` | `sensor_msgs/msg/JointState` |
| 云台 | `/pan_tilt_status` | `pan_tilt_msgs/msg/PanTiltStatus` |
| D455 RGB | `/d455/color/image_raw` | `sensor_msgs/msg/Image` |
| D455 深度 | `/d455/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` |
| D455 点云 | `/d455/depth/color/points` | `sensor_msgs/msg/PointCloud2` |
| 整机聚合 | `/uni_dingo/state` | `uni_dingo_msgs/msg/RobotState` |

### 命令下行

| 子系统 | 话题 | 类型 |
|---|---|---|
| 底盘 | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| 左臂 | `/left_arm_controller/follow_joint_trajectory` | FollowJointTrajectory |
| 右臂 | `/right_arm_controller/follow_joint_trajectory` | FollowJointTrajectory |
| 左夹爪 | `/left_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` |
| 右夹爪 | `/right_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` |
| 云台 | `/pan_tilt_cmd_deg` | `pan_tilt_msgs/msg/PanTiltCmdDeg` |

### 仿真特有话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/pan_tilt/pan_controller/command` | `std_msgs/msg/Float64` | 云台 yaw 位置命令 |
| `/pan_tilt/tilt_controller/command` | `std_msgs/msg/Float64` | 云台 pitch 位置命令 |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 仿真 RGB |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 仿真深度 |

## TF 结构

```text
base_footprint
└── base_link
    ├── imu_link
    ├── leftarm_link
    │   └── left_link00 → ... → left_link06
    ├── rightarm_link
    │   └── right_link00 → ... → right_link06
    └── pan_tilt_mount_link
        └── pan_tilt_base_link → pan_tilt_yaw_link → pan_tilt_pitch_link
            └── pan_tilt_surface → camera_support_link
                └── head_d455_link → head_d455_optical_frame
```

## 仿真 / 真机切换

| 特性 | `simulation` | `real` |
|---|---|---|
| 系统语义 | Gazebo 仿真总入口 | 真机 bringup |
| 双臂规划 | 默认 MoveIt2 | 默认 MoveIt2 |
| 云台控制 | Gazebo 云台控制链 | `pan_tilt_driver` |
| 相机 | Gazebo 组合模型中的相机结构 | `d455_camera_bringup` |
| 模型查看 | 改由 `visualization.launch.py` | 改由 `visualization.launch.py` |
| 底盘 | Gazebo 中生成组合模型 | 外部 Clearpath ROS2 包 |

## 参考文档

- 当前严格对齐状态：`docs/ros2_strict_alignment_status.md`
- 完全对齐实施计划：`docs/superpowers/plans/2026-03-27-ros2-full-alignment-plan.md`
