# Z1-Dingo-Robot 项目架构

## 项目概述

本项目是基于 **ROS2 Humble** 的移动操作机器人工作区，集成 Clearpath Dingo 全向底盘、双 Unitree Z1 六自由度机械臂、IQR Pan-Tilt 云台和 Intel RealSense D455 深度相机。支持仿真和真机两种模式。

**主要能力：**
- Dingo 全向/差速移动底盘控制
- 双 Z1 机械臂独立或协同控制（via MoveIt2）
- Pan-Tilt 云台 look-at 控制
- D455 RGB-D 感知与点云
- 全身键盘遥控

---

## 系统架构

```
┌──────────────────────────────────────────────────────────────────┐
│                         应用层 (Application)                      │
│           whole_body_example (键盘遥控, 用户脚本)                  │
└──────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌──────────────────────────────────────────────────────────────────┐
│                      统一接口层 (uni_dingo_api)                    │
│  ┌──────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌──────────┐ │
│  │ RobotAPI │→│ BaseAPI │→│ ArmAPI  │→│ HeadAPI │→│Perception│ │
│  └──────────┘ └─────────┘ └─────────┘ └─────────┘ └──────────┘ │
│                        ActionFuture 异步模型                      │
└──────────────────────────────────────────────────────────────────┘
                                  │
                    ┌─────────────┼──────────────┐
                    ▼             ▼              ▼
┌──────────────────────────────────────────────────────────────────┐
│                    ROS2 中间件 (DDS + ros2_control)                │
│                                                                   │
│  仿真模式:                         真机模式:                      │
│  ├─ joint_state_publisher_gui      ├─ z1_hardware (ros2_control)  │
│  └─ (URDF 可视化)                  ├─ pan_tilt_driver (rclcpp)    │
│                                    ├─ realsense2_camera           │
│                                    └─ clearpath_robot (底盘)      │
└──────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌──────────────────────────────────────────────────────────────────┐
│                          物理硬件 / 仿真器                        │
│     Dingo 底盘        Z1 机械臂 ×2       云台        D455 相机    │
│   (CAN/Serial)       (UDP/z1_sdk)    (Modbus RTU)  (USB3/UVC)   │
└──────────────────────────────────────────────────────────────────┘
```

---

## 目录结构

```
Z1-Dingo-Robot/
├── src/
│   ├── uni_dingo_msgs_ros2/           # 自定义消息/服务/动作
│   │   ├── msg/                       #   RobotState, ArmState, BaseState, ...
│   │   ├── srv/                       #   GetRobotState, SetEnabled, EmergencyStop
│   │   └── action/                    #   MoveArm, MoveBase, MoveHead, ...
│   │
│   ├── pan_tilt_ros2/
│   │   ├── pan_tilt_msgs/             # PanTiltCmdDeg, PanTiltStatus
│   │   ├── pan_tilt_driver/           # Modbus RTU 驱动 (rclcpp)
│   │   ├── pan_tilt_description/      # URDF + STL 网格
│   │   └── pan_tilt_bringup/          # 启动文件
│   │
│   ├── d455_camera_ros2/              # realsense2_camera 启动封装
│   │
│   ├── z1_ros2/
│   │   ├── z1_description/            # Z1 URDF xacro (带 prefix 支持)
│   │   │   ├── urdf/
│   │   │   │   ├── z1.urdf.xacro      #   可复用臂宏 (prefix, parent, xyz, rpy)
│   │   │   │   ├── z1.ros2_control.xacro  # ros2_control 硬件标签
│   │   │   │   └── const.xacro        #   物理参数 (惯性、关节限位)
│   │   │   └── config/
│   │   │       ├── z1_controllers.yaml       # 单臂控制器
│   │   │       └── z1_dual_controllers.yaml  # 双臂控制器
│   │   ├── z1_hardware/               # ros2_control SystemInterface
│   │   │   ├── src/z1_system.cpp      #   通过 z1_sdk UDP 读写关节
│   │   │   └── third_party/           #   unitree_arm_sdk 头文件 + .so
│   │   └── z1_bringup/               # 单臂/双臂启动文件
│   │
│   ├── unidingo_description_ros2/     # 组合机器人 URDF
│   │   └── urdf/
│   │       ├── dingo_base.xacro       #   简化 Dingo 底盘
│   │       └── uni_dingo_dual_arm.urdf.xacro  # 全系统组装
│   │
│   ├── uni_dingo_api_ros2/            # 统一 Python API
│   │   ├── uni_dingo_api/
│   │   │   ├── api/
│   │   │   │   ├── robot_api.py       #   RobotAPI 入口
│   │   │   │   ├── base_api.py        #   底盘 cmd_vel / odom
│   │   │   │   ├── arm_api.py         #   MoveIt2 运动规划 + 夹爪
│   │   │   │   ├── head_api.py        #   云台角度控制
│   │   │   │   ├── perception_api.py  #   RGB-D + 点云 + TF
│   │   │   │   └── action_future.py   #   异步操作封装
│   │   │   └── robot_state_publisher_node.py  # 状态聚合节点
│   │   ├── config/robot_config.yaml   # 子系统配置
│   │   └── launch/bringup.launch.py   # 全系统启动
│   │
│   └── whole_body_example_ros2/       # 示例
│       └── whole_body_example/
│           └── keyboard_teleop.py     # 键盘遥控
│
├── docs/
│   ├── architecture.md                # 本文档
│   └── api_framework.md               # API 接口说明
│
├── .gitignore
└── README.md
```

---

## ROS2 包依赖关系

```
uni_dingo_msgs ─────────────────────────────────┐
pan_tilt_msgs ───────────────────────────┐      │
                                         │      │
z1_description ──→ z1_hardware           │      │
                   z1_bringup            │      │
pan_tilt_description ──→ pan_tilt_driver │      │
                         pan_tilt_bringup│      │
d455_camera_bringup                      │      │
unidingo_description (uses z1_description│      │
  + pan_tilt_description)                │      │
                                         ▼      ▼
                                    uni_dingo_api
                                         │
                                         ▼
                                  whole_body_example
```

---

## 话题列表

### 传感器话题（上行）

| 子系统 | 话题 | 消息类型 | 频率 |
|--------|------|----------|------|
| 底盘 | `/odometry/filtered` | `nav_msgs/msg/Odometry` | 50 Hz |
| 机械臂 | `/joint_states` | `sensor_msgs/msg/JointState` | 100 Hz |
| 云台 | `/pan_tilt_status` | `pan_tilt_msgs/msg/PanTiltStatus` | 20 Hz |
| 相机 | `/d455/color/image_raw` | `sensor_msgs/msg/Image` | 30 Hz |
| 相机 | `/d455/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | 30 Hz |
| 相机 | `/d455/depth/color/points` | `sensor_msgs/msg/PointCloud2` | 15 Hz |
| 聚合 | `/uni_dingo/state` | `uni_dingo_msgs/msg/RobotState` | 10 Hz |

### 命令话题（下行）

| 子系统 | 话题 | 消息类型 |
|--------|------|----------|
| 底盘 | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| 左臂轨迹 | `/left_arm_controller/joint_trajectory` | JointTrajectoryController |
| 右臂轨迹 | `/right_arm_controller/joint_trajectory` | JointTrajectoryController |
| 左夹爪 | `/left_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` |
| 右夹爪 | `/right_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` |
| 云台 | `/pan_tilt_cmd_deg` | `pan_tilt_msgs/msg/PanTiltCmdDeg` |

### 仿真模式专用话题

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/pan_tilt/pan_controller/command` | `std_msgs/msg/Float64` | 云台 yaw 位置命令 |
| `/pan_tilt/tilt_controller/command` | `std_msgs/msg/Float64` | 云台 pitch 位置命令 |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 仿真 RGB |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 仿真深度 |

---

## ros2_control 架构

### 硬件接口

| 组件 | 插件 | 通信方式 |
|------|------|----------|
| 双 Z1 臂 | `z1_hardware/Z1System` | UDP (z1_sdk) |
| Dingo 底盘 | Clearpath 官方 | CAN / Serial |

### 控制器配置 (双臂)

```yaml
controller_manager:
  joint_state_broadcaster    # 发布 /joint_states
  left_arm_controller        # JointTrajectoryController (left_joint1~6)
  right_arm_controller       # JointTrajectoryController (right_joint1~6)
  left_gripper_controller    # GripperActionController
  right_gripper_controller   # GripperActionController
```

控制器启动顺序：`joint_state_broadcaster` → 两臂 arm_controller (并行) → 两侧 gripper_controller。

---

## TF 树

```
base_footprint
└── base_link
    ├── imu_link
    ├── left_arm_support_link
    │   └── left_link00 → left_link01 → ... → left_link06
    │       └── left_gripperStator → left_gripperMover
    ├── right_arm_support_link
    │   └── right_link00 → right_link01 → ... → right_link06
    │       └── right_gripperStator → right_gripperMover
    └── pan_tilt_support_link
        └── pan_tilt_base_link → pan_tilt_yaw_link → pan_tilt_pitch_link
            └── pan_tilt_surface → camera_support_link
                └── camera_link → camera_color_optical_frame
```

---

## 仿真 vs 真机切换

通过 `mode` 参数和 `robot_config.yaml` 控制：

| 特性 | 仿真模式 (`simulation`) | 真机模式 (`real`) |
|------|------------------------|------------------|
| 关节状态 | `joint_state_publisher_gui` | `ros2_control` + `z1_hardware` |
| 机械臂规划 | MoveIt2 (需额外配置) | MoveIt2 + z1_sdk |
| 云台控制 | Float64 位置命令 | Modbus RTU (`pan_tilt_driver`) |
| 相机 | 无 (或 Gazebo 插件) | `realsense2_camera` |
| 底盘 | 无 (或 Gazebo) | Clearpath ROS2 driver |

---

## 待完成项

- [ ] Dingo 底盘 ROS2 集成（安装 `clearpath_common` + 配置 `robot.yaml`）
- [ ] MoveIt2 双臂 SRDF + 规划组配置
- [ ] Gazebo Classic / Ignition 仿真集成
- [ ] Nav2 导航集成
- [ ] 真机全系统联调测试
