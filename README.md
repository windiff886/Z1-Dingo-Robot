# Z1-Dingo-Robot

Dingo 全向移动底盘 + 双 Unitree Z1 机械臂 + Pan-Tilt 云台 + Intel D455 相机的 ROS 集成工作区。

系统采用组件原生 ROS 话题（Topic）接口进行解耦控制，整机启动统一由 `unidingo_bringup` 负责编排。

## 快速启动

### 仿真模式

```bash
source devel/setup.bash
# 启动 Gazebo 仿真环境与 MoveIt
roslaunch unidingo_bringup bringup.launch mode:=simulation gui:=true rviz:=true
```

### 真机模式

```bash
source devel/setup.bash
# 启动全套真机驱动与控制器
roslaunch unidingo_bringup bringup.launch mode:=real
```

### 键盘遥控示例

通过直接 ROS 话题控制底盘（`/cmd_vel`）与云台角度（真机: `/pan_tilt_cmd_rad`，仿真: 控制器 command 话题）：

```bash
# 仿真遥控
roslaunch whole_body_example keyboard_teleop.launch mode:=simulation

# 真机遥控
roslaunch whole_body_example keyboard_teleop.launch mode:=real
```

## 独立组件启动

### 底盘（Dingo）

```bash
# 真机
roslaunch dingo_control dingo.launch

# 仿真（外部 dingo_gazebo 包仍需显式选择全向底盘）
DINGO_OMNI=1 roslaunch dingo_gazebo dingo_world.launch
```

### 机械臂（Z1）

```bash
# 独立双臂仿真（topic 接口）
roslaunch z1_bringup dual_arm.launch

# 真机：完成现场参数/固件验收后
roslaunch z1_bringup dual_arm.launch backend:=udp hardware_validated:=true
```

### 云台（Pan-Tilt）

```bash
roslaunch pan_tilt_bringup panTilt_bringup.launch
```

### 相机（D455）

```bash
roslaunch d455_camera d455_camera_nodelet.launch
```

## 文档

- [项目架构](docs/architecture.md)
- [组件 Topic 接口文档](docs/component_topics.md)
- [迁移实施计划](docs/TOPIC_INTERFACE_MIGRATION_PLAN.md)

## 目录结构

```text
src/
├── unidingo_bringup/     # 整机启动编排 (仿真/真机 launch)
├── unidingo_description/ # 组合机器人 URDF 与网格模型 (ROS 包名: uni_dingo)
├── whole_body_example/   # 键盘遥控与位姿可视化示例
├── d455_camera/          # Intel D455 相机驱动 (nodelet)
├── pan_tilt_ros/         # 云台驱动与消息 (Modbus RTU)
├── z1_ros/               # Unitree Z1 机械臂
└── dingo/                # Clearpath Dingo 底盘
```
