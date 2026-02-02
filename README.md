# Z1-Dingo-Robot

Dingo 移动底盘 + 双 Unitree Z1 机械臂 + Pan-Tilt 云台 + Intel D455 相机的 ROS 集成工作区。

## 快速启动

### 仿真模式

```bash
source devel/setup.bash
roslaunch uni_dingo_api bringup.launch mode:=simulation gui:=true rviz:=true
```

### 真机模式

```bash
source devel/setup.bash
roslaunch uni_dingo_api bringup.launch mode:=real
```

### 键盘遥控示例

```bash
# 仿真
roslaunch whole_body_example keyboard_teleop.launch mode:=simulation
# 真机
roslaunch whole_body_example keyboard_teleop.launch mode:=real
```

## 独立组件启动

### 底盘（Dingo）

```bash
# 真机
export DINGO_OMNI=1
roslaunch dingo_base base.launch

# 仿真
roslaunch dingo_gazebo dingo_world.launch
```

### 机械臂（Z1）

```bash
# 真机（带 MoveIt）
roslaunch z1_bringup real_arm.launch rviz:=true moveit:=true

# 仿真
roslaunch z1_bringup sim_ctrl.launch
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
- [API Framework](docs/api_framework.md)

## 目录结构

```
src/
├── d455_camera/          # Intel D455 相机驱动
├── pan_tilt_ros/         # 云台控制
├── z1_ros/               # Unitree Z1 机械臂
├── dingo/                # Dingo 底盘
├── uni_dingo_api/        # 统一 API
├── uni_dingo_msgs/       # 消息定义
├── whole_body_example/   # 整机示例
└── unidingo_description/ # 组合 URDF
```
