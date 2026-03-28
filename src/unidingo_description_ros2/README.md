# unidingo_description_ros2 (unidingo_description)

Uni-Dingo 机器人的**统一 URDF/Xacro 模型描述包**，将 Dingo 全向底盘、双 Z1 机械臂、IQR 云台和 D455 深度相机组装为完整的机器人模型。同时提供 Gazebo 仿真所需的 `ros2_control` 硬件接口配置和控制器参数。

## 架构概览

```
                        uni_dingo_dual_arm.urdf.xacro
                                   │
           ┌───────────────────────┼───────────────────────┐
           ▼                       ▼                       ▼
    dingo_base.xacro        z1.urdf.xacro          pan_tilt.urdf.xacro
    (底盘模型 +              (Z1 机械臂 ×2          (云台 + D455 相机
     全向轮 + IMU)            left/right prefix)     支撑柱 + 光学坐标系)
           │                       │
           ▼                       ▼
  dingo_base.ros2_control    z1.ros2_control.xacro
  .xacro (底盘 HW 接口)      (臂 HW 接口,
                              仿真/真机可切换)
```

### 组合 URDF 层次

```
base_link (Dingo 底盘)
├── front_left_wheel / front_right_wheel / rear_left_wheel / rear_right_wheel
├── base_leftarm_joint (fixed) → leftarm_link (支撑柱)
│   └── left_joint1 → ... → left_link06 → left_gripper
├── base_rightarm_joint (fixed) → rightarm_link (支撑柱)
│   └── right_joint1 → ... → right_link06 → right_gripper
└── base_pan_tilt_mount_joint (fixed) → pan_tilt_mount_link (支撑柱)
    └── pan_tilt_yaw_joint → pan_tilt_pitch_joint → pan_tilt_surface
        └── camera_support_link → head_d455_link → head_d455_optical_frame
```

### 可配置参数

`uni_dingo_dual_arm.urdf.xacro` 支持以下 xacro 参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `has_gripper` | `true` | 是否包含夹爪 |
| `pan_tilt` | `true` | 是否包含云台+D455 |
| `left_arm` | `true` | 是否包含左臂 |
| `right_arm` | `true` | 是否包含右臂 |
| `sim_gazebo` | `false` | 是否为 Gazebo 仿真（切换 HW 接口） |
| `left_arm_ip` | `192.168.123.110` | 左臂 MCU IP（真机模式） |
| `left_port_to_mcu` | `8881` | 左臂 MCU 通信端口 |
| `left_port_to_sdk` | `8871` | 左臂 SDK 通信端口 |
| `left_own_port` | `8872` | 左臂本地绑定端口 |
| `right_arm_ip` | `192.168.123.111` | 右臂 MCU IP |
| `right_port_to_mcu` | `8882` | 右臂 MCU 通信端口 |
| `right_port_to_sdk` | `8873` | 右臂 SDK 通信端口 |
| `right_own_port` | `8874` | 右臂本地绑定端口 |

## 目录结构

```
unidingo_description_ros2/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   ├── uni_dingo_dual_arm.urdf.xacro   # 主组合模型文件
│   ├── dingo_base.xacro                # Dingo 底盘模型
│   ├── dingo_base.ros2_control.xacro   # 底盘 ros2_control 接口
│   └── d455_only.urdf.xacro            # 独立 D455 模型
├── config/
│   ├── gazebo_simulation_controllers.yaml  # Gazebo ros2_control 总配置
│   ├── base_simulation_controllers.yaml    # 底盘控制器配置
│   ├── arm_simulation_controllers.yaml     # 双臂关节轨迹控制器
│   ├── pan_tilt_controllers.yaml           # 云台位置控制器
│   ├── gains_omni.yaml                     # 底盘全向 PID 增益
│   └── z1_gazebo_pid.yaml                  # Z1 关节 PID 增益
├── meshes/                                 # 3D 模型文件（STL/DAE）
├── rviz/                                   # RViz 预设配置
├── scripts/
│   └── set_dual_arm_safe_pose.py           # 启动后双臂归安全位姿脚本
├── launch/
│   ├── display.launch.py                   # 基础模型展示
│   ├── dual_arm_display.launch.py          # 双臂模型展示（带 GUI 滑块）
│   ├── dual_arm_gazebo_moveit.launch.py    # Gazebo + MoveIt2 完整仿真
│   ├── dual_arm_hw.launch.py               # 真机 URDF 加载（robot_state_publisher）
│   ├── dual_arm_world.launch.py            # Gazebo 仿真世界 + 模型 Spawn
│   ├── sim_base_control.launch.py          # 仿真底盘控制（控制器激活）
│   ├── sim_teleop.launch.py                # 仿真遥控（keyboard/joystick）
│   └── view_d455_only.launch.py            # 仅查看 D455 模型
└── test/
    └── test_launch_entrypoints.py
```

## Launch 文件说明

### dual_arm_display.launch.py

在 RViz 中查看完整双臂模型，带 `joint_state_publisher_gui` 滑块。

```bash
ros2 launch unidingo_description dual_arm_display.launch.py
```

### dual_arm_world.launch.py

启动 Gazebo 仿真并将完整机器人 Spawn 到仿真世界中。

```bash
ros2 launch unidingo_description dual_arm_world.launch.py \
  gui:=true \
  world:=/path/to/dingo_race.world \
  has_gripper:=true
```

**参数：** `use_sim_time`, `gui`, `world`, `paused`, `x`, `y`, `z`, `yaw`, `has_gripper`, `pan_tilt`

### dual_arm_hw.launch.py

真机模式下加载 URDF 并启动 `robot_state_publisher`（不启动 Gazebo）。

```bash
ros2 launch unidingo_description dual_arm_hw.launch.py
```

### dual_arm_gazebo_moveit.launch.py

完整的 Gazebo + 控制器 + MoveIt2 仿真 launch。

### sim_base_control.launch.py

在仿真中激活底盘控制器和里程计发布。

### sim_teleop.launch.py

启动遥控节点（键盘或手柄）。

### view_d455_only.launch.py

仅查看 D455 相机的 URDF 模型。

```bash
ros2 launch unidingo_description view_d455_only.launch.py
```

## 安装位姿

双臂和云台的安装位置参数（相对于 `base_link`）：

| 组件 | X (m) | Y (m) | Z (m) |
|------|-------|-------|-------|
| 左臂 | 0.151 | 0.117 | 0.572 |
| 右臂 | 0.151 | -0.117 | 0.572 |
| 云台 | 0.151 | 0 | 1.000 |

## ros2_control 架构

### 仿真模式

- 硬件接口：`gazebo_ros2_control/GazeboSystem`
- 底盘：`omni_drive_controller`
- 双臂：`joint_trajectory_controller` × 2
- 夹爪：`gripper_action_controller` × 2
- 云台：`position_controllers/JointGroupPositionController`

### 真机模式

- 底盘：`Clearpath` 官方 `ros2_control` 接口
- 双臂：`z1_hardware/Z1System`（UDP SDK 通信）
- 云台：`pan_tilt_driver_node`（Modbus RTU 串口）

## 依赖

- `ament_cmake`, `xacro`
- `robot_state_publisher`, `joint_state_publisher(_gui)`
- `rviz2`
- `gazebo_ros`, `gazebo_ros2_control`
- `controller_manager`, `joint_state_broadcaster`, `joint_trajectory_controller`
- `position_controllers`, `forward_command_controller`
- `omni_drive_controller`, `robot_localization`
- `z1_description`, `pan_tilt_description`
