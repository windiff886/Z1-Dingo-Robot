# Unidingo Description

Dingo 底盘 + 双 Z1 机械臂组合机器人的 URDF 描述包。

## 组合原理

通过 xacro 宏将各子系统组合为完整机器人：

```
Dingo 底盘
    ├── left_arm (Z1 机械臂，挂载在底盘左侧)
    │   └── left_gripper
    ├── right_arm (Z1 机械臂，挂载在底盘右侧)
    │   └── right_gripper
    └── pan_tilt (云台)
        └── d455_camera
```

## URDF 文件

| 文件 | 说明 |
|------|------|
| `urdf/uni_dingo_dual_arm.urdf.xacro` | 主模型（Dingo + 双 Z1 臂） |
| `urdf/z1_arm.urdf.xacro` | Z1 机械臂宏定义 |
| `urdf/uni_dingo.urdf` | 单体 URDF（无 xacro） |
| `urdf/d455_only.urdf.xacro` | 仅相机模型 |

## 启动文件

### 仿真（Gazebo + MoveIt）

```bash
roslaunch unidingo_description dual_arm_gazebo_moveit.launch
```

参数：
- `gui:=true` - 显示 Gazebo GUI
- `rviz:=true` - 启动 RViz
- `moveit:=true` - 启动 MoveIt

### RViz 显示

```bash
roslaunch unidingo_description dual_arm_display.launch
```

### 真机硬件接口

```bash
roslaunch unidingo_description dual_arm_hw.launch
```

## 安全位姿启动逻辑

`dual_arm_gazebo_moveit.launch` 使用 `set_dual_arm_safe_pose.py` 脚本确保安全启动：

1. **模型生成**：在 Gazebo 中 spawn URDF 模型
2. **设置初始关节配置**：通过 `/gazebo/set_model_configuration` 设置安全位姿
3. **加载控制器**：逐个加载并启动轨迹控制器
4. **确认安全位姿**：发布轨迹命令，确认机械臂到达安全位姿
5. **解除物理暂停**：只有在安全位姿确认后才 unpause Gazebo

## 控制器配置

| 文件 | 说明 |
|------|------|
| `config/left_controllers.yaml` | 左臂控制器参数 |
| `config/right_controllers.yaml` | 右臂控制器参数 |
| `config/z1_gazebo_pid.yaml` | Gazebo 仿真 PID 参数 |

## 坐标系

```
world
  └── base_link (Dingo 底盘中心)
        ├── left_base_link (左臂基座)
        │     └── left_link01 → ... → left_link06
        │           └── left_gripper_link
        ├── right_base_link (右臂基座)
        │     └── right_link01 → ... → right_link06
        │           └── right_gripper_link
        └── pan_tilt_base_link
              └── d455_link
```

## 参数

| 参数 | 说明 |
|------|------|
| `UnitreeGripper` | 是否使用 Unitree 夹爪 |
| `arm_support_radius` | 臂支撑半径 |
| `left_arm_xyz` | 左臂挂载位置 |
| `right_arm_xyz` | 右臂挂载位置 |
