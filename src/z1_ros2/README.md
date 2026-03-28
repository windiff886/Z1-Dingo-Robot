# z1_ros2

Unitree Z1 六自由度机械臂的 ROS2 完整驱动栈，包含 URDF 模型、ros2_control 硬件接口、底层 FSM 控制器、MoveIt2 运动规划配置、示例代码和 RViz 可视化工具共 **9 个子包**。

## 架构概览

```
z1_ros2/
├── z1_description           # URDF/xacro 模型 + ros2_control 配置
├── z1_hardware              # ros2_control 硬件接口（真机 SDK 通信）
├── z1_controller_ros2       # 底层 FSM 控制器（UDP 直控, 示教, 标定等）
├── z1_bringup               # Launch 入口文件
├── z1_moveit_config         # MoveIt2 运动规划配置（双臂）
├── z1_dual_moveit_config    # MoveIt2 双臂独立配置包
├── z1_examples              # C++ 示例代码
├── z1_rviz                  # RViz2 可视化 launch
└── doc/                     # 文档资料
```

### 控制栈层次

```
                        ┌───────────────────────────────┐
                        │       MoveIt2 Move Group      │ ◄─ 运动规划
                        │  (z1_moveit_config /           │
                        │   z1_dual_moveit_config)       │
                        └──────────┬────────────────────┘
                                   │ JointTrajectory
                                   ▼
                        ┌───────────────────────────────┐
                        │   ros2_control Manager        │ ◄─ 控制器管理
                        │   joint_trajectory_controller │
                        │   gripper_action_controller   │
                        └──────────┬────────────────────┘
                                   │ Command/State
                    ┌──────────────┼──────────────┐
                    ▼                              ▼
          ┌──────────────────┐          ┌──────────────────┐
          │ z1_hardware      │          │ GazeboSystem     │
          │ Z1System         │          │ (仿真模式)       │
          │ (真机, UDP SDK)  │          │                  │
          └───────┬──────────┘          └──────────────────┘
                  │ UDP
                  ▼
      ┌──────────────────────┐
      │ Unitree Z1 MCU       │ ◄─ 硬件
      │ (192.168.123.x)      │
      └──────────────────────┘
```

**可选底层路径（绕过 ros2_control）：**

```
z1_controller_ros2           # 直接 FSM 控制（UDP 到 MCU）
├── z1_ctrl_node             # 主控制节点
│   ├── FSM (有限状态机)
│   │   ├── State_Init        # 初始化
│   │   ├── State_Passive     # 被动模式
│   │   ├── State_JointCtrl   # 关节位置控制
│   │   ├── State_LowCmd      # 底层命令
│   │   ├── State_Calibration # 校准
│   │   ├── State_Teach       # 示教
│   │   └── State_TeachRepeat # 示教回放
│   └── IO 接口
│       ├── IOUDP   (真机 UDP)
│       └── IOROS2  (ROS2 话题桥接)
└── io_ros2.cpp              # ROS2 话题 <-> FSM 桥接层
```

## 子包详解

### z1_description

URDF/Xacro 模型与 ros2_control 控制器配置。

**关键文件：**

| 文件 | 说明 |
|------|------|
| `urdf/z1.urdf.xacro` | Z1 单臂 URDF 模型（参数化 prefix） |
| `urdf/z1.ros2_control.xacro` | ros2_control 硬件接口声明 |
| `urdf/z1_standalone.urdf.xacro` | 独立单臂模型 |
| `urdf/z1_dual_standalone.urdf.xacro` | 独立双臂模型 |
| `urdf/const.xacro` | 常量定义（关节限位等） |
| `config/z1_controllers.yaml` | 单臂控制器参数 |
| `config/z1_dual_controllers.yaml` | 双臂控制器参数 |

**Z1 机械臂关节定义（6-DOF + Gripper）：**

| 关节 | 名称模式 | 类型 | 说明 |
|------|----------|------|------|
| Joint 1 | `{prefix}joint1` | revolute | 基座旋转 |
| Joint 2 | `{prefix}joint2` | revolute | 肩部 |
| Joint 3 | `{prefix}joint3` | revolute | 肘部 |
| Joint 4 | `{prefix}joint4` | revolute | 腕部旋转 |
| Joint 5 | `{prefix}joint5` | revolute | 腕部俯仰 |
| Joint 6 | `{prefix}joint6` | revolute | 法兰旋转 |
| Gripper | `{prefix}jointGripper` | prismatic | 夹爪 |

> `prefix` 在组合模型中为 `left_` 或 `right_`。

### z1_hardware

实现 `hardware_interface::SystemInterface` 的 ros2_control 硬件插件，通过 Unitree ARM SDK (UDP) 与 Z1 MCU 通信。

**核心类：`z1_hardware::Z1System`**

| 生命周期 | 功能 |
|----------|------|
| `on_init` | 解析硬件参数（IP、端口、夹爪配置）|
| `on_configure` | 建立 UDP 连接 |
| `on_activate` | 读取当前关节位置，同步命令（防跳变）|
| `on_deactivate` | 切换到 Passive 模式 |
| `read()` | 读取 6 关节 + 夹爪状态（q, dq, tau）|
| `write()` | 发送关节位置命令（JointPositionCtrl 模式）|

**硬件参数：**

| 参数 | 类型 | 说明 |
|------|------|------|
| `mcu_ip` | string | Z1 MCU IP 地址 |
| `port_to_mcu` | int | MCU 通信端口 |
| `port_to_sdk` | int | SDK 中继端口 |
| `controller_ip` | string | 本地 IP |
| `own_port` | int | 本地绑定端口 |
| `has_gripper` | bool | 是否启用夹爪 |
| `gripper_max_tau` | double | 夹爪最大力矩 (N·m, 默认 10) |
| `gripper_epsilon` | double | 夹爪角度容差 (rad, 默认 0.01) |

**第三方依赖：** `unitree_arm_sdk`（包含 SDK 头文件和静态库）

### z1_controller_ros2

Z1 机械臂的底层 FSM（有限状态机）控制器，为独立控制节点，可在不使用 ros2_control 的场景下直接 UDP 控制机械臂。

**FSM 状态：**

| 状态 | 说明 |
|------|------|
| `State_Init` | 初始化，读取编码器 |
| `State_Passive` | 被动模式（电机断力） |
| `State_JointCtrl` | 关节位置控制 |
| `State_LowCmd` | 底层命令模式（q, dq, tau, Kp, Kd） |
| `State_Calibration` | 关节校准模式 |
| `State_Teach` | 示教模式（拖拽记录） |
| `State_TeachRepeat` | 示教回放 |
| `State_ClearError` | 清除错误 |

**IO 接口层 (`io_ros2`)：**

- 订阅 `uni_dingo_msgs/MotorCmd` 接收控制命令
- 发布 `uni_dingo_msgs/MotorState` 反馈状态
- 发布 `sensor_msgs/JointState` 用于 TF

### z1_bringup

Launch 入口文件，负责加载硬件接口、启动控制器管理器和激活控制器。

```bash
# 双臂硬件启动
ros2 launch z1_bringup z1_dual_bringup.launch.py \
  left_arm_ip:=192.168.123.110 \
  left_port_to_mcu:=8881 \
  left_port_to_sdk:=8871 \
  left_own_port:=8872 \
  right_arm_ip:=192.168.123.111 \
  right_port_to_mcu:=8882 \
  right_port_to_sdk:=8873 \
  right_own_port:=8874
```

### z1_moveit_config

MoveIt2 运动规划配置（基于 `SetupAssistant` 生成后定制）。

**配置文件：**

| 文件 | 说明 |
|------|------|
| `config/joint_limits.yaml` | 关节速度/加速度限制 |
| `config/kinematics.yaml` | 运动学求解器配置（KDL） |
| `config/moveit_controllers.yaml` | MoveIt 控制器映射 |
| `config/ompl_planning.yaml` | OMPL 规划器配置 |

**Launch 文件：**

```bash
# 启动 move_group 规划节点
ros2 launch z1_moveit_config move_group.launch.py

# 启动 MoveIt RViz 交互界面
ros2 launch z1_moveit_config moveit_rviz.launch.py
```

### z1_dual_moveit_config

双臂独立 MoveIt2 配置包，定义了 `left_arm_manipulator` 和 `right_arm_manipulator` 两个 Planning Group。

### z1_examples

C++ 示例代码：

| 文件 | 说明 |
|------|------|
| `move_group_interface.cpp` | 使用 MoveIt C++ API 进行运动规划 |
| `ros_gripper_ctrl.cpp` | 夹爪 GripperCommand Action 控制示例 |

### z1_rviz

RViz2 可视化 launch 文件：

```bash
# 查看 Z1 模型（带 GUI 滑块）
ros2 launch z1_rviz view_model.launch.py

# 查看真机状态
ros2 launch z1_rviz view_robot.launch.py
```

## 网络拓扑

```
┌─────────────┐     UDP 8871/8872      ┌──────────────┐
│ 控制 PC      │◄────────────────────▶│ z1_ctrl_ros2  │
│ (ROS2 节点) │                       │ (桥接节点)    │
└─────┬───────┘                       └───────┬──────┘
      │                                       │ UDP 8881
      │ UDP (z1_hardware)                     ▼
      │                              ┌──────────────┐
      └─────────────────────────────▶│ Z1 MCU       │
                                     │ 192.168.123.x │
                                     └──────────────┘
```

**双臂默认端口映射：**

| 参数 | 左臂 | 右臂 |
|------|------|------|
| `arm_ip` | `192.168.123.110` | `192.168.123.111` |
| `port_to_mcu` | `8881` | `8882` |
| `port_to_sdk` | `8871` | `8873` |
| `own_port` | `8872` | `8874` |

## 使用方法

### 仿真模式（推荐通过统一入口）

```bash
ros2 launch uni_dingo_api bringup.launch.py mode:=simulation
```

### 真机模式

```bash
# 方式 1: 通过统一入口
ros2 launch uni_dingo_api bringup.launch.py mode:=real \
  left_arm_ip:=192.168.123.110 \
  right_arm_ip:=192.168.123.111

# 方式 2: 独立启动双臂
ros2 launch z1_bringup z1_dual_bringup.launch.py \
  left_arm_ip:=192.168.123.110 \
  right_arm_ip:=192.168.123.111

# 然后启动 MoveIt2
ros2 launch z1_moveit_config move_group.launch.py
ros2 launch z1_moveit_config moveit_rviz.launch.py
```

### 查看模型

```bash
ros2 launch z1_rviz view_model.launch.py
```

## 依赖

- **核心**: `rclcpp`, `hardware_interface`, `controller_manager`, `pluginlib`
- **MoveIt2**: `moveit_ros_planning`, `moveit_ros_move_group`, `moveit_kinematics`
- **Gazebo**: `gazebo_ros2_control`
- **SDK**: `unitree_arm_sdk`（第三方，包含于 `z1_hardware/third_party/`）
- **消息**: `sensor_msgs`, `control_msgs`, `trajectory_msgs`, `uni_dingo_msgs`
