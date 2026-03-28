# uni_dingo_msgs_ros2 (uni_dingo_msgs)

Uni-Dingo 机器人系统的**自定义 ROS2 接口定义包**，定义了整个系统中所有子系统通信所需的 Message、Service 和 Action 接口。

## 架构概览

本包不包含可执行节点，仅通过 `rosidl_generate_interfaces` 在编译时生成 C++ 和 Python 的类型绑定，供其他包导入使用。

```
uni_dingo_msgs/
├── msg/              # 状态消息
│   ├── RobotState.msg       # 全机器人状态（聚合）
│   ├── BaseState.msg        # 底盘状态
│   ├── ArmState.msg         # 机械臂状态
│   ├── HeadState.msg        # 云台状态
│   ├── GripperState.msg     # 夹爪状态
│   ├── SubsystemStatus.msg  # 子系统健康状态
│   ├── MotorCmd.msg         # 电机底层命令（对齐 ROS1）
│   └── MotorState.msg       # 电机底层状态（对齐 ROS1）
├── srv/              # 服务接口
│   ├── GetRobotState.srv    # 查询机器人状态
│   ├── SetEnabled.srv       # 启用/禁用子系统
│   └── EmergencyStop.srv    # 急停控制
├── action/           # 动作接口
│   ├── MoveArm.action       # 机械臂运动（位姿/关节/命名目标）
│   ├── MoveBase.action      # 底盘导航
│   ├── MoveHead.action      # 云台运动
│   ├── MoveGripper.action   # 夹爪控制
│   └── WholeBodyMove.action # 全身协调运动
├── CMakeLists.txt
└── package.xml
```

## 消息定义详解

### 状态消息 (msg/)

#### RobotState.msg — 全机器人状态

```
std_msgs/Header header
string mode                              # "simulation" | "real"
uni_dingo_msgs/BaseState base
uni_dingo_msgs/ArmState left_arm
uni_dingo_msgs/ArmState right_arm
uni_dingo_msgs/HeadState head
uni_dingo_msgs/SubsystemStatus[] subsystem_statuses
bool emergency_stopped
bool all_ready
builtin_interfaces/Time last_update
```

#### BaseState.msg — 底盘状态

```
std_msgs/Header header
geometry_msgs/Pose pose               # 当前位姿（odom 坐标系）
geometry_msgs/Twist velocity           # 当前速度
bool enabled / moving / emergency_stopped
float32 battery_voltage / battery_percentage
```

#### ArmState.msg — 机械臂状态

```
std_msgs/Header header
string side                             # "left" | "right"
string[] joint_names
float64[] joint_positions / joint_velocities / joint_efforts
geometry_msgs/PoseStamped end_effector_pose
uni_dingo_msgs/GripperState gripper
bool enabled / moving / in_collision / at_limit
string current_named_pose               # "home", "ready", "tucked", ""
```

#### HeadState.msg — 云台状态

```
std_msgs/Header header
float64 pan / tilt                      # 当前角度（弧度）
float64 pan_velocity / tilt_velocity
float64 pan_min / pan_max / tilt_min / tilt_max   # 限位
bool enabled / moving
```

#### GripperState.msg — 夹爪状态

```
float64 position / velocity / effort
bool is_grasping                        # 是否正在抓取
bool stalled                            # 是否堵转
```

#### SubsystemStatus.msg — 子系统健康状态

```
# 状态枚举
uint8 STATUS_UNKNOWN = 0
uint8 STATUS_OK = 1
uint8 STATUS_WARNING = 2
uint8 STATUS_ERROR = 3
uint8 STATUS_DISABLED = 4

string name                             # 子系统名称
uint8 status
bool enabled / ready
string message                          # 可读状态描述
```

#### MotorCmd.msg — 电机命令（ROS1 兼容）

```
uint8 mode            # 电机目标模式
float32 q             # 目标位置 (rad)
float32 dq            # 目标速度 (rad/s)
float32 tau           # 目标力矩 (N*m)
float32 kp            # 弹簧刚度系数
float32 kd            # 阻尼系数
```

> 对齐 ROS1 `unitree_legged_msgs/MotorCmd`，用于 Z1 机械臂仿真 IO 通道。

#### MotorState.msg — 电机状态（ROS1 兼容）

```
uint8 mode            # 当前模式
float32 q / dq / ddq  # 位置 / 速度 / 加速度
float32 tau_est       # 估计输出力矩 (N*m)
int8 temperature      # 电机温度
```

### 服务接口 (srv/)

#### GetRobotState.srv

```
---
uni_dingo_msgs/RobotState state
bool success
string message
```

#### SetEnabled.srv

```
string subsystem      # "base" | "left_arm" | "right_arm" | "head" | "all"
bool enabled
---
bool success
string message
```

#### EmergencyStop.srv

```
bool activate          # true=激活急停, false=释放
---
bool success
string message
```

### 动作接口 (action/)

#### MoveArm.action

```
# Goal
uint8 MOVE_TO_POSE = 0
uint8 MOVE_TO_JOINTS = 1
uint8 MOVE_TO_NAMED = 2
uint8 move_type
geometry_msgs/PoseStamped target_pose    # MOVE_TO_POSE
float64[] target_joints                  # MOVE_TO_JOINTS
string named_target                      # MOVE_TO_NAMED
string planning_group
float64 velocity_scaling / acceleration_scaling
bool allow_replanning

# Result
bool success / string message
geometry_msgs/PoseStamped final_pose / float64[] final_joints

# Feedback
float64 progress (0~1) / string status / geometry_msgs/PoseStamped current_pose
```

#### MoveBase.action

```
# Goal
geometry_msgs/Pose2D target_pose / string frame_id
float64 linear_tolerance / angular_tolerance

# Result
bool success / geometry_msgs/Pose2D final_pose

# Feedback
geometry_msgs/Pose2D current_pose / float64 distance_remaining / progress
```

#### MoveHead.action

```
# Goal
uint8 SET_ANGLES = 0 / LOOK_AT = 1 / HOME = 2
float64 pan / tilt                       # SET_ANGLES
geometry_msgs/PointStamped look_at_point # LOOK_AT

# Result & Feedback
float64 final_pan / final_tilt / current_pan / current_tilt / progress
```

#### MoveGripper.action

```
# Goal
uint8 OPEN = 0 / CLOSE = 1 / SET_POSITION = 2
float64 position / max_effort

# Result
float64 final_position / bool object_detected

# Feedback
float64 current_position / current_effort
```

#### WholeBodyMove.action

```
# Goal（各段可选启用）
bool move_base / move_left_arm / move_right_arm / move_head
geometry_msgs/Pose2D base_target
geometry_msgs/PoseStamped left_arm_target / right_arm_target
geometry_msgs/PointStamped head_look_at
float64 velocity_scaling

# Feedback
float64 overall_progress / base/left_arm/right_arm/head_progress
string current_phase ("base" | "arms" | "head" | "done")
```

## 使用方法

### 在 Python 中导入

```python
from uni_dingo_msgs.msg import RobotState, ArmState, MotorCmd
from uni_dingo_msgs.srv import GetRobotState, EmergencyStop
from uni_dingo_msgs.action import MoveArm, WholeBodyMove
```

### 在 C++ 中导入

```cpp
#include "uni_dingo_msgs/msg/robot_state.hpp"
#include "uni_dingo_msgs/srv/emergency_stop.hpp"
#include "uni_dingo_msgs/action/move_arm.hpp"
```

### 在 package.xml 中声明依赖

```xml
<depend>uni_dingo_msgs</depend>
```

## 编译依赖

- `ament_cmake`
- `rosidl_default_generators` / `rosidl_default_runtime`
- `std_msgs`, `geometry_msgs`, `builtin_interfaces`, `action_msgs`
