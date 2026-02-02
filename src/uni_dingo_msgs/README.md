# Uni-Dingo Messages

统一机器人消息定义包，包含状态消息、服务和动作定义。

## 消息 (msg)

### RobotState.msg

整机状态，聚合所有子系统。

```
std_msgs/Header header
string mode                          # "simulation" | "real"
uni_dingo_msgs/BaseState base
uni_dingo_msgs/ArmState left_arm
uni_dingo_msgs/ArmState right_arm
uni_dingo_msgs/HeadState head
uni_dingo_msgs/SubsystemStatus[] subsystem_statuses
bool emergency_stopped
bool all_ready
time last_update
```

### BaseState.msg

底盘状态。

```
std_msgs/Header header
bool enabled
bool moving
bool emergency_stopped
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
```

### ArmState.msg

机械臂状态。

```
std_msgs/Header header
string side                          # "left" | "right"
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

### HeadState.msg

云台状态。

```
std_msgs/Header header
bool enabled
bool moving
float64 pan
float64 tilt
float64 pan_velocity
float64 tilt_velocity
```

### GripperState.msg

夹爪状态。

```
bool enabled
float64 position
float64 velocity
float64 effort
bool is_open
bool is_closed
bool object_detected
```

### SubsystemStatus.msg

子系统状态。

```
string name
bool enabled
bool ready
bool error
string error_message
```

## 服务 (srv)

### GetRobotState.srv

获取整机状态。

```
---
uni_dingo_msgs/RobotState state
bool success
string message
```

### EmergencyStop.srv

急停控制。

```
bool activate                        # true=急停, false=恢复
---
bool success
string message
```

### SetEnabled.srv

启用/禁用子系统。

```
string subsystem                     # "base", "left_arm", "right_arm", "head"
bool enabled
---
bool success
string message
```

## 动作 (action)

### MoveArm.action

机械臂运动动作。

```
# Goal
string mode                          # "joint", "pose", "named"
float64[] joint_positions            # mode=joint
geometry_msgs/PoseStamped target_pose # mode=pose
string named_target                  # mode=named
float64 velocity_scaling
float64 acceleration_scaling
---
# Result
bool success
string message
float64[] final_joint_positions
---
# Feedback
float64 progress
string status
```

### MoveBase.action

底盘运动动作。

```
# Goal
geometry_msgs/Pose2D target_pose
string frame_id
float64 linear_tolerance
float64 angular_tolerance
---
# Result
bool success
string message
geometry_msgs/Pose2D final_pose
---
# Feedback
float64 distance_remaining
float64 angle_remaining
```

### MoveHead.action

云台运动动作。

```
# Goal
float64 pan
float64 tilt
float64 speed
---
# Result
bool success
string message
---
# Feedback
float64 pan_current
float64 tilt_current
```

### MoveGripper.action

夹爪控制动作。

```
# Goal
string command                       # "open", "close", "position"
float64 position
float64 max_effort
---
# Result
bool success
bool object_detected
float64 final_position
---
# Feedback
float64 position
float64 effort
```

### WholeBodyMove.action

全身运动动作。

```
# Goal
geometry_msgs/Pose2D base_goal
uni_dingo_msgs/MoveArmGoal left_arm_goal
uni_dingo_msgs/MoveArmGoal right_arm_goal
---
# Result
bool success
string message
---
# Feedback
float64 overall_progress
string current_phase
```
