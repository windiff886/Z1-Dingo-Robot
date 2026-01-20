# Z1 Control Node

使用 MoveIt 控制 Z1 机械臂末端位姿，通过 Action 接口控制夹爪。

## 编译

```bash
cd ~/lxy/z1_dingo
catkin_make --pkg z1_control
```

## 使用方法

### 1. 启动机械臂

```bash
# 终端1: 启动机械臂和 MoveIt
roslaunch z1_bringup real_arm.launch rviz:=false
```

### 2. 启动控制节点

```bash
# 终端2: 启动 z1_control_node
source devel/setup.bash
roslaunch z1_control z1_control.launch
```

### 3. 发送控制命令

#### 控制末端位姿

```bash
# 发送目标位姿
rostopic pub /z1_control_node/target_pose geometry_msgs/Pose "
position:
  x: 0.0
  y: 0.0
  z: 0.3
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0" --once
```

#### 控制夹爪

```bash
# 打开夹爪 (位置 -1.57 rad = -90度 = 全开)
rostopic pub /z1_control_node/gripper_command std_msgs/Float64 "data: -1.57" --once

# 关闭夹爪 (位置 0.0 rad = 全关)
rostopic pub /z1_control_node/gripper_command std_msgs/Float64 "data: 0.0" --once
```

#### 查看当前位姿

```bash
rostopic echo /z1_control_node/current_pose
```

## 话题接口

| 话题名 | 类型 | 方向 | 说明 |
|--------|------|------|------|
| `~target_pose` | `geometry_msgs/Pose` | 订阅 | 末端目标位姿 |
| `~gripper_command` | `std_msgs/Float64` | 订阅 | 夹爪位置命令 (rad: 0.0~-1.57) |
| `~current_pose` | `geometry_msgs/PoseStamped` | 发布 | 当前末端位姿 |

## 参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `planning_group` | `manipulator` | MoveIt 规划组名称 |

## 代码中使用

如果需要在自己的代码中调用，可以直接发布上述话题，或者参考 `z1_control_node.cpp` 中的 `Z1Controller` 类。
