# dingo
```bash
export DINGO_TELEOP_ENABLE = 0
export DINGO_OMNI=1

source devel/setup.bash 
  teleop:=keyboard 
```

# z1
```bash
# 终端1: 启动机械臂和 MoveIt
roslaunch z1_bringup real_arm.launch rviz:=true moveit:=true
roslaunch z1_bringup real_arm.launch rviz:=false moveit:=true
```

```bash
# left_link06 to base_link
At time 1768908322.527
- Translation: [0.117, -0.138, 0.743]
- Rotation: in Quaternion [-0.019, -0.033, -0.705, 0.709]
            in RPY (radian) [0.019, -0.074, -1.566]
            in RPY (degree) [1.117, -4.227, -89.720]
```

```bash
# right_link06 to base_link
At time 1768908448.872
- Translation: [-0.118, -0.138, 0.744]
- Rotation: in Quaternion [-0.030, -0.022, -0.710, 0.704]
            in RPY (radian) [-0.010, -0.073, -1.579]
            in RPY (degree) [-0.578, -4.190, -90.481]
```

```bash
# 终端2: 启动 z1_control_node
source devel/setup.bash
roslaunch z1_control z1_control.launch
```

```bash
# 发送目标位姿
rostopic pub /z1_dual_control_node/left_arm/target_pose geometry_msgs/Pose "
position:
  x: 0.117
  y: -0.138
  z: 0.900
orientation:
  x: -0.019
  y: -0.033
  z: -0.705
  w: 0.709" --once
```

```bash
# 发送目标位姿
rostopic pub /z1_dual_control_node/right_arm/target_pose geometry_msgs/Pose "
position:
  x: -0.118
  y: -0.138
  z: 0.900
orientation:
  x: -0.030
  y: -0.022
  z: -0.710
  w: 0.704" --once
```

```bash
# 打开夹爪 (位置 -1.57 rad = -90度 = 全开)
rostopic pub /z1_dual_control_node/left_arm/gripper_command std_msgs/Float64 "data: -1.57" --once

# 关闭夹爪 (位置 0.0 rad = 全关)
rostopic pub /z1_dual_control_node/right_arm/gripper_command std_msgs/Float64 "data: 0.0" --once
```

```bash
# 可视化坐标系
roslaunch z1_description z1_rviz.launch
```

```bash
# 仿真整机
roslaunch uni_dingo dual_arm_gazebo_moveit.launch
```