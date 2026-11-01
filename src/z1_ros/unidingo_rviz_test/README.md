# RViz 双臂 TCP 预览与测试

在 **TCP targets** 面板中选择 **LEFT / RIGHT**，然后拖动 MoveIt 手柄来编辑橙色的目标臂。淡紫色机械臂会播放 SDK 的参考轨迹；只有点击 **SEND POSE**，才会真正发送目标位姿。选择 **Home pose** 同样只更新目标和预览。

## 在 ROS 1 Noetic 部署机启动

```bash
sudo apt install ros-noetic-moveit-ros-visualization ros-noetic-moveit-ros-move-group \
  ros-noetic-moveit-planners-ompl ros-noetic-trac-ik-kinematics-plugin
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch unidingo_rviz_test rviz_tcp_test.launch
```

这个入口会连接真实的双臂 MCU，并启动两个 SDK 控制器，因此与 `z1_bringup arm.launch` 只能二选一。地址和端口来自 [communication.yaml](../z1_bringup/config/communication.yaml)。两臂从 Disabled 状态启动；当反馈有效且机械臂静止时，SEND 可以直接开始有限运动。默认速度比例是 `0.05`。

## 编辑、预览、明确发送

1. 等待反馈和整机模型出现，然后选择 LEFT 或 RIGHT。当前臂的 MoveIt 手柄用于编辑目标；没有编辑目标时，目标状态会跟随实测关节。
2. 拖动平移箭头或旋转圆环。MoveIt 会用 TRAC-IK 更新橙色目标臂，SDK 则在后台计算淡紫色的执行参考。如果目标不可达或反馈过期，SEND 会不可用。
3. 等 SDK 预览就绪后，点击 SEND POSE。发送后按钮会立即禁用，直到该命令返回结果；连续点击不会覆盖正在跟踪的命令。
4. 成功后，界面会按编辑器版本恢复实测状态；如果运动过程中编辑了新目标，则保留新目标。命令被拒绝时，目标会保留，并显示原因。

**橙色目标臂是 MoveIt 的 IK 解，淡紫色动画是 SDK 的 IK＋Ruckig 参考。** 同一个 TCP 位姿可以对应不同的关节解；实际发送的是位姿，SDK 会从接收时的起点重新求解并验证。界面中的秒数是 SDK 参考时长，动画默认按官方配置的 `3x` 播放；改成 `1x` 就可以按参考时序播放。

面板还保留了 **Measured TCP、STOP、Hold after stop、Enable、Disable、Ack fault**。Measured TCP 会清除本臂的编辑，并恢复实测目标；STOP 之后要等 AwaitResume，再 Hold 才能回到 Holding。停止操作不会等待预览服务。数值使用本臂的基座坐标系，四元数顺序为 xyzw。

实心的整机模型来自有效的关节反馈，底盘、云台和显示夹爪保持零位。选择某一臂时，只播放当前臂的参考；另一臂则使用生成该预览时的实测关节，不参与当前轨迹。

## Home 使用官方命名状态

[dual_arm.srdf](config/dual_arm.srdf) 会把[宇树官方 SRDF](https://github.com/unitreerobotics/z1_ros/blob/d3bafd38fb48e805096da7abb0c19a15b636975d/z1_moveit_config/config/z1_description.srdf) 的 Home 按左右臂前缀映射过来：

```text
joint1..joint6 = [0, 0, 0, -0.07435, 0, 0] rad
```

Home pose 会调用 MoveIt 的命名状态接口，HOME TARGETS 则会同时选择两臂的 Home。随后需要分别切换页签并确认发送，不会直接执行关节 Home 命令。

默认的 SDK TCP 是 `tool`，它相对官方规划链末端 `link06` 沿 X 方向平移 `0.051 m`。默认 Home TCP 大约为：

```text
position [m]        = [0.037730, 0, 0.173143]
quaternion [x,y,z,w] = [0, -0.037166, 0, 0.999309]
```

适配器会从 MoveIt 目标状态计算各臂基座下的 SDK TCP 位姿，并使用完整精度。修改控制器 TCP 时，整机显示模型也要包含对应的带前缀 link；修改交互链末端时，要同步修改 SRDF。机械零点写入仍然使用[标定工具](../CALIBRATION.md)。

## 官方组件与 SDK 的边界

官方基准是宇树 [z1_ros Noetic，d3bafd38](https://github.com/unitreerobotics/z1_ros/tree/d3bafd38fb48e805096da7abb0c19a15b636975d) 的 `real_arm.launch → view_robot.launch → robot.rviz`，交互接口对照 [MoveIt 1.1.16](https://github.com/moveit/moveit/tree/1.1.16)。

[SdkMotionPlanning](src/sdk_motion_planning.cpp) 继承官方的 `MotionPlanningDisplay`，沿用它的 RobotInteraction、目标状态、TRAC-IK、虚影、场景坐标系和轨迹播放。适配层只处理左右臂选择、官方 Home 命名状态、SDK TCP 消息和预览复位。原生 MoveIt 规划执行面板被隐藏，操作改由 TCP targets 面板提供。

[preview_move_group.launch](launch/preview_move_group.launch) 提供官方的场景监视和运动学上下文，并设置 `allow_trajectory_execution=false`，不启动 `z1_hw` 或 ros_control 控制器。MoveIt 依赖全部位于这个测试工具内，控制 SDK 和 `z1_msgs` 保持独立。

[pose_preview.cpp](src/pose_preview.cpp) 会从两个实际控制器读取模型和运动限制，并复用 SDK 的 [goal_planner](../z1_control/src/goal_planner.cpp)。它只计算参考，不建立 MCU 连接。显示器播放的是这个参考，而不是 OMPL 规划出的路径。真正的执行仍然只发送原有的 `/left_arm/pose_goal` 和 `/right_arm/pose_goal`。

控制 URDF 的速度上限已经恢复为官方的 `3.1415 rad/s`。用户确认两臂安装原装夹爪后，模型恢复了定子和动子的质量、质心与惯性张量，合计 `0.80224957 kg`。六轴动力学把夹爪固定在闭合零位；夹爪开合不会在线更新这份惯性。TCP 保持原来的定义。

## 排查入口与部署验收

以下名称都相对于 `/rviz_tcp_test/`，其中 `<side>` 是 left 或 right：

| 入口 | 用途 |
| --- | --- |
| `<side>/editor_target` | 从官方目标状态导出的本臂 TCP、编辑状态和 IK 有效性 |
| `editor_action` | 选臂、Home、实测目标和官方轨迹复位 |
| `<side>_preview/preview` | SDK 的只读参考服务，起点顺序为 joint1..joint6 |
| `preview_trajectory` | 原生 MoveIt 轨迹播放器接收的 SDK 参考 |
| `ui_action` / `ui_status` | 面板操作 / 状态和目标数值 |
| `joint_states` | 转发有效的原始关节反馈，并保留源时间戳 |

参数见 [rviz_tcp_test.yaml](config/rviz_tcp_test.yaml)。反馈时效默认是 `0.3 s`，关节消息按自身时间戳判断；如果起点变化超过控制器的 `position_tolerance`，就会重新计算预览。

在部署机上完成 catkin 构建后，请依次检查：

1. MoveIt SDK targets 显示和 TCP targets 面板都能加载；左右页签切换时，手柄能对应到正确的臂。
2. 拖动和 Home 只更新目标与预览，两个 `pose_goal` 话题都没有新命令。
3. 参考从实际安装位置开始；切换页签或点击 Measured TCP 能清除旧动画。
4. 以默认低速发送一次，核对 `pose_goal`、`result` 和实测 TCP；重复点击不会重复提交，成功后能恢复实测目标。
5. 检查新编辑的目标能否保留，以及 STOP → AwaitResume → Hold 的流程。模型惯性已经改变，请先以夹爪闭合状态确认重力补偿和保持表现。
