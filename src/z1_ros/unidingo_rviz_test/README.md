# RViz 双臂 TCP 预览与测试

在 **TCP targets** 面板选择 **LEFT / RIGHT**，拖动 MoveIt 手柄编辑橙色目标臂。淡紫色机械臂播放 SDK 参考轨迹；点击 **SEND POSE** 才发送目标位姿。选择 **Home pose** 也只更新目标和预览。

## 在 ROS 1 Noetic 部署机启动

```bash
sudo apt install ros-noetic-moveit-ros-visualization ros-noetic-moveit-ros-move-group \
  ros-noetic-moveit-planners-ompl ros-noetic-trac-ik-kinematics-plugin
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch unidingo_rviz_test rviz_tcp_test.launch
```

入口连接真实双臂 MCU，包含两个 SDK 控制器，与 `z1_bringup arm.launch` 二选一。地址和端口来自 [communication.yaml](../z1_bringup/config/communication.yaml)。两臂从 Disabled 启动；有效且静止时，SEND 可以直接开始有限运动。默认速度比例为 `0.05`。

## 编辑、预览、明确发送

1. 等待反馈和整机模型，选择 LEFT 或 RIGHT。当前臂的 MoveIt 手柄用于编辑目标，未编辑的目标状态跟随实测关节。
2. 拖动平移箭头或旋转圆环。MoveIt 使用 TRAC-IK 更新橙色目标臂；SDK 在后台计算淡紫色执行参考。不可达或反馈过期时，SEND 不可用。
3. 等 SDK 预览就绪后点击 SEND POSE。发送后按钮立即禁用，直到该命令返回结果；连续点击不会覆盖正在跟踪的命令。
4. 成功后按编辑器版本恢复实测状态；运动中编辑了新目标，则保留新目标。命令被拒绝时保留目标，并显示原因。

**橙色目标臂是 MoveIt 的 IK 解，淡紫色动画是 SDK 的 IK＋Ruckig 参考。** 同一个 TCP 位姿可以对应不同的关节解；实际发送的是位姿，SDK 从接收时的起点重新求解和验证。界面秒数为 SDK 参考时长，动画默认按官方配置的 `3x` 播放；改为 `1x` 可按参考时序播放。

面板保留 **Measured TCP、STOP、Hold after stop、Enable、Disable、Ack fault**。Measured TCP 清除本臂编辑并恢复实测目标；STOP 后等 AwaitResume，再 Hold 回到 Holding。停止操作不等待预览服务。数值使用本臂基座坐标系，四元数顺序为 xyzw。

实心整机来自有效关节反馈，底盘、云台和显示夹爪保持零位。选臂时只播放当前臂的参考，另一臂使用生成该预览时的实测关节，不参与当前轨迹。

## Home 使用官方命名状态

[dual_arm.srdf](config/dual_arm.srdf) 将[宇树官方 SRDF](https://github.com/unitreerobotics/z1_ros/blob/d3bafd38fb48e805096da7abb0c19a15b636975d/z1_moveit_config/config/z1_description.srdf)的 Home 按左右臂前缀映射：

```text
joint1..joint6 = [0, 0, 0, -0.07435, 0, 0] rad
```

Home pose 调用 MoveIt 的命名状态接口，HOME TARGETS 同时选择两臂 Home。随后分别切换页签并确认发送，不会直接执行关节 Home 命令。

默认 SDK TCP 为 `tool`，相对官方规划链末端 `link06` 沿 X 平移 `0.051 m`。默认 Home TCP 约为：

```text
position [m]        = [0.037730, 0, 0.173143]
quaternion [x,y,z,w] = [0, -0.037166, 0, 0.999309]
```

适配器从 MoveIt 目标状态计算各臂基座下的 SDK TCP 位姿，使用完整精度。更改控制器 TCP 时，整机显示模型也要包含对应的带前缀 link；更改交互链末端时同步修改 SRDF。机械零点写入仍使用[标定工具](../CALIBRATION.md)。

## 官方组件与 SDK 的边界

官方基准为宇树 [z1_ros Noetic，d3bafd38](https://github.com/unitreerobotics/z1_ros/tree/d3bafd38fb48e805096da7abb0c19a15b636975d) 的 `real_arm.launch → view_robot.launch → robot.rviz`，交互接口对照 [MoveIt 1.1.16](https://github.com/moveit/moveit/tree/1.1.16)。

[SdkMotionPlanning](src/sdk_motion_planning.cpp) 继承官方 `MotionPlanningDisplay`，沿用其 RobotInteraction、目标状态、TRAC-IK、虚影、场景坐标系和轨迹播放。适配层只处理左右臂选择、官方 Home 命名状态、SDK TCP 消息和预览复位。原生 MoveIt 规划执行面板隐藏，操作由 TCP targets 面板提供。

[preview_move_group.launch](launch/preview_move_group.launch) 提供官方场景监视与运动学上下文，设置 `allow_trajectory_execution=false`，不启动 `z1_hw` 或 ros_control 控制器。MoveIt 依赖全部位于本测试工具内，控制 SDK 与 `z1_msgs` 保持独立。

[pose_preview.cpp](src/pose_preview.cpp) 从两个实际控制器读取模型和运动限制，共用 SDK 的 [goal_planner](../z1_control/src/goal_planner.cpp)。它只计算参考，不建立 MCU 连接。显示器播放该参考，而不是 OMPL 规划路径。真正执行仍只发送原有 `/left_arm/pose_goal`、`/right_arm/pose_goal`。

控制 URDF 的速度上限恢复为官方的 `3.1415 rad/s`。用户确认安装原装夹爪后，模型恢复其定子和动子的质量、质心、惯性张量，合计 `0.80224957 kg`。六轴动力学将夹爪固定在闭合零位；夹爪开合不会在线更新该惯性。TCP 保持原定义。

## 排查入口与部署验收

以下名称相对 `/rviz_tcp_test/`，`<side>` 为 left 或 right：

| 入口 | 用途 |
| --- | --- |
| `<side>/editor_target` | 官方目标状态导出的本臂 TCP、编辑状态与 IK 有效性 |
| `editor_action` | 选臂、Home、实测目标和官方轨迹复位 |
| `<side>_preview/preview` | SDK 只读参考服务，起点顺序为 joint1..joint6 |
| `preview_trajectory` | 原生 MoveIt 轨迹播放器接收的 SDK 参考 |
| `ui_action` / `ui_status` | 面板操作 / 状态和目标数值 |
| `joint_states` | 转发有效原始关节反馈，保留源时间戳 |

参数见 [rviz_tcp_test.yaml](config/rviz_tcp_test.yaml)。反馈时效默认 `0.3 s`，关节消息按自身时间戳判断；起点变化超过控制器 `position_tolerance` 时重新计算预览。

在部署机完成 catkin 构建后，依次检查：

1. MoveIt SDK targets 显示与 TCP targets 面板加载；左右页签切换对应手柄。
2. 拖动和 Home 只更新目标与预览，两个 `pose_goal` 话题均无新命令。
3. 参考从实际安装位置开始；切换页签、Measured TCP 能清除旧动画。
4. 默认低速发送一次，核对 `pose_goal`、`result`、实测 TCP；重复点击不重复提交，成功后恢复实测目标。
5. 检查新编辑目标保留，以及 STOP → AwaitResume → Hold。模型惯性已改变，先以夹爪闭合状态确认重力补偿与保持表现。
