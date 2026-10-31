# Z1 双臂 ROS 1 SDK

本目录提供双 Unitree Z1 机械臂的 C++ 控制内核、ROS 1 Topic 接口、MCU UDP 通信、模型和 RViz 工具。每臂控制六个主关节，检测到夹爪后另行控制夹爪。左右臂各运行一个进程，独立接收目标、执行运动和停止恢复。

## 目录

| 目录 | 内容 |
| --- | --- |
| [z1_control](z1_control) | 不依赖 ROS 的 C++ 控制内核、MCU 适配、`z1_topic_node` 和 `z1_calibrate` |
| [z1_msgs/msg](z1_msgs/msg) | 运动目标、控制操作、连续参考、状态与结果消息 |
| [z1_description](z1_description) | 控制 URDF、显示 Xacro 和网格资源 |
| [z1_bringup](z1_bringup) | 双臂启动文件及通信、控制配置 |
| [unidingo_rviz_test](unidingo_rviz_test) | 整机显示、TCP 虚影与轨迹预览、Home 位姿预设 |
| [vendor/unitree](vendor/unitree) | 官方通信头文件和 Linux x86_64、aarch64 动态库 |
| [tools](tools) | 通信接口离线预检与控制计算分配检查工具 |

ROS 适配使用 ROS 1 + catkin。控制包要求 CMake 3.16、C++17，依赖 Eigen3、yaml-cpp、Boost 头文件、Pinocchio、Ruckig、OpenSSL 和线程库；CMake 的版本请求分别为 Eigen3 3.3、Pinocchio 4.0、Ruckig 0.9.2。MCU 适配支持 Linux x86_64 和 aarch64。构建依赖见 [CMakeLists.txt](z1_control/CMakeLists.txt) 和各包的 `package.xml`。

装好 ROS 1 环境和工作区依赖，在仓库根目录构建：

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## 启动

### 双臂控制器

```bash
roslaunch z1_bringup arm.launch
```

[arm.launch](z1_bringup/launch/arm.launch) 加载通信配置，通过内部 [controllers.launch](z1_bringup/launch/controllers.launch) 启动两个 `z1_control/z1_topic_node`：

| 控制节点 | Topic 命名空间 | 关节及 TF 前缀 | 默认 MCU IP | 本地 UDP 端口 |
| --- | --- | --- | --- | --- |
| `/left_arm/controller` | `/left_arm` | `left_` | `192.168.123.110` | `8881` |
| `/right_arm/controller` | `/right_arm` | `right_` | `192.168.123.111` | `8882` |

启动前建立主机到两臂 MCU 的网络连接。地址、本地端口和接收预算在 [communication.yaml](z1_bringup/config/communication.yaml) 中，MCU 目标端口为 `8880`。本入口从 YAML 读取配置，修改后重新启动。

`communication.yaml` 的 `publish_description` 默认为 `false`。改成 YAML 布尔值 `true` 后，辅助节点加载 [descriptions.launch](z1_bringup/launch/descriptions.launch)，向 `/left_arm/robot_description` 和 `/right_arm/robot_description` 写入模型，并启动两个 `robot_state_publisher`。保持默认时，独立入口只运行控制器；模型和 TF 由整机启动层提供。

### 整机 RViz 与 TCP 测试

```bash
roslaunch unidingo_rviz_test rviz_tcp_test.launch
```

这个入口同时启动双臂控制器、连接真实 MCU，并加载整机模型、TF、交互节点和 RViz，与 `arm.launch` 二选一。底盘、云台和 D455 用于模型显示，相关驱动由各自入口启动。整机模型和 TF 始终由该入口发布。

拖动 TCP 手柄并松开，只更新虚影和 SDK 参考轨迹预览；在 `TCP targets` 面板选择 LEFT / RIGHT 并点击 `SEND POSE`，才发送一次 `PoseGoal`，默认速度比例 `0.05`。`HOME TARGETS` 通过官方 SRDF 命名状态选择 Home，再导出 SDK TCP 位姿，随后分别确认发送。目标编辑与虚影复用官方 MoveIt MotionPlanning，测试入口运行关闭执行的 move_group；实际运动仍走 PoseGoal，SDK 保持独立。操作细节见 [RViz 测试说明](unidingo_rviz_test/README.md)。

## 配置与模型

| 文件 | 用途 |
| --- | --- |
| [communication.yaml](z1_bringup/config/communication.yaml) | 两臂 MCU IP、本地 UDP 端口、接收预算及独立模型发布开关 |
| [control.yaml](z1_bringup/config/control.yaml) | 两臂共用的控制周期、反馈与流式超时、参考生成参数、运动限制、主机和 MCU 增益及到达容差 |
| [z1_control.urdf](z1_description/urdf/z1_control.urdf) | 六轴运动学、关节限位、质量/惯性和 TCP；碰撞描述供上层规划使用 |
| [z1.urdf.xacro](z1_description/urdf/z1.urdf.xacro) | 独立机械臂的显示模型 |
| [rviz_tcp_test.yaml](unidingo_rviz_test/config/rviz_tcp_test.yaml) | 交互目标速度比例和显示反馈时效 |
| [calibration.yaml](z1_control/config/calibration.yaml) | 标定对象与顺序、被动采样、静止检查和零位验证 |

两个控制节点始终读取控制 URDF，显示模型的发布开关不影响控制模型。默认 TCP 名为 `tool`，从 `link06` 沿 X 正方向平移 `0.051 m`，姿态不变，夹爪开合不影响它；显示模型中对应 `left_tool` 和 `right_tool`。

关节数据按 `joint1..joint6` 排列，角度用 rad，角速度用 rad/s，力矩用 N·m，位置用 m。笛卡尔目标使用本臂基座系 `left_link00` 或 `right_link00`，四元数按 `(x, y, z, w)` 填写并归一化。发送其他坐标系下的目标前需完成坐标转换；RViz 交互节点会把 `base_link` 下的目标转到本臂基座系。

## 控制循环

每臂进程由四部分组成：ROS 消息适配、独立的规划回调队列、原生控制线程和 MCU 通信适配。有限运动在规划回调里完成 IK、轨迹生成和关节约束验证，提交时再核对一次控制状态；控制操作和流式消息走普通回调线程。

原生控制线程按单调时钟运行：

```text
发送上一轮输出 → 接收 MCU 反馈 → 更新控制状态和参考 → 计算下一轮输出 → 等待下一时隙
```

ROS 启动配置为 `period=0.004 s`（目标 250 Hz）、`receive_budget_us=3000`，反馈发布 `100 Hz`。节点启动时检查接收预算小于控制周期。错过的绝对时隙会跳过并计数；`status` 里能看到实际周期、收发耗时、唤醒迟到和计算超期。控制计算超期只增加 `overruns`；使能后控制间隔超过反馈有效期会进入故障。

输出分两种控制律：

- **位置控制、保持、停止**：主机用参考位置/速度/加速度和实测反馈，经 `kp`、`kd` 构造期望加速度，再用逆动力学算力矩；同时下发参考 `q/dq` 和 `mcu_kp/mcu_kd`。默认 MCU Kp 为 `[500, 600, 500, 400, 300, 200]`，Kd 为 `[5, 5, 5, 5, 5, 5]`。
- **笛卡尔阻抗**：主机算 TCP 弹簧阻尼力矩，叠加动力学补偿，主臂 MCU Kp/Kd 置零；目标位姿是动态平衡点。

`torque` 和 `torque_rate` 限制主机写进 `tau` 的力矩幅值和变化率，幅值同时受 URDF effort 约束；MCU 的位置/速度增益走独立字段。夹爪用独立位置增益，目标范围 `[-1, 0] rad`。

## 状态与操作

两臂启动后都是 **Disabled**，持续交换被动帧、读取反馈，不提供主动姿态保持。反馈有效且机械臂静止时，第一条有限目标从实测位置规划执行；规划失败则保持 Disabled。单独发送 `ENABLE` 会从当前位置进入 **Holding**。

| 操作或事件 | 行为 |
| --- | --- |
| 有限目标 | Disabled 或 Holding 接收；Disabled 用实测起点，Holding 用保持参考；运动中提交会被拒绝 |
| `ENABLE` | 反馈有效且静止的 Disabled → Holding |
| `JOINT_STREAM` / `POSE_STREAM` / `IMPEDANCE` | Holding → 对应模式，建立新会话，等待第一条目标 |
| `STOP` 或流式超时 | 生成满足关节边界的停止轨迹；停稳后进入 AwaitResume，生成或验证失败则进入 Fault |
| `HOLD` | 反馈有效且静止的 AwaitResume → Holding，结束旧模式 |
| `RESUME` | 反馈有效且静止时：AwaitResume → 原流式模式的新会话，或有限运动停止后的 Holding；Fault → Disabled |
| `DISABLE` | 请求被动输出，取消当前运动 |

起步、使能和恢复使用 `start_velocity_tolerance`；到达和停止完成使用 `velocity_tolerance`，两者默认都是 `0.03 rad/s`。静止判定同时看原始速度和新的位置反馈；速度有尖峰时，用 `stationary_window`（默认 `0.1 s`）内的位置范围确认。`joint_limit_tolerance` 默认 `0.001 rad`，只放宽实测位置的判定，目标和生成轨迹仍按模型限位检查。

同一臂的有限运动共用正数、递增的 `command_id`。`ACCEPTED` 表示计划已接受；轨迹结束后，实测关节进入容差并稳定超过 `100 ms` 才返回 `SUCCEEDED`，位姿运动还需满足 TCP 位姿容差。参考轨迹结束后最多等待 `settle_timeout=3 s`，超时进入 Fault。

发流式目标前读取 `status.session_id`，随后在同一会话里递增 `sequence`，并填写非零 `header.stamp`。默认有效期从源时刻或未来路点时刻中较晚者起算，为 `0.2 s`；进入模式后等待首帧也有超时。停止或恢复后使用新会话号。有限运动的 `STOPPED` 事件表示开始制动，停稳看 `AWAIT_RESUME` 状态。

## 控制与几何规划分层

控制内核只建立运动学、动力学模型，执行关节运动、制动和力矩约束，不创建碰撞对象、不加载碰撞网格，也不计算几何距离。控制 URDF 可以完全省略 `collision` 标签。需要自碰撞、双臂互碰或环境避障时，在上层接入规划器。仓库的 URDF/Xacro 碰撞描述继续保留，供规划和仿真使用。

旧状态字段保留兼容：`collision_distance` 恒为 `NaN`，`collision_pair` 为空，表示距离不可用。旧几何配置和原生查询接口的迁移见[模型配置](INTERFACES.md#6-模型配置)。

## Topic 接口

下面的话题名相对各臂命名空间，例如左臂的关节目标是 `/left_arm/joint_goal`。自定义消息都在 `z1_msgs`。

| 输入 Topic | 消息类型 | 用途 |
| --- | --- | --- |
| `control` | `z1_msgs/ControlCommand` | 使能、失能、停止、恢复和模式选择 |
| `joint_goal` | `z1_msgs/JointGoal` | Ruckig 六轴关节点到点运动 |
| `pose_goal` | `z1_msgs/PoseGoal` | 终点 IK 后执行关节轨迹，中间 TCP 路径由关节轨迹决定 |
| `joint_trajectory` | `z1_msgs/JointTrajectory` | 带时间及可选导数的有限关节轨迹，可选择全局重定时 |
| `joint_waypoints` | `z1_msgs/JointWaypoints` | 自动安排时间的连续关节过点，首尾静止 |
| `linear_path` | `z1_msgs/LinearPath` | TCP 直线与旋转插值、连续 IK；多点可停车或按半径融合转角 |
| `joint_stream` / `joint_reference` | `z1_msgs/JointStream` / `z1_msgs/JointReference` | 持续关节目标 / 带目标时间和可选速度、加速度的关节参考 |
| `pose_stream` / `pose_reference` | `z1_msgs/PoseStream` / `z1_msgs/CartesianReference` | 持续 TCP 目标 / 带目标时间和可选导数的笛卡尔参考；用于 PoseStream 或 Impedance |
| `impedance_gains` | `z1_msgs/ImpedanceGains` | Holding 或 Impedance 下更新刚度和阻尼，阻抗中平滑生效 |
| `gripper_goal` | `z1_msgs/GripperGoal` | 有夹爪且处于 Holding 时接受位置目标，只返回接受或拒绝 |

| 输出 Topic | 消息类型 | 内容 |
| --- | --- | --- |
| `joint_states` | `sensor_msgs/JointState` | 原始六轴 q/dq/tau；有夹爪时追加 `<prefix>gripper_motor` |
| `commanded_joint_states` | `sensor_msgs/JointState` | 主机最近生成的六轴参考位置、参考速度和计算力矩（effort） |
| `tcp_pose` | `geometry_msgs/PoseStamped` | 根据有效关节反馈计算的基座系 TCP 位姿 |
| `status` | `z1_msgs/Status` | 模式、原因、会话、反馈序号/时龄、模型版本、轨迹时长及周期统计 |
| `result` | `z1_msgs/MotionResult` | 命令 ID、接受/成功/拒绝/取消/停止/失败及原因 |

收到首帧后，`joint_states` 持续发布最近收到的测量，消息头时间戳为发布时刻。有效性和新鲜度看 `status.feedback_valid`、`feedback_age` 和 `feedback_sequence`；`tcp_pose` 在反馈有效时发布。`commanded_joint_states` 记录六轴主机参考和计算力矩，结合 `status.mode` 读取。MCU 增益在设备侧参与控制，实际执行结果看反馈。

启动后可以直接读状态和对照数据：

```bash
rostopic echo /left_arm/status
rostopic echo /left_arm/result
rostopic echo /left_arm/joint_states
rostopic echo /left_arm/commanded_joint_states
```

操作码、消息字段、时间戳规则和发布示例见 [Topic 接口说明](INTERFACES.md)，参考生成与力矩计算见[算法详解](SDK_ALGORITHMS.md)。

## 零点标定

先退出双臂控制器或整机 RViz 入口，释放 UDP 端口，再把机械臂摆到机械零位。在已加载工作区的终端执行：

```bash
# 只检查握手、被动反馈和静止状态
rosrun z1_control z1_calibrate --check-only

# 按提示逐臂确认并标定
rosrun z1_control z1_calibrate
```

程序读取 [calibration.yaml](z1_control/config/calibration.yaml)，复用通信 YAML 中的地址和本地端口，默认先右臂、后左臂。标定请求走官方接口，标定后的六轴反馈由程序独立检查，默认参考为 `[0, 0, 0, -0.07435, 0, 0] rad`。`--check-only` 不发标定请求，也不校验机械零位姿态。结果含义、单臂选择和配置方式见[双臂零点标定](CALIBRATION.md)。

## 独立 C++ 构建与离线检查

控制内核和维护工具可以脱离 ROS 构建。装好原生依赖后，在仓库根目录执行：

```bash
cmake -S src/z1_ros/z1_control -B /tmp/z1-control-build \
  -DZ1_BUILD_ROS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/z1-control-build -j2
ctest --test-dir /tmp/z1-control-build --output-on-failure
```

catkin 构建自动打开 ROS 适配；普通 CMake 构建默认关闭 ROS 适配、启用控制内核。只构建 MCU 适配和标定工具时，再加 `-DZ1_BUILD_CORE=OFF`。原生 `Config` 的周期缺省值是 `0.002 s`；直接调用原生 API 要自己传运行配置，ROS YAML 里的 `0.004 s` 只对 ROS 节点生效。

只依赖官方通信库的接口预检见 [transport_preflight](tools/transport_preflight/README.md)，控制计算分配检查见 [performance_audit](tools/performance_audit/README.md)。
