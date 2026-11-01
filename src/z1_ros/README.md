# Z1 双臂 ROS 1 SDK

本目录提供两台 Unitree Z1 机械臂的 C++ 控制内核、ROS 1 Topic 接口、MCU UDP 通信、模型和 RViz 工具。每臂控制六个主关节，检测到夹爪后再单独控制夹爪。左右臂各运行一个进程，分别接收目标、执行运动，并在需要时停止和恢复。

## 目录

| 目录 | 内容 |
| --- | --- |
| [z1_control](z1_control) | 不依赖 ROS 的 C++ 控制内核、MCU 适配、`z1_topic_node` 和 `z1_calibrate` |
| [z1_msgs/msg](z1_msgs/msg) | 运动目标、控制操作、连续参考、状态与结果消息 |
| [z1_description](z1_description) | 控制 URDF、显示 Xacro 和网格资源 |
| [z1_bringup](z1_bringup) | 双臂启动文件，以及通信和控制配置 |
| [unidingo_rviz_test](unidingo_rviz_test) | 整机显示、TCP 虚影与轨迹预览、Home 位姿预设 |
| [vendor/unitree](vendor/unitree) | 官方通信头文件和 Linux x86_64、aarch64 动态库 |
| [tools](tools) | 通信接口离线预检工具，以及控制计算的分配检查工具 |

ROS 适配使用 ROS 1 + catkin。控制包要求 CMake 3.16 和 C++17，依赖 Eigen3、yaml-cpp、Boost 头文件、Pinocchio、Ruckig、OpenSSL 和线程库；其中 CMake 请求的 Eigen3、Pinocchio、Ruckig 版本分别为 3.3、4.0 和 0.9.2。MCU 适配支持 Linux x86_64 和 aarch64。完整的构建依赖见 [CMakeLists.txt](z1_control/CMakeLists.txt) 和各包的 `package.xml`。

装好 ROS 1 环境和工作区依赖后，在仓库根目录构建：

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## 启动

### 双臂控制器

```bash
roslaunch z1_bringup arm.launch
```

[arm.launch](z1_bringup/launch/arm.launch) 负责加载通信配置，并通过内部的 [controllers.launch](z1_bringup/launch/controllers.launch) 启动两个 `z1_control/z1_topic_node`：

| 控制节点 | Topic 命名空间 | 关节及 TF 前缀 | 默认 MCU IP | 本地 UDP 端口 |
| --- | --- | --- | --- | --- |
| `/left_arm/controller` | `/left_arm` | `left_` | `192.168.123.110` | `8881` |
| `/right_arm/controller` | `/right_arm` | `right_` | `192.168.123.111` | `8882` |

启动前，需要先建立主机到两臂 MCU 的网络连接。地址、本地端口和接收预算都在 [communication.yaml](z1_bringup/config/communication.yaml) 中配置，MCU 的目标端口为 `8880`。这个入口会从 YAML 读取配置，修改后必须重新启动才生效。

`communication.yaml` 中的 `publish_description` 默认为 `false`。把它改成 YAML 布尔值 `true` 后，辅助节点会加载 [descriptions.launch](z1_bringup/launch/descriptions.launch)，向 `/left_arm/robot_description` 和 `/right_arm/robot_description` 写入模型，并启动两个 `robot_state_publisher`。如果保持默认值，这个独立入口只运行控制器；模型和 TF 由整机启动层提供。

### 整机 RViz 与 TCP 测试

```bash
roslaunch unidingo_rviz_test rviz_tcp_test.launch
```

这个入口会同时启动双臂控制器、连接真实 MCU，并加载整机模型、TF、交互节点和 RViz，因此与 `arm.launch` 只能二选一。底盘、云台和 D455 只用于模型显示，它们的驱动仍需由各自的入口启动。整机模型和 TF 始终由这个入口发布。

拖动 TCP 手柄并松开时，只会更新虚影和 SDK 的参考轨迹预览。只有在 `TCP targets` 面板中选择 LEFT 或 RIGHT，再点击 `SEND POSE`，才会真正发送一次 `PoseGoal`，默认速度比例为 `0.05`。`HOME TARGETS` 会通过官方 SRDF 的命名状态选择 Home，再导出 SDK 的 TCP 位姿，随后需要分别确认发送。目标编辑和虚影都复用官方 MoveIt MotionPlanning：测试入口运行的是关闭轨迹执行的 move_group，实际运动仍然通过 PoseGoal 下发，SDK 保持独立。操作细节见 [RViz 测试说明](unidingo_rviz_test/README.md)。

### 关节摩擦辨识

运行 `roslaunch z1_bringup friction_identification.launch`。程序会在预检通过后，自动依次执行左臂 J1～J6 和右臂 J1～J6 的往返运动与起动测量，并保存原始 CSV 和摩擦参数 YAML。每测完一个轴，机械臂都会返回固定的辨识基准姿态，而不是回到 Home。测试轴、速度和基准姿态都在 `z1_control/config/friction_identification.yaml` 中配置。这个入口会自行启动控制器，因此与其他控制 launch 只能二选一。操作方法和数据判定见[摩擦辨识说明](FRICTION_IDENTIFICATION.md)。

### 摩擦补偿与实机对照

左右臂的六轴参数统一放在 `z1_control/config/friction_compensation.yaml` 中，默认全部为零。标准控制入口会自动加载这些参数，位置控制和笛卡尔阻抗两种控制律都支持摩擦补偿。完成辨识后，用 `rosrun z1_control friction_export.py /本次记录目录` 更新参数并备份，然后重启控制器生效。`roslaunch z1_bringup friction_comparison.launch` 会在保持轨迹、增益和限幅不变的前提下，只切换补偿开关，输出实测误差、速度波动和停走比例。完整流程和开关接口见[摩擦补偿与 A/B 对照](SDK_ALGORITHMS.md#9-摩擦补偿参数与实机对照)。

## 配置与模型

| 文件 | 用途 |
| --- | --- |
| [communication.yaml](z1_bringup/config/communication.yaml) | 两臂 MCU 的 IP、本地 UDP 端口、接收预算，以及独立模型发布的开关 |
| [control.yaml](z1_bringup/config/control.yaml) | 两臂共用的控制周期、反馈与流式超时、参考生成参数、运动限制、主机和 MCU 增益，以及到达容差 |
| [z1_control.urdf](z1_description/urdf/z1_control.urdf) | 六轴运动学、关节限位、质量/惯性和 TCP；碰撞描述供上层规划使用 |
| [z1.urdf.xacro](z1_description/urdf/z1.urdf.xacro) | 独立机械臂的显示模型 |
| [rviz_tcp_test.yaml](unidingo_rviz_test/config/rviz_tcp_test.yaml) | 交互目标的速度比例和显示反馈时效 |
| [calibration.yaml](z1_control/config/calibration.yaml) | 标定对象与顺序、被动采样、静止检查和零位验证 |

两个控制节点始终读取控制 URDF，显示模型是否发布不影响控制模型。默认 TCP 名为 `tool`，它从 `link06` 沿 X 正方向平移 `0.051 m`，姿态保持不变，夹爪开合也不会影响它；显示模型里对应的是 `left_tool` 和 `right_tool`。

控制节点会根据 MCU 握手的结果自动选择附件模型：如果检测到 8 个电机，就把夹爪的质量和惯量计入模型；如果检测到 7 个电机，就移除夹爪子树（J2 使用两个电机）。当前设备对应的是左臂带夹爪、右臂无夹爪。实际选择会通过 `controller/device_has_gripper` 和 `controller/effective_model_xml` 公布；这不会改变六轴限位和 TCP 变换。构建时新增了 TinyXML2 依赖；辨识用的 Python 节点需要 NumPy 和 PyYAML。

关节数据按 `joint1..joint6` 排列，角度用 rad，角速度用 rad/s，力矩用 N·m，位置用 m。笛卡尔目标使用本臂的基座系 `left_link00` 或 `right_link00`，四元数按 `(x, y, z, w)` 填写并归一化。如果要发送其他坐标系下的目标，需要先做好坐标转换；RViz 交互节点会把 `base_link` 下的目标转换到本臂基座系。

## 控制循环

每臂的进程由四部分组成：ROS 消息适配、独立的规划回调队列、原生控制线程和 MCU 通信适配。有限运动会在规划回调中完成 IK、轨迹生成和关节约束验证，提交时再核对一次控制状态；控制操作和流式消息则由普通回调线程处理。

原生控制线程按照单调时钟运行：

```text
发送上一轮输出 → 接收 MCU 反馈 → 更新控制状态和参考 → 计算下一轮输出 → 等待下一时隙
```

ROS 启动配置为 `period=0.004 s`（目标 250 Hz）、`receive_budget_us=3000`，反馈发布频率为 `100 Hz`。节点启动时会检查接收预算是否小于控制周期。错过的绝对时隙会被跳过并计数；从 `status` 中可以看到实际周期、收发耗时、唤醒迟到和计算超期。控制计算超期只会增加 `overruns`；使能之后，如果控制间隔超过反馈有效期，就会进入故障。

输出分为两种控制律：

- **位置控制、保持和停止**：主机根据参考位置/速度/加速度和实测反馈，用 `kp`、`kd` 构造期望加速度，再通过逆动力学计算力矩；同时下发参考 `q/dq` 和 `mcu_kp/mcu_kd`。默认 MCU Kp 为 `[500, 600, 500, 400, 300, 200]`，Kd 为 `[5, 5, 5, 5, 5, 5]`。
- **笛卡尔阻抗**：主机计算 TCP 的弹簧阻尼力矩，再叠加动力学补偿，并把主臂 MCU 的 Kp/Kd 置零；目标位姿就是动态平衡点。

两种控制律都会在总的主机限幅之前叠加已配置的摩擦前馈：位置模式使用参考关节速度，阻抗模式使用滤波后的实测关节速度，并限制补偿量的变化率。系数全为零时，输出与原来完全一致；夹爪不使用六轴摩擦补偿。

`torque` 和 `torque_rate` 限制主机写入 `tau` 的力矩幅值和变化率，幅值同时受 URDF effort 约束；MCU 的位置/速度增益通过独立字段下发。夹爪使用独立的位置增益，目标范围为 `[-1, 0] rad`。

## 状态与操作

两臂启动后都处于 **Disabled** 状态，只持续交换被动帧、读取反馈，不提供主动姿态保持。当反馈有效且机械臂静止时，第一条有限目标会从实测位置开始规划和执行；如果规划失败，则保持 Disabled。单独发送 `ENABLE` 会让机械臂从当前位置进入 **Holding**。

| 操作或事件 | 行为 |
| --- | --- |
| 有限目标 | 可以在 Disabled 或 Holding 状态下接收；Disabled 用实测位置作为起点，Holding 用当前保持参考作为起点；运动过程中提交新目标会被拒绝 |
| `ENABLE` | 当反馈有效且机械臂静止时，从 Disabled 进入 Holding |
| `JOINT_STREAM` / `POSE_STREAM` / `IMPEDANCE` | 从 Holding 进入对应模式，建立新会话，并等待第一条目标 |
| `STOP` 或流式超时 | 生成一条满足关节边界的停止轨迹；停稳后进入 AwaitResume，如果生成或验证失败则进入 Fault |
| `HOLD` | 当反馈有效且机械臂静止时，从 AwaitResume 回到 Holding，并结束旧模式 |
| `RESUME` | 当反馈有效且机械臂静止时：从 AwaitResume 回到原流式模式的新会话，或从有限运动停止后回到 Holding；从 Fault 回到 Disabled |
| `DISABLE` | 请求被动输出，并取消当前运动 |

起步、使能和恢复使用 `start_velocity_tolerance`；到达和停止完成使用 `velocity_tolerance`，两者的默认值都是 `0.03 rad/s`。静止判定会同时参考原始速度和新的位置反馈；如果速度出现尖峰，就用 `stationary_window`（默认 `0.1 s`）内的位置范围来确认。`joint_limit_tolerance` 默认为 `0.001 rad`，它只放宽实测位置的判定，目标和生成轨迹仍然按模型限位检查。

同一臂上的有限运动共用一个正数且递增的 `command_id`。`ACCEPTED` 表示计划已经被接受；轨迹结束后，实测关节进入容差范围并稳定超过 `100 ms`，才会返回 `SUCCEEDED`，位姿运动还需要满足 TCP 位姿容差。参考轨迹结束后最多等待 `settle_timeout=3 s`，超时则进入 Fault。

发送流式目标前，先读取 `status.session_id`，然后在同一个会话中递增 `sequence`，并填写非零的 `header.stamp`。默认有效期从源时刻或未来路点时刻中较晚的那个开始计算，长度为 `0.2 s`；进入模式后等待第一帧也有超时限制。停止或恢复之后，需要使用新的会话号。有限运动的 `STOPPED` 事件表示开始制动，是否停稳要看 `AWAIT_RESUME` 状态。

## 控制与几何规划分层

控制内核只建立运动学和动力学模型，负责执行关节运动、制动和力矩约束；它不创建碰撞对象、不加载碰撞网格，也不计算几何距离。因此控制 URDF 可以完全省略 `collision` 标签。如果需要自碰撞、双臂互碰或环境避障，需要在上一层接入规划器。仓库中的 URDF/Xacro 碰撞描述仍然保留，供规划和仿真使用。

旧状态字段为了兼容而保留：`collision_distance` 恒为 `NaN`，`collision_pair` 为空，表示距离不可用。旧几何配置和原生查询接口的迁移方法见[模型配置](INTERFACES.md#6-模型配置)。

## Topic 接口

下面的话题名都是相对各臂命名空间的，例如左臂的关节目标是 `/left_arm/joint_goal`。自定义消息都定义在 `z1_msgs` 中。

| 输入 Topic | 消息类型 | 用途 |
| --- | --- | --- |
| `control` | `z1_msgs/ControlCommand` | 使能、失能、停止、恢复和模式选择 |
| `joint_goal` | `z1_msgs/JointGoal` | 用 Ruckig 生成六轴关节点到点运动 |
| `pose_goal` | `z1_msgs/PoseGoal` | 先求终点 IK，再执行关节轨迹，中间的 TCP 路径由关节轨迹决定 |
| `joint_trajectory` | `z1_msgs/JointTrajectory` | 带时间和可选导数的有限关节轨迹，可以选择全局重定时 |
| `joint_waypoints` | `z1_msgs/JointWaypoints` | 自动安排时间的连续关节过点，首尾静止 |
| `linear_path` | `z1_msgs/LinearPath` | TCP 直线与旋转插值，配合连续 IK；多点时可以停车，或按半径融合转角 |
| `joint_stream` / `joint_reference` | `z1_msgs/JointStream` / `z1_msgs/JointReference` | 持续关节目标 / 带目标时间和可选速度、加速度的关节参考 |
| `pose_stream` / `pose_reference` | `z1_msgs/PoseStream` / `z1_msgs/CartesianReference` | 持续 TCP 目标 / 带目标时间和可选导数的笛卡尔参考；用于 PoseStream 或 Impedance 模式 |
| `impedance_gains` | `z1_msgs/ImpedanceGains` | 在 Holding 或 Impedance 下更新刚度和阻尼，阻抗模式下会平滑生效 |
| `gripper_goal` | `z1_msgs/GripperGoal` | 有夹爪且处于 Holding 时接受位置目标，只返回接受或拒绝 |

| 输出 Topic | 消息类型 | 内容 |
| --- | --- | --- |
| `joint_states` | `sensor_msgs/JointState` | 原始六轴 q/dq/tau；有夹爪时追加 `<prefix>gripper_motor` |
| `commanded_joint_states` | `sensor_msgs/JointState` | 主机最近生成的六轴参考位置、参考速度和计算力矩（effort） |
| `tcp_pose` | `geometry_msgs/PoseStamped` | 根据有效关节反馈计算的基座系 TCP 位姿 |
| `status` | `z1_msgs/Status` | 模式、原因、会话、反馈序号/时龄、模型版本、轨迹时长以及周期统计 |
| `result` | `z1_msgs/MotionResult` | 命令 ID，以及接受/成功/拒绝/取消/停止/失败的结果和原因 |

收到首帧之后，`joint_states` 会持续发布最近收到的测量值，消息头的时间戳是发布时刻。反馈是否有效、是否新鲜，要看 `status.feedback_valid`、`feedback_age` 和 `feedback_sequence`；`tcp_pose` 只在反馈有效时发布。`commanded_joint_states` 记录六轴主机参考和计算力矩，需要结合 `status.mode` 来读。MCU 增益在设备侧参与控制，实际执行结果要看反馈。

启动后可以直接读取状态和对照数据：

```bash
rostopic echo /left_arm/status
rostopic echo /left_arm/result
rostopic echo /left_arm/joint_states
rostopic echo /left_arm/commanded_joint_states
```

操作码、消息字段、时间戳规则和发布示例见 [Topic 接口说明](INTERFACES.md)，参考轨迹的生成方式和力矩计算见[算法详解](SDK_ALGORITHMS.md)。

## 零点标定

先退出双臂控制器或整机 RViz 入口，释放 UDP 端口，再把机械臂摆到机械零位。然后在已经加载工作区的终端中执行：

```bash
# 只检查握手、被动反馈和静止状态
rosrun z1_control z1_calibrate --check-only

# 按提示逐臂确认并标定
rosrun z1_control z1_calibrate
```

程序会读取 [calibration.yaml](z1_control/config/calibration.yaml)，复用通信 YAML 中的地址和本地端口，默认先标右臂、再标左臂。标定请求通过官方接口发送，标定后的六轴反馈由程序独立检查，默认参考值是 `[0, 0, 0, -0.07435, 0, 0] rad`。`--check-only` 不会发送标定请求，也不会校验机械零位姿态。结果含义、单臂选择和配置方式见[双臂零点标定](CALIBRATION.md)。

## 独立 C++ 构建与离线检查

控制内核和维护工具可以脱离 ROS 构建。装好原生依赖后，在仓库根目录执行：

```bash
cmake -S src/z1_ros/z1_control -B /tmp/z1-control-build \
  -DZ1_BUILD_ROS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/z1-control-build -j2
ctest --test-dir /tmp/z1-control-build --output-on-failure
```

catkin 构建会自动打开 ROS 适配；普通 CMake 构建默认关闭 ROS 适配、启用控制内核。如果只想构建 MCU 适配和标定工具，再加 `-DZ1_BUILD_CORE=OFF`。原生 `Config` 的周期默认值是 `0.002 s`；直接调用原生 API 时需要自己传入运行配置，ROS YAML 中的 `0.004 s` 只对 ROS 节点生效。

只依赖官方通信库的接口预检见 [transport_preflight](tools/transport_preflight/README.md)，控制计算的分配检查见 [performance_audit](tools/performance_audit/README.md)。
