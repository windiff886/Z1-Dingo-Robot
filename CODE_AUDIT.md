# 代码审核：删除重复保护，保留真实约束

日期：2026-10-30。

> 历史审计记录：下文 Z1 自研控制、通信和 RViz 条目对应迁移前版本，相关文件可从 Git 历史查看。2026-11-02 起机械臂使用 [src/z1_sdk](src/z1_sdk/README.md) 的官方控制栈；本文不描述当前机械臂实现。

本轮审核覆盖 Z1 控制、规划、通信、标定、ROS/RViz 适配，Dingo 底盘与遥控、云台、相机，以及相关的模型、启动配置、构建和测试。判断依据是调用链、数据来源和实际故障后果：如果内部不变量已经成立，重复检查就可以删除；外部输入和硬件故障则要在进入系统时处理。

## 已删除与简化

| 位置 | 改动 | 依据 |
| --- | --- | --- |
| [controller.cpp](src/z1_ros/z1_control/src/controller.cpp) | 删除第二套 DISABLE 分支 | DISABLE 入队时会清空邮箱并放到首项，`step` 在同一把锁内优先处理它；后面的循环不会再遇到它 |
| 同上 | 删除提交阶段重复的 ID 和 Holding 起点检查 | 接受有限目标和状态切换都会递增 `generation_`，版本检查已经能让并发的旧计划失效；Disabled 下实测起点的漂移仍然单独检查 |
| 同上 | 删除 IK 成功后的重复限位检查，以及轨迹工厂之后的重复空轨迹检查 | IK 只返回限位内的解；轨迹工厂要么返回有效轨迹，要么抛出异常。关节目标输入和轨迹极值检查仍然保留 |
| 同上 | 合并反馈数值检查和内部采样裁剪 | 位置、速度和接收时刻已经有有限的边界比较，可以拒绝 NaN/Inf；内部运动时间由单调控制时刻推进，始终非负 |
| 同上 | 删除静止历史的 1024 帧上限 | 历史会按 `stationary_window` 裁剪。原始速度有噪声、需要用位置窗口确认时，额外的帧数上限会让高频输入攒不齐所需的时长 |
| 同上 | 删除没有被读取的夹爪目标初始化，以及停止时的阻抗参考重算 | 有效目标和模式入口会在使用前重新赋值；Stopping 使用关节控制律 |
| [runtime.cpp](src/z1_ros/z1_control/src/runtime.cpp) | 删除重复的被动输出和退出错误标志 | 通信故障已经交给控制器统一转成 Fault/被动输出，退出循环按模式结束 |
| [模型与轨迹](src/z1_ros/z1_control/src) | 删除重复清零、内部不可达回退和重复数值检查 | 生命周期和调用方已经建立了相应条件；公开 API 的校验和数值溢出检查保留 |
| [model.cpp](src/z1_ros/z1_control/src/model.cpp) | 删除“重力模长不得超过 20 m/s²”的限制 | 刚体动力学计算直接使用配置向量，这个数值上限没有算法依据；重力有限性检查保留 |
| [通信适配](src/z1_ros/z1_control/src/mcu_transport.cpp) | 删除只写不读的 `missed`，并把命令校验限定到实际发送的字段 | Disabled 输出全零被动帧；六轴设备不会发送夹爪字段，使能且带夹爪时仍然校验该字段 |
| [UDP 实现](src/z1_ros/z1_control/src/observed_udp.cpp) | 删除已经建立 Scope 后的重复空指针判断和重复缓冲清理 | 厂商库的握手、收发与标定都在观察 Scope 中执行；回执使用新的有效接收事件和精确长度判定 |
| [calibrate.cpp](src/z1_ros/z1_control/src/calibrate.cpp) | 删除任意的样本数、采样间隔和等待上限 | 参数改为由正值、至少两个样本、时间关系和类型容量来约束；乘法按 64 位计算，毫秒转微秒时会检查类型容量 |
| [RViz 交互](src/z1_ros/unidingo_rviz_test/scripts/interactive_tcp_test.py) | 删除缓存目标的重复校验，收窄 Home 的错误捕获范围 | TCP 和交互反馈入口会先验证位姿；Home 复用已有的错误处理，编程错误继续暴露 |
| [云台驱动](src/pan_tilt_ros/pan_tilt_driver) | 删除构造后的非空判断、析构后的置空、未使用函数，以及串口未打开时的空跑回退 | 对象与线程改用明确的所有权；串口成功打开才完成构造，失败就直接结束启动 |

默认运动参数、控制周期、增益、标定参考值和设备启动动作都保持原值。重力、采样与等待参数的可配置范围扩大了；当前约束见[标定说明](src/z1_ros/CALIBRATION.md)和[算法说明](src/z1_ros/SDK_ALGORITHMS.md)。

## 必须保留的检查

- **外部命令**：消息时间戳、坐标系、四元数、数组维度和数值，用来防止错误目标进入控制计算。原生 C++ 和 ROS 各自都是入口。
- **运动与停止**：关节限位、整段多项式极值、导数限制、候选制动路径、主机力矩及其变化率。目标点合法，不能代替对中间路径和停止路径的检查。
- **反馈与通信**：反馈时龄、电机状态、UDP 来源、长度、CRC、发送错误和积压处理。网络故障、设备错误和旧反馈都可能在运行中发生。
- **命令生命周期**：ID、会话、序号、规划状态版本、实测到达与停止恢复。它们分别处理重复目标、旧流式帧、并发规划和实际运动完成。
- **数值算法**：IK 收敛与分支检查、SO(3) 小角度展开、导数溢出与非有限力矩处理。它们对应算法可能遇到的数值条件。
- **相机和云台数据**：相机时间域、畸变模型、无效深度与量程转换，云台报文、角度和速度限制。它们定义了设备数据和命令的含义。
- **有界队列与等待**：参考点、结果事件、UDP 队列和通信超时限制资源占用，避免发布者或设备异常拖住控制线程。
- **键盘的两条退出路径**：保留 `rospy.on_shutdown(stop)` 和 `finally`。前者在 ROS 内部关闭前发送零速度，后者处理循环退出与异常并恢复终端；[rospy 的关闭顺序](https://github.com/ros/ros_comm/blob/noetic-devel/clients/rospy/src/rospy/core.py#L534-L579)决定了两者不能互相替代。

## 一并修复的真实问题

### 云台退出先结束采集，再释放串口

原来的析构函数只改变循环标志，随后就删除串口对象，此时采集线程可能还在读。现在改用原子停止标志，等待线程 `join()` 之后再释放串口，消除了生命周期竞争。

### 底盘无线诊断处理系统查询失败与无地址记录

`getifaddrs()` 失败时，原来的代码会遍历未初始化的指针；成功返回的接口记录也可能没有 `ifa_addr`。现在失败时会记录系统错误并发布断开状态，遍历时跳过无地址记录，成功路径还会释放链表。

## 控制内核移除几何碰撞功能

SDK 负责生成和执行运动参考，几何碰撞检测则由需要该能力的调用方在上层接入。这次删除了控制模型中的碰撞对象、碰撞对、固定盒、AABB 缓存、距离求解器以及 `collision.cpp`，控制周期只更新 TCP，不再查询几何距离。

- 移除了 ROS 几何配置读取，以及 YAML 中的 `ignored_collision_pairs`、`fixed_boxes`；原生几何配置与查询 API 也同步移除。迁移入口见[模型配置](src/z1_ros/INTERFACES.md#6-模型配置)。
- 保留运动学、动力学、关节限制、制动验证、力矩限制、反馈和通信检查；URDF/Xacro 中的碰撞描述留给规划和仿真。
- 保留 ROS `Status` 的字段类型与顺序，旧距离字段返回 `NaN` 和空名称；原生 `Snapshot` 使用相同含义。
- 模型指纹改为控制 URDF 全文、TCP 名称和重力向量。升级后请重新记录指纹。
- 删除了用于旧几何实现性能对照的独立 Coal 距离工具及其构建入口；控制计算分配工具继续测量 FK、雅可比、RNEA 和控制步骤。

新增了 `model_only` 回归：分别删除全部碰撞描述、引用不存在的碰撞网格，然后对照原模型的限位、FK、雅可比、RNEA、ABA、目标执行和受控停止，结果一致。

## RViz 改用官方交互组件

日期：2026-10-31。当前的目标状态、IK、手柄、虚影、场景坐标系和轨迹播放都直接复用官方 MoveIt 实现，替代了之前自写的交互链路。双臂、Home、SDK 参考预览和明确发送 `PoseGoal` 仍然保留在适配层。

### 固定源码基准

- 宇树 ROS 1 Noetic：[z1_ros，d3bafd38fb48e805096da7abb0c19a15b636975d](https://github.com/unitreerobotics/z1_ros/tree/d3bafd38fb48e805096da7abb0c19a15b636975d)。[官方 README](https://github.com/unitreerobotics/z1_ros/blob/d3bafd38fb48e805096da7abb0c19a15b636975d/README.md)区分了这个 ROS 驱动和独立的 `unitreerobotics/z1_sdk`，两者不能混用硬件执行接口。
- 官方模型：该提交引用的 [unitree_ros，8ebecd883a842a4631fcfa98933fae586aeb6709](https://github.com/unitreerobotics/unitree_ros/tree/8ebecd883a842a4631fcfa98933fae586aeb6709/robots/z1_description)。
- 交互 API：[MoveIt 1.1.16](https://github.com/moveit/moveit/tree/1.1.16)。部署机上实际安装版本是否构建兼容，仍然需要在 ROS 1 环境中检查。

[官方启动入口](https://github.com/unitreerobotics/z1_ros/blob/d3bafd38fb48e805096da7abb0c19a15b636975d/z1_bringup/launch/real_arm.launch)会通过 `z1_rviz/launch/view_robot.launch` 加载 [z1_rviz/rviz/robot.rviz](https://github.com/unitreerobotics/z1_ros/blob/d3bafd38fb48e805096da7abb0c19a15b636975d/z1_rviz/rviz/robot.rviz)，并使用 `MotionPlanning`。本次按这个实际入口来对照，而不是另一份 `z1_moveit_config/launch/moveit.rviz`。

### 函数与配置逐项对应

| 官方职责与源码 | 当前实现 | 保留的适配 |
| --- | --- | --- |
| [MotionPlanningDisplay::onRobotModelLoaded](https://github.com/moveit/moveit/blob/1.1.16/moveit_ros/visualization/motion_planning_rviz_plugin/src/motion_planning_display.cpp) 创建 RobotInteraction 和目标状态 | [SdkMotionPlanning](src/z1_ros/unidingo_rviz_test/src/sdk_motion_planning.cpp) 继承该类并调用上游实现 | 适配层保留左右臂切换、Home 和目标状态消息 |
| [InteractionHandler::handleEndEffector/updateStateEndEffector](https://github.com/moveit/moveit/blob/1.1.16/moveit_ros/robot_interaction/src/interaction_handler.cpp) 更新目标 IK 和虚影 | 使用原生处理，不再创建 Python InteractiveMarkerServer 或独立的目标虚影 | 回调继续调用原来的 `scheduleDrawQueryGoalState`，只追加 SDK 位姿导出 |
| 官方 [kinematics.yaml](https://github.com/unitreerobotics/z1_ros/blob/d3bafd38fb48e805096da7abb0c19a15b636975d/z1_moveit_config/config/kinematics.yaml) | 两组都使用 TRAC-IK、0.005 的搜索分辨率和 0.005 s 超时 | `manipulator` 映射为 `left_arm` / `right_arm` |
| [PlanningSceneDisplay::calculateOffsetPosition](https://github.com/moveit/moveit/blob/1.1.16/moveit_ros/visualization/planning_scene_rviz_plugin/src/planning_scene_display.cpp) 定位整个场景 | 沿用原生场景节点和轨迹播放器 | 整机根为 `base_link`；删除了自行设置 Ogre 根位姿的 `FramedTrajectory` |
| 官方 SRDF 命名状态 | [dual_arm.srdf](src/z1_ros/unidingo_rviz_test/config/dual_arm.srdf) 保留 Home 六轴值 | 为两臂加前缀，把链末端延伸到固定的 SDK `tool`；通过 `setToDefaultValues(jmg, "home")` 选择 |
| 官方运动学、规划上下文与场景监视 | [preview_move_group.launch](src/z1_ros/unidingo_rviz_test/launch/preview_move_group.launch) 加载 SRDF、IK、官方规划限值和 OMPL 上下文 | 关闭轨迹执行，不启动 `z1_hw` 或 ros_control；播放器接收 SDK 参考，不接收 OMPL 结果 |
| [官方 executeButtonClicked](https://github.com/moveit/moveit/blob/1.1.16/moveit_ros/visualization/motion_planning_rviz_plugin/src/motion_planning_frame_planning.cpp) 立即禁用执行按钮 | TCP targets 的 SEND 会立即禁用；适配器用非零 `sent_id` 保持单条在途命令 | 硬件出口仍然是现有的 `PoseGoal` 和控制话题，原生 MoveIt 执行面板被隐藏并禁用 |
| [CurrentStateMonitor](https://github.com/moveit/moveit/blob/1.1.16/moveit_ros/planning/planning_scene_monitor/src/current_state_monitor.cpp) 按消息源时间判断状态时效 | 原生监视器处理 MoveIt 状态；SDK 适配器也会检查关节消息自身的时间戳 | 保留源时间戳；延迟消息的剩余寿命不会因为刚刚收到而重新计时 |
| 官方模型速度与基础惯性 | [z1_control.urdf](src/z1_ros/z1_description/urdf/z1_control.urdf) 的六轴上限恢复为 3.1415 rad/s，基础惯性一致 | 默认控制配置仍然是 0.8 rad/s；MoveIt 规划限值按官方配置使用 3.14 rad/s |
| 官方原装夹爪惯性 | 控制模型保留定子、动子的完整质量、质心和惯性张量，移除了 0.25 kg 的工具近似 | 六轴动力学把夹爪固定在闭合零位；`tool` 是无质量 TCP frame |

### 删除的重复实现与修复

- 删除了自建手柄、菜单、拖动状态、TF 转换和单独的青色目标虚影消息；不再使用两份 `*_preview_` 显示模型或空 SRDF。
- 删除了 `FramedTrajectory` 和它的第二条轨迹订阅，并清除了预览中直接调用原生播放器 `reset()` 的代码。
- 删除了预览服务中未被使用的终点数组、逐点 TCP 路径和 Home 发布器。Home 由官方 SRDF 命名状态提供。
- 修复了重复 SEND 覆盖 `sent_id`、旧关节消息成为新起点这两个问题。Home、实测复位和选臂请求在等待编辑器回执时，都会立即使旧计划失效；旧 RPC 结果不能重新启用 SEND。成功复位带有原生编辑器版本条件，避免覆盖仍在传输中的新编辑。
- CMake 继续保留 SDK 纯 C++ 消费者配置回归和公共 C++17 声明；SDK 与 `z1_msgs` 都没有新增 MoveIt 依赖。

### 原装夹爪与六轴接口的明确边界

用户已经确认两臂安装的是宇树原装夹爪。采用[官方 const.xacro](https://github.com/unitreerobotics/unitree_ros/blob/8ebecd883a842a4631fcfa98933fae586aeb6709/robots/z1_description/xacro/const.xacro)的定子质量 0.52603655 kg、动子质量 0.27621302 kg，合计 0.80224957 kg，并保留各自的质心和惯性张量。

控制接口仍然是六轴，所以把官方夹爪关节固定在闭合零位，而不是增加第七个手臂控制坐标；开合时不会在线改变惯性。这是一个静态闭合夹爪模型。原生回归重新恢复了第七个夹爪关节，在它的位置、速度、加速度都为零时，对照 250 组手臂配置的 RNEA 力矩，六轴结果一致。

默认 TCP 仍然相对 `link06` 沿 X 平移 0.051 m。Home 关节值没有变化，`tool` 的 Home 位置约为 `[0.037730, 0, 0.173143] m`。MoveIt 目标臂使用 TRAC-IK；SDK 执行参考仍然由原来的 IK + Ruckig 生成，两者的关节解可以不同，但目标 TCP 位姿相同。橙色目标臂和淡紫色 SDK 参考在界面与说明中分别命名。

### 当前证据与部署机验收

- 源码对照确认六轴原点、轴向、位置上下限、effort 和基础惯性与官方模型一致；25 组配置的 `link06` FK 一致。
- 模型离线回归现在会同时比较 velocity、SRDF Home、TCP 和原装夹爪的各项惯性，替代了只比较位置上下限的旧覆盖。
- 当前 10 组原生控制与数值回归通过；增强的模型参考回归还验证了官方速度上限拒绝，以及七坐标闭合夹爪的动力学等价性。
- 30 项发送适配、8 项接线、8 项模型和 10 项启动配置离线检查通过。
- 本机没有启动 ROS/RViz。ROS 1 catkin 构建、插件加载、原生拖动手柄、选臂和 Home/复位交互仍然需要由部署机验收；新夹爪惯性请先以夹爪闭合状态确认重力补偿和保持表现，步骤见[测试工具说明](src/z1_ros/unidingo_rviz_test/README.md)。

## 验证结果

- Pinocchio 4.0、Ruckig 0.9.2 下原生构建通过。
- 10 组控制与数值回归通过，覆盖无碰撞模型、高密度静止窗口、NaN/Inf 反馈、DISABLE 优先级、共享有限 ID 和退出时通信异常。高密度窗口用例在删除上限之前能复现失败，修改后通过。
- UDP 回环、26 项模拟标定和官方库内存通信预检通过。
- 本次启动配置 10 项、模型 8 项、RViz 发送适配 30 项及接线配置 8 项离线测试通过；前一轮键盘用例包含发布通道关闭前发送零速度。
- 独立 SDK 消费者与分配工具的 CMake 配置检查通过。
- `allocation_audit` 的旧模型历史测量结果：稳定 JointStream 统计窗口中的普通 `new/new[]` 为 0 次，动态窗口均值为 0.12 次/步、最大 1 次。
- 云台线程退出与串口启动失败测试通过，线程退出另外经过 ASan/UBSan 检查。
- 无线诊断夹具编译了实际回调，覆盖系统查询失败、空地址、IPv6 和非 IP 记录，并核对了发布与释放行为。

构建并执行原生回归：

```bash
cmake -S src/z1_ros/z1_control -B /tmp/z1-code-audit-build \
  -DZ1_BUILD_ROS=OFF -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/ros/humble
cmake --build /tmp/z1-code-audit-build -j2
ctest --test-dir /tmp/z1-code-audit-build --output-on-failure \
  -E 'mcu_udp_loopback|dual_arm_calibration'
```

`CMAKE_PREFIX_PATH` 指向本机原生依赖前缀。通信用例使用本地 MCU 模拟器；工具入口见[通信接口预检](src/z1_ros/tools/transport_preflight/README.md)。

本机提供的是 ROS 2 Humble，完整的 ROS 1 catkin 构建和真实设备动作都没有运行。部署时请在 ROS 1 环境中完成整机构建，再验证真实串口、RViz 交互和运动闭环。
