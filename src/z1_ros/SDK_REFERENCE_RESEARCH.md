# 机械臂控制 SDK 参考评估

调研日期：2026-10-11；默认控制频率说明更新于 2026-10-23。用于支持 [Z1 重构需求](REFACTOR_REQUIREMENTS.md)，不替代需求访谈结论。本文原始参考筛选与后续实现复验分别记录，当前实现进展见 [实施记录](IMPLEMENTATION_REPORT.md) 第 7、8 节。

## 1. 评估目标

已确认的目标是独立控制内核、ROS 1 launch/topic 接口、关节与末端的有限目标运动及流式跟踪。暂保留官方 Z1 MCU 通信适配，因此寻找的是可借鉴的运动管理、算法和执行设计；其他品牌的 SDK 不能直接替代 Z1 驱动。

当前主要参考方向已收敛为 **Franka 官方 libfranka**。第三方 franky 作为其上层运动管理的补充；下文其他项目保留为专题比较资料，不要求同时引入所有框架。新的参考基线见第 9 节。

本次依据项目维护者和厂商公开文档、接口说明与仓库结构进行筛选，未完成这些项目的本地构建、性能基准或真机验证。“适合作为参考”不表示其整套实现可以移植到 Z1。

## 2. 推荐参考

| 项目 | 定位与可借鉴能力 | 对本项目的价值 | 适用边界 |
| --- | --- | --- | --- |
| [franky](https://github.com/TimSchneider42/franky) | C++ / Python 运动控制库；Ruckig 平滑轨迹、在线重规划、抢占与事件反应 | 优先参考关节/末端运动管理和非实时应用到实时执行的分层 | 底层依赖 Franka；不能直接换成 Z1 IP 使用 |
| [libfranka](https://github.com/frankarobotics/libfranka) | 官方低层客户端；随新状态执行控制回调、提供实际周期并报告控制/网络/实时错误 | 优先参考执行循环、时效约束和故障契约 | 1 kHz 是 Franka 的控制接口指标，不是 Z1 指标 |
| [Universal Robots Client Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library) | C++ 通信库；提供有限轨迹及开放式轨迹流的示例 | 参考通信与执行接口隔离、流的结束及欠载处理 | 协议与设备控制器是 UR 专用；轨迹流不等于本项目的最新目标跟踪 |
| [Flexiv RDK](https://github.com/flexivrobotics/flexiv_rdk) | 厂商 C++ / Python 开发包；区分实时与非实时模式，提供关节/笛卡尔运动与力相关接口 | 参考控制模式划分、输入节奏与机器人内部控制职责 | 文档与示例可参考，不认定内部控制算法实现可直接复用 |
| [Pink](https://github.com/pink-kinematics/pink) | 基于 Pinocchio 与 QP 的差分逆运动学算法库 | 参考带配置/速度限制的末端流式跟踪、任务权重与姿态任务 | 不是完整硬件 SDK；Python 实现不构成本项目高频 C++ 循环的性能保证 |

## 3. 补充参考：franky 的运动层

公开文档当前标注为 2.0.0。它支持关节与笛卡尔的位置/速度运动，并用 Ruckig 处理速度、加速度和 jerk 限制；允许运动途中更新并重规划目标。这些能力与本项目两类流式位置目标直接相关。[项目说明](https://github.com/TimSchneider42/franky)、[轨迹控制说明](https://timschneider42.github.io/franky/guide/trajectory-control.html)。

建议参考其运动对象、运动替换、停止与回调的组织方式，不照搬 Franka 特定的关节数量、机器人状态结构或内部笛卡尔控制接口。Z1 的末端跟踪仍需要我们自己的逆运动学和关节约束处理。[运动执行说明](https://timschneider42.github.io/franky/guide/executing-motions.html)。

franky 也提供阻抗控制接口，可参考其中不要求额外力传感器输入的控制结构。用户已明确不使用力传感器；依赖特定测力硬件或外力测量的算法不纳入本轮。Z1 力矩执行与补偿职责仍须验证。[阻抗控制说明](https://timschneider42.github.io/franky/guide/impedance-control.html)。

这一参考说明 Ruckig 本身不能仅因参考 SDK 较老就被判定为过时：更值得改变的是其目标更新、状态连续性、限制配置和错误处理方式。此结论是基于 franky 当前公开功能作出的设计判断，不是两者性能对比实验。

## 4. libfranka 的执行层

官方接口以收到的机器人状态驱动回调，同时传入实际回调时间间隔；支持不同命令类型并明确报告网络、控制和实时调度错误。Franka 的控制接口运行在 1 kHz。[官方 Robot API](https://frankarobotics.github.io/libfranka/latest/classfranka_1_1Robot.html)。

建议借鉴“测量、周期与控制结果构成明确契约”的设计，避免循环计时到了就认为收到新反馈，也避免只保存一个持续刷新但来源不明的状态缓存。这是对 Z1 内核的建议，仍需结合保留的通信适配能提供的信息实现。

不能把其控制周期或错误恢复方法直接复制为 Z1 的硬件承诺；固件、通信时延和失联行为必须验证。

## 5. 末端跟踪算法：Pink

Pink 将末端位姿等目标表示成任务，通过差分 IK 求关节速度，并加入配置与速度限制。持续更新任务目标时，它可用于持续跟踪；它是局部算法，不保证全局 IK 解或绕过障碍。[维护者项目说明](https://github.com/pink-kinematics/pink)。

相比参考 SDK 的无位置限位雅可比更新，本项目可以评估带约束差分 IK、合适的旋转误差表示和可配置任务权重。但不能只在 IK 后添加一次轨迹平滑，就宣称末端跟踪误差与所有约束仍有保证；需要验证整个控制链路。

建议借鉴其数学建模，再评估 C++ 实现与求解器选择，不将 Python 调用直接放进高频执行线程。求解失败、计算超时、近奇异情况与目标跳变都需要可观察的处理策略；具体策略在需求访谈中确定。

## 6. 通信与模式参考

UR 的轨迹流示例将有限轨迹与不预知总点数的开放式流区分开；其通信架构说明了流结束与欠载失败的区别。可借鉴这些执行语义，但本项目的流式目标模式不因此改成无界队列。[官方轨迹流示例](https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/trajectory_streaming.rst)、[通信接口说明](https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/architecture/reverse_interface.rst)。

Flexiv RDK 区分实时与非实时模式，公开实时模式的 1 kHz 通信指标，并明确部分动力学补偿由控制系统承担。适合参考模式与补偿职责的清晰划分；需要厂商测力硬件或专用外力测量的控制能力按本轮约束排除。[官方控制模式文档](https://flexiv.us/software/rdk/manual/v2.x/control_modes.html)。

## 7. 对 Z1 重构的建议

以官方 libfranka 为执行与阻抗控制的主要参考，以第三方 franky 补充在线运动管理；Pink、UR / Flexiv 仅保留为局部算法或接口行为的补充比较。该排序按用户最新要求收敛，不等于已选择这些项目全部作为运行依赖。

控制频率应拆开测量：上层输入、IK/轨迹更新、内核执行、MCU 收发及状态发布。2026-10-11 的 250 Hz 配置及 2026-10-23 首轮性能升级的 250 Hz 数据保留为历史基线；2026-10-23 后续实现已将原生默认周期推进至 2 ms（500 Hz），ROS 状态发布默认 100 Hz。该软件配置不代表 Z1 MCU 的采样和执行频率已经得到确认，也不代表已在机器人上的 i7-13700H 部署机验证。验收须在同一部署机、同一机器人和相同任务下比较 250/500 Hz 的跟踪与扰动响应，同时测量有效新反馈率、周期抖动、最坏计算时间、丢包和测量时龄；其他品牌 SDK 的频率只作为架构参考。当前默认预算与调度实现见 [实施记录](IMPLEMENTATION_REPORT.md) 第 8 节。

用户已明确不使用力传感器，也不需要外力估计。首期范围为位置跟踪与无需外力反馈的笛卡尔阻抗；外力估计、基于外力估计的碰撞诊断与接触力闭环均排除。阻抗可根据位姿/速度误差和模型计算力矩，不需要先估计外力，控制结构可参考 [libfranka 官方示例](https://frankarobotics.github.io/libfranka/0.18.2/cartesian_impedance_control_8cpp-example.html)。

### 7.1 为什么控制源码不能原样替换硬件

以 Franka 为例，官方状态接口区分总关节力矩 `tau_J`、去除机器人与负载模型影响的外力矩估计 `tau_ext_hat_filtered`，并明确期望力矩 `tau_J_d` 不包含重力。Z1 的现有头文件只显示其自身的关节力矩与命令字段，不能直接按同名或近似字段替换。动力学补偿与外力反馈语义必须适配。[官方状态定义](https://frankarobotics.github.io/libfranka/latest/structfranka_1_1RobotState.html)。

因此，参考 SDK 可帮助复用控制结构、数学方法和模式设计；Z1 仍需确认自己的反馈来源、补偿职责、负载、增益、限制和周期。本轮筛选时排除依赖额外力传感器或特定厂商测力能力的算法，不将 Franka 的外力矩估计字段直接当作 Z1 可用输入。

## 8. 碰撞防护实现的参考

用户已确认首期建立基于模型的几何碰撞防护，并希望通过模型调整适配结构变化。以下区分公开 SDK 接口、机器人内部能力和可读的算法实现；不将它们一概称为可直接移植的“防碰撞算法”。

| 参考 | 公开实现或行为 | 对 Z1 的启示 |
| --- | --- | --- |
| Franka / libfranka | 内部接近自碰撞配置时报告 `self_collision_avoidance_violation`；另有基于估计外力/外力矩与阈值的接触或碰撞反应 | 参考自碰撞错误与运行停止契约；外力估计反应按本轮约束排除；内部自碰撞算法不认定为 SDK 中可读、可移植的实现 |
| franky | 可注册实时条件并触发停止或替代运动；公开示例包含检查机器人自碰撞错误标志 | 参考“风险事件 → 明确运动反应”的组织方式；该示例不证明 franky 自行加载任意 URDF 并提供全局绕障 |
| Universal Robots | 控制器提供关节限制、安全平面、工具姿态/速度等边界；部分边界处理考虑停止距离 | 参考可配置空间边界及制动提前量；安全平面不等于任意网格碰撞场景或自动绕障，不认定 SDK 提供全部内部算法源码 |
| Pinocchio + Coal | Pinocchio 提供模型与碰撞几何集成；Coal 提供几何碰撞、距离及间距查询 | 直接作为 Z1 独立内核的几何检查候选基础，不依赖 MoveIt 或外力反馈；不是完整运动规划器 |
| Mink | `CollisionAvoidanceLimit` 将几何对之间允许的法向接近速度写成 QP 不等式，配置最小距离与激活距离 | 参考流式末端跟踪的局部约束方法；MuJoCo/Python 实现是方法参考，不直接替换 Pinocchio/C++ 内核，也不保证找到全局绕障路径 |

Franka 官方文档明确区分自碰撞错误与基于估计外力的反应，并说明自碰撞错误并不保证任意速度和配置下都能避免碰撞。SDK 暴露这些行为不代表内部算法完整开源。[FCI 官方概述](https://frankarobotics.github.io/docs/doc/libfranka/docs/overview.html)。franky 的反应机制能根据状态条件停止或切换运动；其中力阈值例子不属于本项目选定方案。[franky 实时反应](https://timschneider42.github.io/franky/guide/reactions.html)。

UR 官方说明的安全平面可限制工具或工具与肘部，关节位置、安全平面等限制考虑预期停止距离。它说明“预留制动空间”与边界配置的价值，不支持推断存在面向任意 URDF 的公开网格绕障算法。[UR 可配置安全功能](https://www.universal-robots.com/manuals/EN/HTML/SW5_25/Content/prod-usr-man/complianceUR12e/H_g5_sections/safetyFunctionsAndinterfaces/configurable_functions.htm)。

Pinocchio 维护者说明其依赖 Coal 做碰撞计算；Coal 提供 GJK/EPA 等几何算法及碰撞间距支持。建议复用这些底层查询，将模型和碰撞规则配置与执行策略分开。[Pinocchio](https://github.com/stack-of-tasks/pinocchio)、[Coal](https://github.com/coal-library/coal)。具体版本、官方动态库 ABI 与性能须在实现前确认，不能只凭候选库功能认定依赖兼容。

Mink 的距离限制在接近激活范围时限制两物体继续接近的法向速度，适合借鉴局部流式控制。使用这种约束后仍需验证最终执行命令，包括后续平滑是否改变原本可行的运动；力矩阻抗与关节位置生成也不能共用未经验证的处置假设。[Mink 碰撞限制 API](https://kevinzakka.github.io/mink/api/limits.html)。

结论：用户已接受首期模型驱动的几何距离检查、候选运动验证与减速/停止机制，主动改变方向继续运动延后。更换几何、工具和安装变换可主要通过模型/配置完成；增加动态物体来源、跨臂状态协调或全局路径搜索仍需扩展算法与接口。Flexiv 现有公开 RDK 材料不足以在本次调研中确定其内部自碰撞算法，暂不作为这部分的源码参考。

### 8.1 检测风险后的具体行为

风险处置不是所有 SDK 的统一行为，需要区分执行保护、应用反应和局部运动求解：

- **Franka / libfranka**：自碰撞边界触发错误；力反馈反应则区分较低接触阈值和较高碰撞阈值，前者可只报告接触而继续运动，后者触发停止并锁存至恢复。这些接口不等于自动搜索绕障路线。本项目只借鉴执行错误与恢复的契约，不纳入力反馈反应。[FCI 概述](https://frankarobotics.github.io/docs/doc/libfranka/docs/overview.html)、[Robot API](https://frankarobotics.github.io/libfranka/latest/classfranka_1_1Robot.html)。
- **franky**：由应用注册条件和反应运动，官方自碰撞错误示例选择 `StopMotion`，另一些示例会替换运动；替代运动的逻辑由应用给出，不是框架自动发现障碍并保证绕开。已经由机器人控制器触发的停止/错误，也不能假设可直接用普通反应运动继续运行。[实时反应示例](https://timschneider42.github.io/franky/guide/reactions.html)。
- **UR 控制器**：关节位置、安全平面等边界考虑停止距离，接近限制时降低速度；它提供的是对已配置边界的约束，不会因此自动给任意目标寻找另一条路径。[可配置安全功能](https://www.universal-robots.com/manuals/EN/HTML/SW5_22/Content/prod-usr-man/complianceUR20/H_g5_sections/safetyFunctionsAndinterfaces/configurable_functions.htm)。
- **MoveIt Servo**（控制软件参考，非厂商硬件 SDK）：接近自碰撞或环境碰撞时缩小速度，公开状态区分因碰撞减速与停止。这与首期模型检查和提前停止的建议相关，借鉴其行为不要求重新引入 MoveIt 依赖。[Servo 设计说明](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)、[状态定义](https://moveit.picknik.ai/humble/api/html/namespacemoveit__servo.html)。
- **Mink**（算法库参考，非厂商硬件 SDK）：把物体接近限制写入 QP，与目标跟踪一起求解允许的局部运动。解可改变原本希望的关节运动，但不保证目标一定能到达；求解失败和执行停止仍需要调用方处理。[碰撞约束](https://kevinzakka.github.io/mink/api/limits.html)。

因此，“先限速，必要时停止”是这些公开控制接口中可直接参考的行为；主动改变方向属于额外的局部求解能力，完整绕障属于路径搜索能力。三者可以分层增加，不以是否具有全局绕障判断底层控制接口是否先进。用户已选择 Z1 首期减速/停止方案；具体阈值、恢复及阻抗模式停止转换仍需定义。

另需核对参考代码的离散时间语义：Mink 维护者的未发布变更记录包含碰撞约束步长和激活距离的修正，说明只复制公式或接口名仍可能得到不同的保护行为。若参考该方法，必须固定版本并验证实际时步、离散位移限制及激活范围，而非直接复制其默认参数。[维护者变更记录](https://github.com/kevinzakka/mink/blob/main/CHANGELOG.md)。

## 9. Franka 作为主要参考的基线

用户建议直接采用 Franka SDK 为参考，以减少逐个决定常规控制细节。这里以厂商维护的 [libfranka](https://github.com/frankarobotics/libfranka) 为主要入口，并将 [franky](https://github.com/TimSchneider42/franky) 明确标注为独立维护的第三方上层库。

### 9.1 代码阅读与适配对应

| 需求 | 官方阅读入口 | Z1 设计中需要完成的适配 |
| --- | --- | --- |
| 控制循环与周期 | `include/franka/robot.h`，官方 `Robot::control` 与主动控制接口 | 独立控制内核驱动 Z1 通信，区分执行周期与新反馈周期，不能复制 Franka 1 kHz 为硬件承诺 |
| 命令与状态 | `include/franka/control_types.h`、`robot_state.h`、`errors.h` | 按 Z1 六个关节与实际反馈字段定义内核类型；不要引入外力估计字段或七轴固定数组假设 |
| 关节运动 | `examples/generate_joint_position_motion.cpp`、`examples/examples_common.h` | 参考目标到执行的链路；示例运动生成器不等于完整在线命令替换、目标超时与队列管理 |
| 外部执行循环 | `examples/generate_joint_position_motion_external_control_loop.cpp` | 参考读状态、获取实际周期、计算与发送命令的组织方式；MCU 适配能提供的信息须独立验证 |
| 关节阻抗与变化率限制 | `examples/joint_impedance_control.cpp` | 核实反馈、前馈与 MCU 增益的职责，避免双重 PD 或动力学补偿；限值采用 Z1 验证后的配置 |
| 笛卡尔阻抗 | `examples/cartesian_impedance_control.cpp` | 用 Pinocchio/Z1 模型计算 FK、J 与所需补偿；使用状态误差控制，不依赖外力测量或估计 |
| 模型与负载 | `include/franka/model.h`、`Robot::setLoad` 等接口 | 参考质量、质心、惯量与安装参数的显式配置；Franka 模型和补偿实现不作为 Z1 模型 |
| 停止与错误恢复 | `Robot::stop`、`MotionFinished`、`automaticErrorRecovery` 与错误定义 | 分清运动结束、停止、故障与恢复；流式超时显式恢复采用本项目已确认契约，不能被示例行为覆盖 |

以上是已定位的公开阅读入口和适配任务清单，尚未完成所有文件的逐行源码审计、本地构建或运行验证。主要来源：[官方 Robot API](https://frankarobotics.github.io/libfranka/latest/classfranka_1_1Robot.html)、[官方关节目标示例接口](https://frankarobotics.github.io/libfranka/0.19.0/examples__common_8h.html)、[笛卡尔阻抗源码](https://github.com/frankarobotics/libfranka/blob/main/examples/cartesian_impedance_control.cpp)。完整实现阅读应固定参考 tag/commit，并记录结论对应版本。

### 9.2 主参考未直接解决的需求

libfranka 官方定位是 Franka 实时底层客户端，不能据此认定它已提供 Z1 所需的通用 IK、任意 URDF 场景的几何防护，或完整的上层目标管理。Franka 内部提供的控制和保护能力，也不能因为 SDK 有对应接口就认定实现源码全部存在于客户端仓库。

在线目标更新与 Ruckig 管理可补充阅读 franky；Z1 模型与几何继续由 Pinocchio、兼容的 Coal 等工具实现。几何风险处置沿用用户确认的减速/停止，不增加外力反馈，不恢复 MoveIt 为首期必需依赖。[libfranka 官方定位](https://github.com/frankarobotics/libfranka)、[franky 运动层说明](https://github.com/TimSchneider42/franky)。

### 9.3 后续需求收敛方式

先按主参考形成一份完整默认设计，说明选用的行为、必要的 Z1 适配与仍存在的验证条件。常规实现方式由代码研究给出建议；新增功能、改变已确认契约或扩大硬件替换边界的事项继续由用户确认，不再把每个内部实现细节都转换成访谈问题。

此前已确认的六轴 Z1、单臂独立进程、ROS 1 topics、无外力估计、模型防护减速/停止、流式超时显式恢复等约束优先于参考 SDK 行为。当前有限目标新命令替换、夹爪、仿真等未选范围仍不视为已确定，最终需求文档需明确标识默认建议与已确认要求。
