# Z1 Topic 接口说明

左、右臂分别使用 `/left_arm` 和 `/right_arm` 命名空间。下文使用相对话题名，例如左臂的 `joint_goal` 实际是 `/left_arm/joint_goal`。消息定义见 [z1_msgs/msg](z1_msgs/msg)，启动方式见 [README.md](README.md)。

## 1. 通用约定

### 1.1 单位与坐标系

- 关节顺序固定为 `joint1..joint6`；角度单位是 rad，角速度单位是 rad/s，力矩单位是 N·m，位置单位是 m。
- TCP 位姿的四元数按 `(x, y, z, w)` 填写，并保持单位长度；适配器允许模长偏差不超过 `0.001`，接收到之后会做归一化。`frame_id` 使用本臂基座 `<prefix>link00`，例如 `left_link00`。如果目标定义在其他坐标系下，请先转换再发送，控制器不会查询 TF。
- 默认 TCP 名为 `tool`：它从 `link06` 沿 X 正方向平移 `(0.051, 0, 0)` m，姿态不变，夹爪开合也不会影响它。
- `joint_states` 中的关节名带有前缀。检测到夹爪时，会追加 `<prefix>gripper_motor`，这是夹爪电机的原始反馈；描述模型里的夹爪连杆角是另一套量。

### 1.2 时间戳、时钟与队列

- 有限目标、`control`、`impedance_gains` 和 `gripper_goal` 的 `header.stamp` 可以留零；流式消息则必须使用非零时间戳。对于非零时间戳，默认允许它位于当前 ROS 时刻之前 200 ms、之后 20 ms 的范围内，过去的时龄上限由 `command_max_age` 配置。STOP 和 DISABLE 会跳过时间戳检查。
- 流式输入的 `header.stamp` 表示上游生成该消息的时刻。适配器会把源时刻和路点时刻映射到单调时钟；延迟已经消耗掉的那部分有效期不会重新补满，只有有效帧才会续期。
- 内核按单调时钟推进轨迹和期限。ROS 时间只影响新消息的时龄检查；已经映射到内核的期限仍然按单调时钟执行。
- 输出消息的 `header.stamp` 是发布时刻。实际接收序号和时龄需要查看 `status`；固件没有提供采样时刻。
- 订阅队列长度：普通目标和流式消息为 1，`joint_reference` / `pose_reference` 为 32，`control` 为 16。原生时间线最多保留 32 个点，其中包含插值所需的前驱点。如果多个流式发布者共用同一个会话，序号和目标时间需要由上层自行协调。

### 1.3 运动边界与几何规划

控制器负责执行关节位置、速度、加速度、jerk、主机力矩及其变化率的限制，并检查反馈、电机状态和关节制动路径。几何碰撞检测和绕障属于上层规划：控制内核不创建碰撞对象，也不加载碰撞网格。

`joint_limit_tolerance` 默认为 `0.001 rad`，用来给编码器在模型限位附近的偏差留出余量。反馈超出这个容差时会被判为无效；通过检查后，FK 和动力学使用原始 q/dq，状态发布也保留原始测量。用实测位置建立规划或保持参考时，会把容差内的越界量投影回模型限位；目标、轨迹多项式和制动路径则按正式限位检查。独立调用 `Model::ik` 时，初值允许同样的 1 mrad 偏差，迭代前会先投影到限位内，最终解仍满足模型限位。

静止判定分为两档：`start_velocity_tolerance` 用于 Disabled 起步、ENABLE、HOLD 和恢复，`velocity_tolerance` 用于到达和停止完成，两者的默认值都是 `0.03 rad/s`。

如果已经拿到两帧新反馈，只要原始速度和相邻位置差分速度都低于阈值，就算通过。如果原始速度出现尖峰，则改用至少 `stationary_window`（默认 100 ms）的新位置历史来判断：窗口位置范围除以实际时长，以及最近的相邻位置差分速度，都必须低于阈值。位置范围取最大值减最小值，因此可以识别窗口内的往返运动。如果只有一帧位置，就按原始速度判定。反馈无效时会清空历史；重复使用旧反馈不会增加窗口时长。

夹爪原始反馈的测量范围允许为 `[-1.02, 0.02] rad`，而命令和用户目标限制在 `[-1, 0] rad`。保持或生成夹爪参考时，程序会先把测量值投影到命令范围，再做限速。传输异常会把底层原因写进 `status.reason` 和失败命令的 `result.reason`，因此可以分清是命令校验失败、发送失败还是接收积压。

## 2. 命令接口

| Topic | 消息 / 字段 | 接收条件与约束 |
| --- | --- | --- |
| `joint_goal` | JointGoal：command_id、positions[6]、speed_scale | ID 必须为正数且递增；speed_scale 在 `(0,1]` 内；仅限 Disabled 或 Holding |
| `pose_goal` | PoseGoal：command_id、pose、speed_scale | 条件同上；pose 使用基座 frame_id；IK 失败时拒绝 |
| `joint_trajectory` | JointTrajectory：command_id、retime、points[] | 仅限 Disabled 或 Holding；每个点包含 time_from_start、q/v/a 和导数标志；默认保持给定时间 |
| `joint_waypoints` | JointWaypoints：command_id、waypoints[].positions[6]、speed_scale | 仅限 Disabled 或 Holding；自动安排时间和内部过点速度，首尾静止 |
| `linear_path` | LinearPath：command_id、poses[]、speed_scale、blend_radius | 仅限 Disabled 或 Holding；使用基座 frame_id；单点等于 MoveL，多点可以停车或圆滑通过 |
| `joint_stream` | JointStream：session_id、sequence、positions[6] | 必须处于 JointStream；session 要与 `status.session_id` 一致，sequence 递增 |
| `pose_stream` | PoseStream：session_id、sequence、pose | 必须处于 PoseStream 或 Impedance；使用基座 frame_id；给出连续参考 |
| `joint_reference` | JointReference：session_id、sequence、target_time、positions/velocities/accelerations[6]、has_velocity/has_acceleration | 必须处于 JointStream；可以排入短时未来路点 |
| `pose_reference` | CartesianReference：session_id、sequence、target_time、pose、velocity(Twist)、acceleration(Accel)、has_velocity/has_acceleration | 必须处于 PoseStream 或 Impedance；位姿和导数都在基座系中，导数定义在 TCP 上 |
| `impedance_gains` | ImpedanceGains：command_id、stiffness[6]、damping[6] | 在 Holding 或 Impedance 下可用；顺序为 XYZ / 旋转 XYZ；六轴对角增益必须为有限正数；阻抗模式下按一阶平滑更新 |
| `gripper_goal` | GripperGoal：command_id、position | 需要处于 Holding 且有夹爪；范围 `[-1, 0] rad`，负值表示打开；只返回接受或拒绝 |
| `control` | ControlCommand：command_id、operation | 操作码见第 3 节 |

私有服务 `controller/set_friction_compensation`（`std_srvs/SetBool`）只用于切换摩擦补偿。它只在 Disabled，或反馈有效且机械臂静止的 Holding 状态下接受请求，运动中会拒绝；启用时还要求参数来源与当前附件/模型匹配。系数通过 `friction_file` 和 `friction_arm` 启动参数加载，服务本身不会热更新参数。完整流程见[摩擦补偿与对照](SDK_ALGORITHMS.md#9-摩擦补偿参数与实机对照)。

### 2.1 有限运动

有限目标可以从两种状态开始：反馈有效且静止的 Disabled，或者 Holding。在 Disabled 下，程序用实测位置（先投影到限位内）作为起点；在 Holding 下，则用当前的保持参考作为起点。运动过程中提交新的有限目标会被拒绝，需要先停止再发送。

同一臂的所有有限运动接口共用一个正数且递增的 `command_id`，新 ID 必须大于最后一次接受的有限目标 ID。控制操作、夹爪和增益消息的 ID 只用于关联结果，不占用这套递增序列。规划在调用线程中完成，控制周期继续运行；提交时程序会检查状态版本、反馈与模式，以及 Disabled 下的实测起点偏差。另一条有限目标被接受时也会更新版本，使旧计划失效。

- `joint_goal`：关节终点直接交给 Ruckig，生成一条静止到静止、速度和加速度及 jerk 均受限的轨迹。`speed_scale` 只缩放最大关节速度。
- `pose_goal`：先对终点求 IK，再交给 Ruckig。中间的 TCP 路径由关节运动决定；如果要沿直线运动，请使用 `linear_path`。
- `joint_trajectory`：完整的有限轨迹，`time_from_start` 从执行开始时算起，单位为秒。如果首点时间大于零，控制器会自动补一个 t=0 的当前起点；如果首点为零，它必须与当前静止参考一致。各点时间必须严格递增，提供加速度时也必须提供速度。首尾都静止，执行完成后进入 Holding。
- `joint_waypoints`：只提供关节位置，由控制器自动安排时间。`speed_scale=s` 会把基准速度、加速度和 jerk 分别按 s、s²、s³ 缩放，然后验证，必要时再做全局伸缩。内部同向路点通常以非零速度通过，反向轴可能会停一下。
- `linear_path`：单个位姿等于一次 MoveL。TCP 平移沿直线运动，姿态按 SO(3) 最短旋转插值，两者共用同一个路径进度。`blend_radius=0` 时在每个路点停车；正值时会在转角附近用 C2 曲线圆滑通过，不再精确经过中间点。路径长度按 `max(平移距离[m], 0.24×旋转角[rad])` 度量，裁剪长度最多为相邻段的 45%，因此纯旋转也能连续连接。

`linear_path` 在准备阶段完成连续 IK 和关节多项式拟合，执行时再按时间采样。默认的平移拟合阈值为 0.1 mm、旋转拟合阈值为 1 mrad；程序会为每段选择 9 个时间点，核对 FK 与目标路径，超差就加密采样，最多 5 轮。这是对参考路径采样误差的检查；实测到达使用第 4 节的容差。如果路径不可达、IK 分支断开，或者拟合仍然超差，命令会被拒绝。

STOP 和 DISABLE 会让正在准备的旧计划立即失效。已经接受的路径按单调时钟直接采样；在线制动仍然会检查关节边界。

### 2.2 流式参考

流式模式由 `control` 进入，控制器随即建立新会话并更新 `status.session_id`。会话初值在每个进程中随机生成。发送目标前先读取这个会话号，之后在同一次会话中递增 `sequence`。一次会话只能选择一种输入 Topic：`joint_stream` 或 `joint_reference`，`pose_stream` 或 `pose_reference`。

`target_time` 是路点的目标时刻，留零表示使用源时刻。非零值不能早于源时刻，默认也不能超过当前控制时刻之后 500 ms（`reference_horizon`）。按目标时间递增发送可以追加路点；如果插入较早的路点，会替换时间相同以及更晚的旧点。

流式有效期从源时刻和目标时刻中较晚的那个开始计算，再加上 `stream_timeout`（默认 200 ms）。控制器只有收到有效帧才会续期。会话号会在进入模式、停止和恢复时更新，恢复后旧会话的帧都会失效。

导数按下面的规则补齐：

- 当 `has_velocity=false` 时，用相邻点的位置差分估计速度，再按 `reference_velocity_filter`（默认 10 ms）做一阶滤波。
- 省略加速度时同样进行估计。提供加速度时必须同时提供速度。
- 关节量的单位是 rad/s 和 rad/s²；笛卡尔量是 m/s、rad/s、m/s²、rad/s²。角速度和角加速度都用基座系向量表达。
- 两端都有加速度时使用五次 Hermite 插值，否则使用三次；姿态在 SO(3) 上插值，旋转向量导数和角速度之间按左雅可比转换。

控制周期会采样前瞻 30 ms 处的参考，超过最后一点后最多外推 100 ms；超过外推期限后保持外推终点，并把导数置零。流式超时后进入停止流程。对应参数为 `reference_preview` 和 `reference_extrapolation`。

在 Holding 或 Impedance 下可以提交阻抗增益；在阻抗计算期间，实际值会按 `impedance_gain_time_constant`（默认 100 ms）平滑逼近配置值。

## 3. 控制操作与模式

| 码 | 名称 | 行为 |
| --- | --- | --- |
| 0 | ENABLE | 当处于 Disabled、反馈有效且机械臂静止时，进入 Holding |
| 1 | DISABLE | 取消运动并转为被动输出，停止主动姿态保持；任何状态都可以请求 |
| 2 | STOP | 从保持或运动状态生成受控停止；清掉旧目标，停稳后进入 AwaitResume |
| 3 | RESUME | 从 AwaitResume 回到原流式模式的新会话，或回到有限运动停止后的 Holding；从 Fault 回到 Disabled |
| 4 | JOINT_STREAM | 从 Holding 进入 JointStream，建立新会话，并等待第一条目标 |
| 5 | POSE_STREAM | 从 Holding 进入 PoseStream，建立新会话，并等待第一条目标 |
| 6 | IMPEDANCE | 从 Holding 进入 Impedance，建立新会话，参考从实测 TCP 开始 |
| 7 | HOLD | 从 AwaitResume 回到 Holding，结束旧模式；要求反馈有效且机械臂静止 |

如果要在运动中切换模式，请先发送 STOP，等待进入 `AwaitResume`，再发送 HOLD，最后选择新模式。处理完 Fault 的原因后，等反馈有效且机械臂静止时发送 RESUME，就能回到 Disabled。

模式数值：Disabled=0、Holding=1、JointGoal=2、PoseGoal=3、JointStream=4、PoseStream=5、Impedance=6、Stopping=7、AwaitResume=8、Fault=9、JointTrajectory=10、CartesianPath=11。

## 4. 状态与结果

| Topic | 内容 |
| --- | --- |
| `joint_states` | 最近收到的原始 q/dq/tau；首帧之后持续发布。有效性和新鲜度要看 `status.feedback_valid` / `feedback_age`，不能根据消息到达频率判断 |
| `commanded_joint_states` | 六轴主机参考 q/dq 和计算力矩（effort）；对应 `joint_states` 的前六轴，要结合 `status.mode` 读取 |
| `control_samples` | 可选的 `z1_msgs/ControlSample`；每收到一条新 MCU 反馈就发布一条，包含真实的单调接收/发送时刻、实际发送的上一轮输出、反馈、记录用重力和 V2 温度；需要设置 `record_control_samples=true` |
| `tcp_pose` | 反馈有效时发布，frame_id 为基座；夹爪位移不会改变固定 TCP |
| `status` | mode、reason、model_revision、session_id、active_command_id、feedback_sequence、feedback_valid、new_feedback、feedback_age、has_gripper、compute_time、loop_period/max_loop_period、overruns，以及主机周期数、漏过时隙、收发耗时、唤醒迟到和线程设置结果 |
| `result` | command_id + ACCEPTED / SUCCEEDED / REJECTED / CANCELED / STOPPED / FAILED + reason |

`status.collision_distance` 和 `collision_pair` 是保留的兼容字段，分别恒为 `NaN` 和空字符串，表示没有计算几何距离。这两个字段的类型和含义保持不变。

`status.friction_compensation_enabled` 表示当前的补偿开关。`ControlSample` 追加了一个同名字段和 `friction_feedforward[6]`，前者表示实际发送输出中的补偿状态，后者表示总限幅前请求的关节摩擦前馈；采样中的前馈并不等同于限幅后单独施加的电机力矩。新增消息字段后，发布端和订阅端都需要重新构建。

`ACCEPTED` 表示有限目标已经完成准备并被控制器接受。轨迹执行结束后，实测关节误差需要小于 `position_tolerance=0.015 rad`，并满足静止判定；位姿运动还要求 TCP 平移误差小于 `0.005 m`、旋转误差小于 `0.025 rad`。这些条件要按新反馈的接收时间持续满足超过 100 ms，才会返回 `SUCCEEDED`。末端等待超过 `settle_timeout=3 s` 时进入 Fault，并返回 `FAILED`；准备阶段的 IK 或轨迹验证失败则返回 `REJECTED`。

流式帧的拒绝原因会打印在 ROS 日志中，模式和停止原因则要看 `status`。发送有限目标前先订阅 `result`，再按 `command_id` 关联接受事件和终态。原生事件缓存和 ROS 发布队列的容量都是 128，事件不会被持久保存。

到达超时的 `reason` 会记录触发瞬间的最大关节误差、TCP 位姿误差（适用时）、静止判定、实测与参考关节角，以及上一周期的命令力矩。超时进入 Fault 后，输出转为被动，实测位置可能回落；如果想知道运动期间主机给过多大力矩，请查看当时的 `commanded_joint_states`。Fault 之后参考位置会保留，计算力矩清零。

`STOPPED` 表示活动的有限命令已经结束、制动已经开始；机械臂停稳后，模式才会变为 `AWAIT_RESUME`。

## 5. 发布示例

### 5.1 保持、停止与回到 Holding

启动控制器后，在另一个已经加载工作区环境的终端中查看状态：

```bash
rostopic echo /left_arm/status
```

当反馈有效且机械臂静止时，发送 ENABLE 进入 Holding：

```bash
rostopic pub -1 /left_arm/control z1_msgs/ControlCommand '{command_id: 1, operation: 0}'
```

需要停止时发送 STOP：

```bash
rostopic pub -1 /left_arm/control z1_msgs/ControlCommand '{command_id: 2, operation: 2}'
```

等 `status.mode` 变成 `AWAIT_RESUME=8` 后，再发送 HOLD，回到 Holding：

```bash
rostopic pub -1 /left_arm/control z1_msgs/ControlCommand '{command_id: 3, operation: 7}'
```

### 5.2 指定 1 s 的关节轨迹

下面的函数供已经初始化的 `rospy` 节点调用。`pub` 是连接到 `/left_arm/joint_trajectory` 的发布者，`q_target` 按 `joint1..joint6` 排列，`command_id` 要大于该臂最后接受的有限目标 ID。

```python
import rospy
from z1_msgs.msg import JointTrajectory, TrajectoryPoint


def send_timed_goal(pub, command_id, q_target):
    m = JointTrajectory()
    m.command_id = command_id
    m.header.stamp = rospy.Time.now()
    m.retime = False
    end = TrajectoryPoint()
    end.time_from_start = rospy.Duration(1.0)
    end.positions = q_target
    end.has_velocity = end.has_acceleration = True  # 消息默认导数为零
    m.points = [end]
    pub.publish(m)
```

从静止的 Disabled 或 Holding 状态发送时，控制器会自动补一个 t=0 的当前起点。如果 1 s 内无法满足运动限制，命令会被拒绝；如果允许自动延长时间，请把 `retime` 改为 `True`。发布后可以通过 `result` 读取接受和完成结果。

### 5.3 持续发布未来 40 ms 的关节参考

流式发布程序先发送 `JOINT_STREAM`（operation=4），收到 JointStream 状态和新的 `session_id` 后立即开始发送；等待首帧同样受 200 ms 超时约束。通常以 20–100 Hz 调用下面的函数，并在每次调用时递增 `sequence`：

```python
import rospy
from z1_msgs.msg import JointReference


def send_reference(pub, session, sequence, q, dq, ddq):
    m = JointReference()
    m.header.stamp = rospy.Time.now()
    m.target_time = m.header.stamp + rospy.Duration(0.04)
    m.session_id = session
    m.sequence = sequence
    m.positions, m.velocities, m.accelerations = q, dq, ddq
    m.has_velocity = m.has_acceleration = True
    pub.publish(m)
```

`pub` 对应 `/left_arm/joint_reference`，可以使用 `queue_size=32`；进入流式模式前，控制器应处于 Holding。停止或恢复之后，要重新读取会话号再发送。ROS 消息发布频率与 250 Hz 控制周期相互独立，新测量的频率要看 `status.feedback_sequence` 的增长。

### 5.4 原生 C++ 调用

原生接口使用 SDK 的单调时钟秒数。已经进入 JointStream 并取得会话号后，可以提交同样的未来参考：

```cpp
const double now = z1::monotonicTime();
z1::JointReference point;
point.time = now + 0.04;
point.position = desired_q;
point.velocity = desired_dq;
point.acceleration = desired_ddq;
point.has_velocity = point.has_acceleration = true;
bool accepted = runtime.controller().stream(session, ++sequence, point, now);
```

有限定时运动使用相对执行起点的秒数：

```cpp
z1::TrajectoryPoint end;
end.time_from_start = 1.0;
end.position = desired_q;
end.has_velocity = end.has_acceleration = true;  // 默认导数为零
bool accepted = controller.trajectory(20, {end}, false);
```

其他入口为 `goal`、`poseGoal`、`jointWaypoints`、`moveL` 和 `linearWaypoints`。每次只能选择一个有限运动，等回到 Holding 后再提交下一条；它们的时间和路径语义区别见[算法对照表](SDK_ALGORITHMS.md#83-各接口的算法区别)。

## 6. 模型配置

两个控制节点在各自的私有参数空间中加载同一份 [control.yaml](z1_bringup/config/control.yaml)。地址、本地端口和接收预算由通信 YAML 加载到对应节点，launch 则设置各臂的 `model_file` 和 `joint_prefix`。如果要为某一臂使用不同的控制参数、模型或基座重力，只需修改对应节点的参数加载，控制代码是同一套。

ROS 默认 `period=0.004`、`receive_budget_us=3000`。节点启动时会检查接收预算小于控制周期；当前配置给控制计算和调度留了大约 1 ms。轨迹准备在锁外计算，只有在读取起点和提交计划时才短暂持锁。

`control_cpu=-1` 表示不指定 CPU，其他值是部署机上的逻辑 CPU 编号，需要按实际拓扑选择。`realtime_priority=0` 表示使用默认调度，正值会请求控制线程使用 SCHED_FIFO，权限由部署环境提供。原生构造可以传入 `RuntimeOptions{priority, cpu}`；`Runtime::timing()` 返回统计信息，时间单位为微秒。

`status` 中与运行时有关的字段如下：

| 字段 | 含义 |
| --- | --- |
| `runtime_cycles` / `missed_control_slots` | 运行周期数 / 跳过的过期时隙数 |
| `exchange_time` / `max_exchange_time` | 最近一次 / 最大一次 `Transport::exchange` 的耗时，单位为秒 |
| `wakeup_lateness` / `max_wakeup_lateness` | 最近一次 / 最大一次唤醒迟到，单位为秒 |
| `realtime_scheduling` / `cpu_affinity_applied` | 请求的实时优先级 / CPU 亲和性是否设置成功 |
| `realtime_error` / `cpu_affinity_error` | 设置线程时返回的错误码，0 表示没有错误；没有请求设置时也是 0 |
| `trajectory_duration` / `trajectory_time_scale` | 最近接受的有限轨迹时长（秒）/ 全局时间伸缩倍率；进入流式模式后重置为 0 / 1 |

`runtime_cycles`、漏时隙、交换耗时和唤醒迟到统计会在 `Runtime::start()` 时清零；控制器的 `max_loop_period` 和 `overruns` 则随 Controller 对象累计。`overruns` 统计的是控制计算超期，与漏过调度时隙分开计算；单次计算超期只会增加计数，输出继续。控制间隔如果超过反馈有效期，就会进入 Fault。流式参考按实际经过时间推进，错过的时隙不会集中补算。

控制模型会读取六轴关节、限位、质量、惯性、TCP 和基座重力。URDF 可以省略所有 `collision` 标签；已有的碰撞描述留给规划和仿真使用，控制进程不需要把碰撞网格解析成几何对象。质量、质心和惯性在控制 URDF 中修改，显示模型中的对应参数也要同步更新。

原生 `ModelConfig.has_gripper` 默认为 `true`，保留已有的调用语义。ROS 控制节点会根据 MCU 握手得到的 8/7 电机配置自动设置它，然后构建控制模型和规划模型。无夹爪时，程序会从源 URDF 中移除夹爪子树，保留六个主关节和无质量的 `tool`。`controller/device_has_gripper` 和 `effective_model_xml` 可以读取当前选择；模型版本会区分两种有效 XML。夹爪开合仍然沿用现有闭合姿态下的六轴约化模型，没有从原始夹爪编码器推导手指关节角。

`control_samples` 的命令字段是主机实际传给本次交换的输出，不是固件执行确认；反馈字段是本次新接收到的包。队列溢出会通过 `dropped_samples` 累计报告，不会覆盖旧样本。记录通道不改变原状态话题的频率和字段含义。实验入口和输出格式见[摩擦辨识](FRICTION_IDENTIFICATION.md)。

`status.model_revision` 是控制 URDF 全文、TCP 名称和重力向量的 SHA-256。修改 URDF 中的显示或碰撞标签也会改变指纹；外部网格文件和控制器增益不计入指纹。

### 从原几何接口迁移

- 从控制 YAML 中删除 `ignored_collision_pairs` 和 `fixed_boxes`，节点已经停止读取这两个参数；需要的排除规则和环境对象请交给上层规划器配置。
- 原生 API 已经移除 `FixedBox`、`ModelConfig::ignored_pairs` / `fixed_boxes`、`Clearance` / `ClearanceSample` 以及 `Model::clearance` / `clearanceSample`。调用方请改用上层几何库；`Model` 的 FK、IK、雅可比和动力学接口保持不变。
- `Snapshot` 和 ROS `Status` 保留旧距离字段，按第 4 节的约定表示不可用。依赖距离值的程序请改为读取上层几何计算结果。
- 模型指纹已经移除旧几何配置部分，升级后请重新记录 `model_revision`。
