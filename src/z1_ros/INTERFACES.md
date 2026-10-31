# Z1 Topic 接口说明

左、右臂分别使用 `/left_arm`、`/right_arm` 命名空间。下文使用相对话题名，例如左臂的 `joint_goal` 是 `/left_arm/joint_goal`。消息定义见 [z1_msgs/msg](z1_msgs/msg)，启动方式见 [README.md](README.md)。

## 1. 通用约定

### 1.1 单位与坐标系

- 关节顺序固定为 `joint1..joint6`，角度 rad，角速度 rad/s，力矩 N·m，位置 m。
- TCP 位姿的四元数按 `(x, y, z, w)` 填写，取单位长度；适配器允许模长偏差不超过 `0.001`，接收后归一化。`frame_id` 用本臂基座 `<prefix>link00`，例如 `left_link00`；其他坐标系下的目标先转换再发送，控制器不查 TF。
- 默认 TCP 名为 `tool`，从 `link06` 沿 X 正方向平移 `(0.051, 0, 0)` m，姿态不变，夹爪开合不影响它。
- `joint_states` 的关节名带前缀。检测到夹爪时追加 `<prefix>gripper_motor`，这是夹爪电机的原始反馈；描述模型里的夹爪连杆角是另一套量。

### 1.2 时间戳、时钟与队列

- 有限目标、`control`、`impedance_gains` 和 `gripper_goal` 的 `header.stamp` 可以留零；流式消息需要非零时间戳。非零时间戳默认允许位于当前 ROS 时刻前 200 ms、后 20 ms 内，过去时龄上限由 `command_max_age` 配置。STOP、DISABLE 跳过时间戳检查。
- 流式输入的 `header.stamp` 表示上游生成时刻。适配器将源时刻和路点时刻映射到单调时钟，延迟已经消耗的有效期不会重新补满；有效帧才续期。
- 内核按单调时钟推进轨迹和期限。ROS 时间影响新消息的时龄检查，已经映射到内核的期限继续按单调时钟执行。
- 输出消息的 `header.stamp` 是发布时刻。实际接收序号和时龄见 `status`；固件未提供采样时刻。
- 订阅队列：普通目标和流式消息为 1，`joint_reference` / `pose_reference` 为 32，`control` 为 16。原生时间线最多保留 32 个点，包含插值所需的前驱点。多个流式发布者共用会话时，由上层协调序号和目标时间。

### 1.3 运动边界与几何规划

控制器执行关节位置、速度、加速度、jerk、主机力矩及变化率限制，并检查反馈、电机状态和关节制动路径。几何碰撞检测和绕障属于上层规划；控制内核不创建碰撞对象，也不加载碰撞网格。

`joint_limit_tolerance` 默认 `0.001 rad`，给编码器在模型限位附近的偏差留余量。反馈超出该容差时判为无效；通过检查后，FK 和动力学使用原始 q/dq，状态发布也保留原始测量。用实测位置建立规划或保持参考时，将容差内的越界量投影回模型限位；目标、轨迹多项式和制动路径按正式限位检查。独立调用 `Model::ik` 时，初值允许同样的 1 mrad 偏差，迭代前先投影到限位内，最终解满足模型限位。

静止判定分两档：`start_velocity_tolerance` 用于 Disabled 起步、ENABLE、HOLD 和恢复，`velocity_tolerance` 用于到达和停止完成，默认均为 `0.03 rad/s`。

已有两帧新反馈时，原始速度和相邻位置差分速度均低于阈值，即可通过。原始速度有尖峰时，改用至少 `stationary_window`（默认 100 ms）的新位置历史：窗口位置范围除以实际时长，以及最近的相邻位置差分速度，都需低于阈值。位置范围取最大值减最小值，可识别窗口内的往返运动。只有一帧位置时，按原始速度判定。反馈无效会清空历史，重复使用旧反馈不会增加窗口时长。

夹爪原始反馈允许 `[-1.02, 0.02] rad` 的测量范围，命令和用户目标限制在 `[-1, 0] rad`。保持或生成夹爪参考时，先把测量投影到命令范围，再做限速。传输异常会把底层原因写进 `status.reason` 和失败的 `result.reason`，可以分清命令校验、发送失败和接收积压。

## 2. 命令接口

| Topic | 消息 / 字段 | 接收条件与约束 |
| --- | --- | --- |
| `joint_goal` | JointGoal：command_id、positions[6]、speed_scale | ID 为正且递增；speed_scale `(0,1]`；Disabled 或 Holding |
| `pose_goal` | PoseGoal：command_id、pose、speed_scale | 同上；pose 用基座 frame_id；IK 失败则拒绝 |
| `joint_trajectory` | JointTrajectory：command_id、retime、points[] | Disabled 或 Holding；点含 time_from_start、q/v/a 和导数标志；默认保持给定时间 |
| `joint_waypoints` | JointWaypoints：command_id、waypoints[].positions[6]、speed_scale | Disabled 或 Holding；自动安排时间和内部过点速度，首尾静止 |
| `linear_path` | LinearPath：command_id、poses[]、speed_scale、blend_radius | Disabled 或 Holding；基座 frame_id；单点为 MoveL，多点可停车或圆滑通过 |
| `joint_stream` | JointStream：session_id、sequence、positions[6] | 处于 JointStream；session 与 `status.session_id` 一致，sequence 递增 |
| `pose_stream` | PoseStream：session_id、sequence、pose | 处于 PoseStream 或 Impedance；基座 frame_id；连续参考 |
| `joint_reference` | JointReference：session_id、sequence、target_time、positions/velocities/accelerations[6]、has_velocity/has_acceleration | 处于 JointStream；可排入短时未来路点 |
| `pose_reference` | CartesianReference：session_id、sequence、target_time、pose、velocity(Twist)、acceleration(Accel)、has_velocity/has_acceleration | 处于 PoseStream 或 Impedance；位姿和导数都在基座系，导数定义在 TCP |
| `impedance_gains` | ImpedanceGains：command_id、stiffness[6]、damping[6] | Holding 或 Impedance；顺序 XYZ / 旋转 XYZ；有限对角正值；阻抗中一阶平滑更新 |
| `gripper_goal` | GripperGoal：command_id、position | Holding 且有夹爪；`[-1, 0] rad`，负值打开；只返回接受或拒绝 |
| `control` | ControlCommand：command_id、operation | 操作码见第 3 节 |

### 2.1 有限运动

有限目标可以从两种状态开始：反馈有效且静止的 Disabled，或者 Holding。Disabled 下用实测位置（投影到限位内）作为起点；Holding 下用当前保持参考。运动中提交新的有限目标会被拒绝，先停再发。

同一臂的所有有限运动接口共用一个正数、递增的 `command_id`，新 ID 需大于最后一次接受的有限目标 ID。控制操作、夹爪和增益消息的 ID 用于关联结果，不占用这套递增序列。规划在调用线程完成，控制周期继续运行；提交时检查状态版本、反馈与模式，以及 Disabled 下的实测起点偏差。另一条有限目标被接受时也会更新版本，使旧计划失效。

- `joint_goal`：关节终点直接交给 Ruckig，生成静止到静止、速度和加速度及 jerk 受限的轨迹。`speed_scale` 只缩放最大关节速度。
- `pose_goal`：先对终点求 IK，再走 Ruckig。中间 TCP 路径由关节运动决定；要沿直线走，用 `linear_path`。
- `joint_trajectory`：完整有限轨迹，`time_from_start` 从执行开始算，单位秒。首点时间大于零时，控制器自动补一个 t=0 的当前起点；首点为零时，它要和当前静止参考一致。各点时间严格递增，提供加速度时也要提供速度。首尾都静止，执行完进入 Holding。
- `joint_waypoints`：只给关节位置，控制器自动排时间。`speed_scale=s` 把基准速度、加速度、jerk 分别按 s、s²、s³ 缩放，然后验证并在需要时全局伸缩。内部同向路点通常以非零速度通过，反向轴可能停一下。
- `linear_path`：单个位姿等于一次 MoveL。TCP 平移走直线，姿态走 SO(3) 最短旋转，两者共用路径进度。`blend_radius=0` 时在每个路点停车；正值时在转角附近用 C2 曲线圆滑通过，不再精确经过中间点。路径长度按 `max(平移距离[m], 0.24×旋转角[rad])` 度量，裁剪长度最多为相邻段的 45%，纯旋转也能连续连接。

`linear_path` 在准备阶段完成连续 IK 和关节多项式拟合，执行时按时间采样。默认平移拟合阈值 0.1 mm、旋转 1 mrad；每段选 9 个时间点核对 FK 与目标路径，超差则加密，最多 5 轮。这是参考路径的采样误差检查；实测到达使用第 4 节的容差。路径不可达、IK 分支断开或拟合仍超差时，命令被拒绝。

STOP 和 DISABLE 会让正在准备的旧计划立即失效。已接受的路径按单调时钟直接采样；在线制动仍检查关节边界。

### 2.2 流式参考

流式模式由 `control` 进入，控制器随即建立新会话并更新 `status.session_id`。会话初值在每个进程中随机生成。发目标前先读这个会话号，之后在同一次会话里递增 `sequence`。一次会话只选一种输入 Topic：`joint_stream` 或 `joint_reference`，`pose_stream` 或 `pose_reference`。

`target_time` 是路点的目标时刻，留零表示使用源时刻。非零值不早于源时刻，默认不超过当前控制时刻之后 500 ms（`reference_horizon`）。按目标时间递增发送可追加路点；插入较早路点会替换时间相同及更晚的旧点。

流式有效期从源时刻和目标时刻中较晚的那个算起，再加 `stream_timeout`（默认 200 ms）。控制器收到有效帧才续期。会话号在模式进入、停止和恢复时更新，恢复后旧会话的帧失效。

导数按下面的规则补齐：

- `has_velocity=false` 时，用相邻点的位置差分估计速度，再按 `reference_velocity_filter`（默认 10 ms）做一阶滤波。
- 省略加速度时同样估计。提供加速度必须同时提供速度。
- 关节量的单位是 rad/s、rad/s²；笛卡尔量是 m/s、rad/s、m/s²、rad/s²。角速度和角加速度都用基座系向量表达。
- 两端都有加速度时用五次 Hermite 插值，否则用三次；姿态在 SO(3) 上插值，旋转向量导数和角速度之间按左雅可比转换。

控制周期在前瞻 30 ms 处采样参考，超过最后一点最多外推 100 ms；超过外推期限后保持外推终点，导数置零。流式超时进入停止流程。对应参数为 `reference_preview` 和 `reference_extrapolation`。

Holding 或 Impedance 下可提交阻抗增益；在阻抗计算期间，实际值按 `impedance_gain_time_constant`（默认 100 ms）平滑逼近配置值。

## 3. 控制操作与模式

| 码 | 名称 | 行为 |
| --- | --- | --- |
| 0 | ENABLE | Disabled、反馈有效且静止 → Holding |
| 1 | DISABLE | 取消运动并转被动输出，停止主动姿态保持；任何状态都可请求 |
| 2 | STOP | 从保持或运动生成受控停止；清掉旧目标，停稳后进入 AwaitResume |
| 3 | RESUME | AwaitResume → 原流式模式的新会话，或有限运动停止后的 Holding；Fault → Disabled |
| 4 | JOINT_STREAM | Holding → JointStream，新会话，等待第一条目标 |
| 5 | POSE_STREAM | Holding → PoseStream，新会话，等待第一条目标 |
| 6 | IMPEDANCE | Holding → Impedance，新会话，参考从实测 TCP 开始 |
| 7 | HOLD | AwaitResume → Holding，结束旧模式；要求反馈有效且静止 |

运动中换模式：先 STOP，等待 `AwaitResume`，再 HOLD，最后选择新模式。Fault 的原因处理完后，反馈有效且机械臂静止时发送 RESUME，回到 Disabled。

模式数值：Disabled=0、Holding=1、JointGoal=2、PoseGoal=3、JointStream=4、PoseStream=5、Impedance=6、Stopping=7、AwaitResume=8、Fault=9、JointTrajectory=10、CartesianPath=11。

## 4. 状态与结果

| Topic | 内容 |
| --- | --- |
| `joint_states` | 最近收到的原始 q/dq/tau；首帧之后持续发布。有效性与新鲜度看 `status.feedback_valid` / `feedback_age`，不能由消息到达频率判断 |
| `commanded_joint_states` | 六轴主机参考 q/dq 和计算力矩（effort）；对应 `joint_states` 的前六轴，结合 `status.mode` 读取 |
| `tcp_pose` | 反馈有效时发布，frame_id 为基座；夹爪位移不改变固定 TCP |
| `status` | mode、reason、model_revision、session_id、active_command_id、feedback_sequence、feedback_valid、new_feedback、feedback_age、has_gripper、compute_time、loop_period/max_loop_period、overruns，以及主机周期数、漏过时隙、收发耗时、唤醒迟到和线程设置结果 |
| `result` | command_id + ACCEPTED / SUCCEEDED / REJECTED / CANCELED / STOPPED / FAILED + reason |

`status.collision_distance` 和 `collision_pair` 是保留的兼容字段，分别恒为 `NaN` 和空字符串，表示未计算几何距离。消息字段顺序与类型保持不变。

`ACCEPTED` 表示有限目标已完成准备并被控制器接受。轨迹执行结束后，实测关节误差需小于 `position_tolerance=0.015 rad`，并满足静止判定；位姿运动还需 TCP 平移误差小于 `0.005 m`、旋转误差小于 `0.025 rad`。这些条件按新反馈接收时间持续超过 100 ms，才返回 `SUCCEEDED`。末端等待超过 `settle_timeout=3 s` 时进入 Fault，返回 `FAILED`；准备阶段的 IK 或轨迹验证失败返回 `REJECTED`。

流式帧的拒绝原因在 ROS 日志中提示，模式和停止原因看 `status`。发有限目标前订阅 `result`，按 `command_id` 关联接受与终态。原生事件缓存和 ROS 发布队列容量均为 128，事件不持久保存。

到达超时的 `reason` 会记录触发瞬间的最大关节误差、TCP 位姿误差（适用时）、静止判定、实测与参考关节角、上一周期命令力矩。超时进入 Fault 后输出转被动，实测位置可能回落；想知道运动期间主机给过多少力矩，查当时的 `commanded_joint_states`，Fault 后参考位置保留、计算力矩清零。

`STOPPED` 是活动有限命令结束、制动开始的事件；机械臂停稳后，模式才变为 `AWAIT_RESUME`。

## 5. 发布示例

### 5.1 保持、停止与回到 Holding

启动控制器后，在另一个已加载工作区环境的终端查看状态：

```bash
rostopic echo /left_arm/status
```

反馈有效且机械臂静止时，发送 ENABLE 进入 Holding：

```bash
rostopic pub -1 /left_arm/control z1_msgs/ControlCommand '{command_id: 1, operation: 0}'
```

需要停止时发送 STOP：

```bash
rostopic pub -1 /left_arm/control z1_msgs/ControlCommand '{command_id: 2, operation: 2}'
```

等 `status.mode` 变为 `AWAIT_RESUME=8`，再发送 HOLD，回到 Holding：

```bash
rostopic pub -1 /left_arm/control z1_msgs/ControlCommand '{command_id: 3, operation: 7}'
```

### 5.2 指定 1 s 的关节轨迹

下面的函数供已初始化的 `rospy` 节点调用。`pub` 是连接到 `/left_arm/joint_trajectory` 的发布者，`q_target` 按 `joint1..joint6` 排列，`command_id` 大于该臂最后接受的有限目标 ID。

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

从静止的 Disabled 或 Holding 发送时，控制器自动补 t=0 的当前起点。1 s 内无法满足运动限制就拒绝；允许自动延长时间时，将 `retime` 改为 `True`。发布后通过 `result` 读取接受与完成结果。

### 5.3 持续发布未来 40 ms 的关节参考

流式发布程序先发送 `JOINT_STREAM`（operation=4），收到 JointStream 状态及新 `session_id` 后立即开始发送，首帧等待也受 200 ms 超时约束。通常以 20–100 Hz 调用下面的函数，每次递增 `sequence`：

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

`pub` 对应 `/left_arm/joint_reference`，可使用 `queue_size=32`；进入流式前控制器应处于 Holding。停止或恢复后，重新读取会话号再发送。ROS 消息发布频率与 250 Hz 控制周期相互独立，新测量频率看 `status.feedback_sequence` 的增长。

### 5.4 原生 C++ 调用

原生接口使用 SDK 的单调时钟秒数。已进入 JointStream 并取得会话号时，可以提交同样的未来参考：

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

其他入口为 `goal`、`poseGoal`、`jointWaypoints`、`moveL` 和 `linearWaypoints`。每次选择一个有限运动，等待回到 Holding 后再提交下一条；它们的时间与路径区别见[算法对照表](SDK_ALGORITHMS.md#83-各接口的算法区别)。

## 6. 模型配置

两个控制节点在各自私有参数空间加载同一份 [control.yaml](z1_bringup/config/control.yaml)。地址、本地端口和接收预算由通信 YAML 加载到对应节点，launch 设置各臂的 `model_file` 和 `joint_prefix`。要逐臂用不同的控制参数、模型或基座重力，改对应节点的参数加载即可，控制代码是同一套。

ROS 默认 `period=0.004`、`receive_budget_us=3000`。节点启动时检查接收预算小于控制周期；当前配置给控制计算和调度留约 1 ms。轨迹准备在锁外计算，起点读取和计划提交时短暂持锁。

`control_cpu=-1` 表示不指定 CPU，其他值是部署机上的逻辑 CPU 编号，按部署机的实际拓扑选择。`realtime_priority=0` 用默认调度，正值请求控制线程使用 SCHED_FIFO，权限由部署环境提供。原生构造可传 `RuntimeOptions{priority, cpu}`；`Runtime::timing()` 返回统计，时间单位为微秒。

`status` 中和运行时有关的字段：

| 字段 | 含义 |
| --- | --- |
| `runtime_cycles` / `missed_control_slots` | 运行周期数 / 跳过的过期时隙数 |
| `exchange_time` / `max_exchange_time` | 最近 / 最大一次 `Transport::exchange` 耗时，秒 |
| `wakeup_lateness` / `max_wakeup_lateness` | 最近 / 最大唤醒迟到，秒 |
| `realtime_scheduling` / `cpu_affinity_applied` | 请求的实时优先级 / CPU 亲和性是否设置成功 |
| `realtime_error` / `cpu_affinity_error` | 设置线程时的错误码，0 表示无错误；不请求设置时也是 0 |
| `trajectory_duration` / `trajectory_time_scale` | 最近接受的有限轨迹时长（秒）/ 全局时间伸缩倍率；进入流式模式后重置为 0 / 1 |

`runtime_cycles`、漏时隙、交换耗时和唤醒迟到统计在 `Runtime::start()` 时清零；控制器的 `max_loop_period`、`overruns` 随 Controller 对象累计。`overruns` 统计控制计算超期，与漏过调度时隙分开计算；单次计算超期增加计数，输出继续。控制间隔超过反馈有效期则进入 Fault。流式参考按实际经过时间推进，错过的时隙不集中补算。

控制模型读取六轴关节、限位、质量、惯性、TCP 和基座重力。URDF 可以省略所有 `collision` 标签；已有碰撞描述留给规划和仿真，碰撞网格无需在控制进程中解析为几何对象。质量、质心、惯性在控制 URDF 中修改，显示模型的对应参数同步更新。

`status.model_revision` 是控制 URDF 全文、TCP 名称和重力向量的 SHA-256。修改 URDF 中的显示或碰撞标签也会改变指纹；外部网格文件和控制器增益不在指纹中。

### 从原几何接口迁移

- 从控制 YAML 删除 `ignored_collision_pairs` 和 `fixed_boxes`，节点已停止读取这两个参数；需要的排除规则和环境对象交给上层规划器配置。
- 原生 API 已移除 `FixedBox`、`ModelConfig::ignored_pairs` / `fixed_boxes`、`Clearance` / `ClearanceSample` 和 `Model::clearance` / `clearanceSample`。调用方改用上层几何库；`Model` 的 FK、IK、雅可比和动力学接口保持不变。
- `Snapshot` 与 ROS `Status` 保留旧距离字段，按第 4 节表示不可用。依赖距离值的程序改为读取上层几何计算结果。
- 模型指纹已移除旧几何配置部分，升级后重新记录 `model_revision`。
