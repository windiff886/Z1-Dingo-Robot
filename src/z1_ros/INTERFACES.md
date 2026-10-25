# 单臂 Topic 与控制契约

## 1. 单位、时间和命名

- 主臂顺序固定 `joint1..joint6`；角度 rad，速度 rad/s，主臂力矩 Nm，位置 m。
- TCP 位姿 quaternion `(x,y,z,w)` 需单位长度。frame_id 必须是该臂的 `<prefix>link00`，例如 `left_link00`；不隐式查询 TF 转换。
- 默认 TCP 为 `tool`，相对 `link06` 固定平移 `(0.051,0,0)` m、无旋转，不随夹爪开合改变。
- `joint_states` 的 name 带前缀；夹爪存在时追加 `<prefix>gripper_motor`，这是独立电机反馈，不声称对应描述模型的某个夹爪连杆角度。
- 命令 header.stamp：有限目标/控制可为零（按接收处理），流式必须非零；默认允许过去 200 ms、未来 20 ms。有效流式帧的 ROS 时龄会扣减单调时钟有效期，延迟帧不会获得完整的新有效期；无效帧不续期。
- 内核超时使用单调时钟；ROS 时间跳变不重置控制期限。ROS header 是发布时刻，实际有效接收序号和接收时龄位于 status。没有固件采样时刻。
- 普通目标 topic 队列为 1；`joint_reference` / `pose_reference` 队列为 32，`control` 为 16。原生时间线最多保留 32 点，新时间点会替换该时间及之后的旧路点。多发布者需在上层协调。

## 2. 命令

所有类型位于 [z1_msgs/msg](z1_msgs/msg)。

| Topic | 消息 / 字段 | 约束 |
| --- | --- | --- |
| `joint_goal` | JointGoal：command_id、positions[6]、speed_scale | ID 正数、递增；speed_scale `(0,1]`；Holding 接收 |
| `pose_goal` | PoseGoal：command_id、pose、speed_scale | 同上；指定基座 frame_id；局部 IK 失败则拒绝 |
| `joint_trajectory` | JointTrajectory：command_id、retime、points[] | Holding 接收完整有限轨迹；点含 time_from_start、q/v/a 和导数有效标志；默认保持时间 |
| `joint_waypoints` | JointWaypoints：command_id、waypoints[].positions[6]、speed_scale | Holding；自动安排时间和内部过点速度，首尾停止 |
| `linear_path` | LinearPath：command_id、poses[]、speed_scale、blend_radius | Holding；基座 frame_id；单点为 MoveL，多点可停车或圆滑连续通过 |
| `joint_stream` | JointStream：session_id、sequence、positions[6] | JointStream 模式；token 等于 status，会话内 sequence 递增 |
| `pose_stream` | PoseStream：session_id、sequence、pose | PoseStream 或 Impedance 模式；基座 frame_id；连续参考，不承诺到达每帧 |
| `joint_reference` | JointReference：session_id、sequence、target_time、positions/velocities/accelerations[6]、has_velocity/has_acceleration | JointStream 模式；时间插值的动态参考；可排入短时未来路点 |
| `pose_reference` | CartesianReference：session_id、sequence、target_time、pose、velocity(Twist)、acceleration(Accel)、has_velocity/has_acceleration | PoseStream 或 Impedance 模式；位姿及所有导数均在基座系，导数定义于 TCP |
| `impedance_gains` | ImpedanceGains：command_id、stiffness[6]、damping[6] | Holding 或 Impedance；顺序 XYZ/旋转XYZ，对角正值；平移 K≤500 N/m，旋转 K≤30 Nm/rad，D≤100；阻抗中一阶平滑更新 |
| `gripper_goal` | GripperGoal：command_id、position | Holding、有夹爪，[-1,0] rad；负值打开；只返回接收/拒绝 |
| `control` | ControlCommand：command_id、operation | 操作码见下表 |

控制操作码：

| 码 | 名称 | 行为 |
| --- | --- | --- |
| 0 | ENABLE | Disabled、有效且静止、距离大于警告间距 → Holding |
| 1 | DISABLE | 直接被动输出；可从任何状态请求，不代表硬件急停确认 |
| 2 | STOP | 从保持/运动生成受控停止；清除旧目标，停止后 AwaitResume |
| 3 | RESUME | AwaitResume → 原流式模式及新会话，或有限运动停止后 → Holding；Fault → Disabled，需再次使能 |
| 4 | JOINT_STREAM | Holding → JointStream，新会话，等待新目标 |
| 5 | POSE_STREAM | Holding → PoseStream，新会话，等待新目标 |
| 6 | IMPEDANCE | Holding → Impedance，新会话；参考从实际 TCP 开始 |
| 7 | HOLD | AwaitResume → Holding，显式结束旧模式；有效且静止、几何间距足够才允许 |

模式不能在运动中直接抢占；先 STOP，等 AwaitResume，HOLD，再选择其他模式。Fault 不自动清除；无效反馈和碰撞条件未解除时 RESUME 无效。STOP/DISABLE 不因 ROS 命令时间戳错误而被拒绝。

## 3. 状态与结果

| Topic | 内容 |
| --- | --- |
| `joint_states` | 原始实测 q/dq/tau；旧测量仍可能保留发布，使用者必须结合 status.feedback_valid/age 判定有效性 |
| `tcp_pose` | 只在反馈有效时发布，frame_id 为基座；夹爪位移不改变固定 TCP 定义 |
| `status` | mode、reason、model_revision、session_id、active_command_id、feedback_sequence、feedback_valid、new_feedback、feedback_age、has_gripper、collision_distance/pair、compute_time、loop_period/max_loop_period、overruns；另含主机周期数、漏过时隙、收发耗时、唤醒迟到及线程设置结果 |
| `result` | command_id + ACCEPTED/SUCCEEDED/REJECTED/CANCELED/STOPPED/FAILED + reason |

模式数值为 Disabled=0、Holding=1、JointGoal=2、PoseGoal=3、JointStream=4、PoseStream=5、Impedance=6、Stopping=7、AwaitResume=8、Fault=9、JointTrajectory=10、CartesianPath=11。旧数值保持不变。

有限目标接受后才会有最终结果；最终成功需实际 q/dq 满足容差并稳定 100 ms，末端目标还需实际 TCP 满足位姿容差。到达超期、IK/轨迹失败不报成功。持续流没有“每帧到达”承诺，拒绝的流式帧在 ROS 日志报告；模式和停止原因通过 status 判断。result 为有界事件流（128），发布者应及时订阅，不能当永久审计日志。

默认超时：反馈 80 ms、流式 200 ms。普通流式按源时间计；未来路点允许执行到最后有效路点时间再加 200 ms。收到无效/过期/旧会话/倒序目标不会刷新有效期。会话初值每个进程随机生成，模式进入/停止/恢复均更新。新会话不能执行停止前或停止期间缓存，恢复后无新目标会再次超时。STOP 优先于普通排队操作；`STOPPED` 事件表示开始受控停止，实际停止完成以 `AWAIT_RESUME` 状态为准。

## 4. 示例

```bash
# 启动双臂，以下命令操作左臂
roslaunch z1_bringup arm.launch

# 反馈有效、现场姿态静止且间距足够时使能
rostopic pub -1 /left_arm/control z1_msgs/ControlCommand '{command_id: 1, operation: 0}'

# 等待 Holding 后，从 joint_states 或 tcp_pose 反馈构造对应的有限目标。
# 不假定真机初始姿态；关节2/3需要满足其非对称限位。

# 流式模式，先读取 status.session_id，之后发布带当前 stamp 和递增 sequence 的目标
rostopic pub -1 /left_arm/control z1_msgs/ControlCommand '{command_id: 3, operation: 4}'
rostopic echo /left_arm/status

# 停止、恢复（恢复后必须用新的 session_id）
rostopic pub -1 /left_arm/control z1_msgs/ControlCommand '{command_id: 4, operation: 2}'
rostopic pub -1 /left_arm/control z1_msgs/ControlCommand '{command_id: 5, operation: 3}'
```

流式发布者应在有效期内持续送帧，如 20–100 Hz；原生控制周期默认 500 Hz，状态发布默认 100 Hz。消息频率不等于新 MCU 测量频率。不要用固定 token 的 `rostopic -r` 当作恢复后的控制器。

### 带时间和导数的参考

新消息的 `header.stamp` 是上游生成时刻，仍需满足源消息时效；`target_time` 是该路点的目标时刻，零值表示使用 `header.stamp`。非零值应不早于源时刻，默认不超过当前控制时刻后 500 ms。多个路点按目标时间递增发送，sequence 在整个会话中递增；较早的新路点会截去后续旧计划。每条流选择一种输入 topic。

`has_velocity=false` 时根据相邻点估计并滤波速度；省略加速度时同样估计加速度。提供加速度必须同时提供速度。关节单位为 rad/s、rad/s²；笛卡尔为 m/s、rad/s、m/s²、rad/s²，角速度和角加速度均是基座系向量，不能填写欧拉角导数。两端提供加速度时使用五次 Hermite 插值，否则为三次；姿态在 SO(3) 上插值并转换旋转导数。

默认前瞻 30 ms、最多外推 100 ms、导数估计滤波时间常数 10 ms；外推期后保持末位置并请求零导数，流超时仍进入停止流程。参数分别为 `reference_preview`、`reference_extrapolation`、`reference_velocity_filter`；最大未来范围为 `reference_horizon`。这些时间用于生成期望参考，实际轨迹还受 v/a/jerk、关节及几何约束，不保证严格按时经过每个路点。阻抗增益平滑时间常数为 `impedance_gain_time_constant`，默认 100 ms。

### 预计算定时轨迹和连续路径

`joint_trajectory` 接收完整有限计划。`time_from_start` 为执行开始后的秒数，使用 ROS `duration`；它与流式消息的绝对 `target_time` 不同。首点时间可为正，此时自动补入当前参考和零导数的 t=0 点；首点为零时必须匹配当前静止参考。点时间严格递增，提供加速度必须同时提供速度。首尾必须静止，完成后进入 Holding。所有有限运动共用递增 command_id，不使用流式 session 或续期超时。

相邻点构成 C2 五次 Hermite 轨迹。显式给出的 v/a 被用于端点条件；省略内部速度时根据相邻斜率估计，反向处取零；省略加速度时取零。`retime=false` 保持原时间及显式导数，任何分段内部的关节位置或 v/a/jerk 超限都拒绝；`retime=true` 允许一次全局时间伸缩，保持关节路径形状，v/a/jerk 分别按伸缩倍率的一、二、三次方降低。重定时不能修复越过位置限位或碰撞的路径。

`joint_waypoints` 自动补入当前参考，不要求上游提供时间。内部同方向关节路点通常以非零速度通过，反向或相关轴停顿可能需要减速/停止；不能承诺任意路径都不停。`speed_scale=s` 对基准 v/a/jerk 分别施加 s、s²、s³，随后验证并全局伸缩。当前时间安排并非全局时间最优。

`linear_path` 的单点目标对应原生 `moveL`：TCP 平移沿直线，姿态沿 SO(3) 最短旋转，使用共同路径进度。准备阶段完成连续分支 IK 和关节多项式拟合，执行阶段直接采样。`speed_scale` 缩放 0.12 m/s 和 0.5 rad/s 的任务速度基准；关节 v/a/jerk 需要时进一步降低整条路径速度。

多点 `linear_path` 的 `blend_radius=0` 表示到每个路点停止；正值表示在转角附近用 C2 曲线圆滑通过，不精确经过该中间点。半径限制两侧裁剪长度，度量为 `max(平移距离[m], 0.24×旋转角[rad])`，并限制为相邻段长度的 45%，因此纯旋转也能连续连接。平移曲线在裁剪转角的凸包内；实际关节拟合另有数值误差。它不是 MoveC 圆弧指令，也不保证比逐点停车耗时更短。

原生 `LinearPathOptions` 默认平移误差阈值 0.1 mm、旋转 1 mrad。准备阶段在每个关节样条段的 9 个时间点检查 FK 与目标路径，超差则加密，最多 5 轮；这些是数值拟合验收阈值，不是连续区间的解析误差上界，也不代表真实机械臂精度。路径不可达、连续 IK 分支断开或拟合仍超差时拒绝，不自动寻找绕障路线。原 `pose_goal` 保持终点 IK 后关节运动的语义。

规划、拟合、全段几何检查均在提交线程完成，不持有控制锁。ROS 有限运动使用独立的单线程规划回调队列，操作和流式消息使用普通回调线程；原生执行仍为 500 Hz。STOP/DISABLE 会使尚在准备的旧计划失效。已接受路径按单调时钟直接采样，不再经过流式 Ruckig 前瞻；在线制动和几何检查继续使用同一控制内核。

以下原生调用均需从 Holding 发起，等待上一运动结束后再提交下一条：

```cpp
z1::TrajectoryPoint end;
end.time_from_start = 1.0;
end.position = desired_q;
end.has_velocity = end.has_acceleration = true; // 默认零导数
controller.trajectory(20, {end});               // 严格保持 1 秒
controller.jointWaypoints(21, {q1, q2}, 0.5);  // 自动连续过点
controller.moveL(22, desired_pose, 0.5);        // 单段直线
z1::LinearPathOptions path;
path.speed = 0.5;
path.blend_radius = 0.003;
controller.linearWaypoints(23, {pose1, pose2}, path);
```

ROS 定时轨迹示例（当前为 Holding，`q_target` 按 joint1..joint6 排列）：

```python
from z1_msgs.msg import JointTrajectory, TrajectoryPoint
pub = rospy.Publisher('/left_arm/joint_trajectory', JointTrajectory, queue_size=1)
m = JointTrajectory()
m.command_id = 20
m.header.stamp = rospy.Time.now()
p = TrajectoryPoint()
p.time_from_start = rospy.Duration(1.0)
p.positions = q_target
p.has_velocity = p.has_acceleration = True
m.points = [p]
pub.publish(m)
```

原生 C++ 与 ROS 无关，时间采用 SDK 单调时钟秒数：

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

ROS 示例片段（已进入 JointStream 并读取当前 `session`）：

```python
from z1_msgs.msg import JointReference
pub = rospy.Publisher('/left_arm/joint_reference', JointReference, queue_size=32)
m = JointReference()
m.header.stamp = rospy.Time.now()
m.target_time = m.header.stamp + rospy.Duration(0.04)
m.session_id, m.sequence = session, sequence
m.positions, m.velocities, m.accelerations = q, dq, ddq
m.has_velocity = m.has_acceleration = True
pub.publish(m)
```

## 5. 模型配置

默认 [control.yaml](z1_bringup/config/control.yaml) 中所有向量可逐臂覆盖。配置先加载再由 launch 覆盖地址、前缀与后端。双臂可用不同配置、模型/描述文件和基座系重力；底层控制代码相同。

默认 `period=0.002`、`receive_budget_us=1000`、`lookahead_budget=0.0005`。接收和在线检查串行执行；ROS 适配器要求两项预算之和小于周期，余量供其他控制计算和调度使用。这些是时间预算，不代表每轮都会用满。有限目标的离线规划预算不占控制锁。

`control_cpu=-1` 表示不指定 CPU；其他值为部署机上的逻辑 CPU 编号，13700H 应按实际拓扑选择性能核。`realtime_priority=0` 沿用默认调度，正值请求该控制线程使用 SCHED_FIFO，所需操作系统权限由部署环境提供。原生构造可传入 `RuntimeOptions{priority, cpu}`；`Runtime::timing()` 返回统计，持续时间单位为微秒。

ROS `status` 新增字段：

| 字段 | 含义 |
| --- | --- |
| `runtime_cycles` / `missed_control_slots` | 主机运行周期数 / 跳过的过期时隙；不代表 MCU 丢包数 |
| `exchange_time` / `max_exchange_time` | 最近 / 最大 Transport::exchange 耗时，秒 |
| `wakeup_lateness` / `max_wakeup_lateness` | 最近 / 最大相对预定时刻的启动迟到，秒 |
| `realtime_scheduling` / `cpu_affinity_applied` | 请求的线程优先级 / CPU 亲和性是否成功应用 |
| `realtime_error` / `cpu_affinity_error` | 线程设置错误码；0 表示无错误，默认不请求设置时也为0 |
| `trajectory_duration` / `trajectory_time_scale` | 最近接受的有限轨迹时长（秒）/ 全局时间伸缩倍率；进入流式模式重置为 0 / 1 |

计数和最大值在 Runtime 启动时清零。`overruns` 仍表示控制计算超期，与漏过调度时隙分别统计。超期后运行循环跳过已过期的时隙，避免突发追赶；流式参考按实际经过时间推进。旧 MCU 接口仍为发送上一轮输出后接收反馈，新反馈生成的输出在下一轮事务发送。

固定结构格式：

```yaml
fixed_boxes:
  - name: measured_support
    size: [0.10, 0.10, 0.20]
    xyz: [0.30, 0.00, -0.20]
    rpy: [0.00, 0.00, 0.00]
```

数值仅说明格式，需替换为实测安装几何。排除碰撞对基于连杆名，工具/模型变更后须审查规则。每对几何的运动范围由模型自动推导，共同祖先的刚体运动不会膨胀自碰撞距离。修改质量/质心/惯性通过控制 URDF 完成；显示 xacro 应同步。几何变化不要求修改 Coal 距离算法。

`status.model_revision` 是控制 URDF 内容、TCP 名称、重力、碰撞排除对和固定结构配置的 SHA-256。它不包含外部网格文件内容或控制器增益；当前默认碰撞几何均为 URDF 内的基本形状。
