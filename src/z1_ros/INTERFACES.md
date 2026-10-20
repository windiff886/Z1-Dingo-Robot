# 单臂 Topic 与控制契约

## 1. 单位、时间和命名

- 主臂顺序固定 `joint1..joint6`；角度 rad，速度 rad/s，主臂力矩 Nm，位置 m。
- TCP 位姿 quaternion `(x,y,z,w)` 需单位长度。frame_id 必须是该臂的 `<prefix>link00`，例如 `left_link00`；不隐式查询 TF 转换。
- `joint_states` 的 name 带前缀；夹爪存在时追加 `<prefix>gripper_motor`，这是独立电机反馈，不声称对应描述模型的某个夹爪连杆角度。
- 命令 header.stamp：有限目标/控制可为零（按接收处理），流式必须非零；默认允许过去 200 ms、未来 20 ms。有效流式帧的 ROS 时龄会扣减单调时钟有效期，延迟帧不会获得完整的新有效期；无效帧不续期。
- 内核超时使用单调时钟；ROS 时间跳变不重置控制期限。ROS header 是发布时刻，实际有效接收序号和接收时龄位于 status。没有固件采样时刻。
- 每种命令 topic 的队列为 1；运行控制队列有界。多发布者必须在上层协调，本轮没有自动控制权竞争机制。

## 2. 命令

所有类型位于 [z1_msgs/msg](z1_msgs/msg)。

| Topic | 消息 / 字段 | 约束 |
| --- | --- | --- |
| `joint_goal` | JointGoal：command_id、positions[6]、speed_scale | ID 正数、递增；speed_scale `(0,1]`；Holding 接收 |
| `pose_goal` | PoseGoal：command_id、pose、speed_scale | 同上；指定基座 frame_id；局部 IK 失败则拒绝 |
| `joint_stream` | JointStream：session_id、sequence、positions[6] | JointStream 模式；token 等于 status，会话内 sequence 递增 |
| `pose_stream` | PoseStream：session_id、sequence、pose | PoseStream 或 Impedance 模式；基座 frame_id；连续参考，不承诺到达每帧 |
| `impedance_gains` | ImpedanceGains：command_id、stiffness[6]、damping[6] | Holding；顺序 XYZ/旋转XYZ，对角正值；平移 K≤500 N/m，旋转 K≤30 Nm/rad，D≤100 |
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
| `status` | mode、reason、model_revision、session_id、active_command_id、feedback_sequence、feedback_valid、new_feedback、feedback_age、has_gripper、collision_distance/pair、compute_time、loop_period/max_loop_period、overruns |
| `result` | command_id + ACCEPTED/SUCCEEDED/REJECTED/CANCELED/STOPPED/FAILED + reason |

模式数值为 Disabled=0、Holding=1、JointGoal=2、PoseGoal=3、JointStream=4、PoseStream=5、Impedance=6、Stopping=7、AwaitResume=8、Fault=9。

有限目标接受后才会有最终结果；最终成功需实际 q/dq 满足容差并稳定 100 ms，末端目标还需实际 TCP 满足位姿容差。到达超期、IK/轨迹失败不报成功。持续流没有“每帧到达”承诺，拒绝的流式帧在 ROS 日志报告；模式和停止原因通过 status 判断。result 为有界事件流（128），发布者应及时订阅，不能当永久审计日志。

默认超时：反馈 80 ms、流式 200 ms。收到无效/过期/旧会话/倒序目标不会刷新有效期。会话初值每个进程随机生成，模式进入/停止/恢复均更新。新会话不能执行停止前或停止期间缓存，恢复后无新目标会再次超时。

## 4. 示例

```bash
# 单臂启动默认 namespace=arm, prefix=arm_
roslaunch z1_bringup arm.launch

# 使能：仿真等待反馈稳定后发送目标
rostopic pub -1 /arm/control z1_msgs/ControlCommand '{command_id: 1, operation: 0}'

# 小幅有限关节目标；关节2/3需要满足其非对称限位
rostopic pub -1 /arm/joint_goal z1_msgs/JointGoal \
  '{command_id: 2, positions: [0.05, 0.8, -1.1, 0.3, 0.0, 0.0], speed_scale: 0.2}'

# 流式模式，先读取 status.session_id，之后发布带当前 stamp 和递增 sequence 的目标
rostopic pub -1 /arm/control z1_msgs/ControlCommand '{command_id: 3, operation: 4}'
rostopic echo /arm/status

# 停止、恢复（恢复后必须用新的 session_id）
rostopic pub -1 /arm/control z1_msgs/ControlCommand '{command_id: 4, operation: 2}'
rostopic pub -1 /arm/control z1_msgs/ControlCommand '{command_id: 5, operation: 3}'
```

流式发布者应在有效期内持续送帧，如 20–100 Hz；原生控制周期仍可为 250 Hz。消息频率不等于新 MCU 测量频率。不要用固定 token 的 `rostopic -r` 当作恢复后的控制器。

## 5. 模型配置

默认 [control.yaml](z1_bringup/config/control.yaml) 中所有向量可逐臂覆盖。配置先加载再由 launch 覆盖地址、前缀与后端。双臂可用不同配置、模型/描述文件和基座系重力；底层控制代码相同。

固定结构格式：

```yaml
fixed_boxes:
  - name: measured_support
    size: [0.10, 0.10, 0.20]
    xyz: [0.30, 0.00, -0.20]
    rpy: [0.00, 0.00, 0.00]
```

数值仅说明格式，需替换为实测安装几何。排除碰撞对基于连杆名，工具/模型变更后须审查规则和半径上界。修改质量/质心/惯性通过控制 URDF 完成；显示 xacro 应同步。几何变化不要求修改 Coal 距离算法。

`status.model_revision` 是控制 URDF 内容、TCP 名称、重力、碰撞排除对和固定结构配置的 SHA-256。它不包含外部网格文件内容或控制器增益；当前默认碰撞几何均为 URDF 内的基本形状。
