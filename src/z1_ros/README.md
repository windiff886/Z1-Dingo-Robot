# Z1 机械臂

这个项目仅面向 Unitree Z1 真机，负责控制六个主关节，以及设备存在时的夹爪。上层通过 ROS 1 Topic 告诉它运动目标，它根据 MCU 返回的真实反馈生成运动过程，计算关节力矩，再根据实际运动持续修正输出。每只机械臂独立运行这一闭环。

读运行过程前，先区分三个量：

| 名称 | 表示什么 | 例子 |
| --- | --- | --- |
| 用户目标 | 希望机械臂最终到哪里，或接下来如何运动 | 六个关节角、一个 TCP 位姿、一串带时间的路点 |
| 内部参考 | 控制器安排的、当前时刻应该达到的位置、速度和加速度 | 从当前位置平滑走向目标时，每 2 ms 的期望状态 |
| 实际反馈 | 机械臂当前测得的位置、速度和力矩 | `joint_states` 发布的数据 |

TCP（工具中心点）的位姿对应模型中 `tool_frame` 指定的工具坐标系，默认是 `tool`。运动目标、参考和反馈可以不同；控制器的工作就是在运动限制内，让实际状态跟上参考。

通过完整机器人模型拖动 TCP 发送测试目标，见 [RViz 末端目标测试](unidingo_rviz_test/README.md)。该入口明确将 `base_link` 中的目标转换到各臂 `link00`，再使用原有 `pose_goal` 接口；默认 TCP 为 `link06` 沿 X 正方向 0.051 m 的固定坐标系。

一次有限运动的主要过程如下：

```text
启动，开始读取反馈
  → 等待 ENABLE
  → 以当前姿态进入保持
  → 接收目标，准备并检查运动
  → 按时间取出参考，结合反馈计算力矩
  → 发送输出，读取下一次反馈，反复修正
  → 实际到达并稳定，报告成功，继续保持
```

流式跟踪和阻抗共用反馈、通信及状态管理流程，但参考生成和力矩计算有所不同，下面分别说明。

## 1. 启动后，先知道机械臂现在在哪里

完成固件、力矩模式、工具/负载、安装模型及参数的现场验收后，在已经构建并 source 的 ROS 1 工作区中启动单臂：

```bash
roslaunch z1_bringup arm.launch hardware_validated:=true \
  mcu_ip:=192.168.123.110 local_port:=8881
```

默认命名空间是 `/arm`，关节及 TF 前缀是 `arm_`。启动时先读入控制模型和参数：关节限位、质量与惯性、工具位置、基座系重力，以及用于距离检查的几何。它们分别用于计算末端位置、计算力矩、判断运动是否超限或接近碰撞。

接下来通过官方 MCU 通信适配完成 UDP 握手，开始周期性收发：输出发给真实关节，反馈来自 MCU 返回的报文。程序记录有效报文的主机接收时间和接收序号，用于判断反馈是否过期；主机接收时间并不代表固件采样时刻。

运行入口固定使用 MCU 通信，未连接设备或握手失败就退出。`hardware_validated` 默认是 `false`，未显式确认时在连接前拒绝启动；它是现场确认后的启动条件，程序不会据此自动完成硬件验收。当前待确认内容见[实施记录](IMPLEMENTATION_REPORT.md#6-真机待验收)。旧的 `backend` 参数已移除，残留参数会报错。

**启动完成后仍处于 `Disabled`，不会自动执行运动。**此时可以分别在两个终端观察：

```bash
rostopic echo /arm/status
rostopic echo /arm/joint_states
```

`feedback_valid=true` 表示最近的反馈通过了时龄、数值、关节范围、速度和可用设备故障信息的检查。默认反馈有效期是 80 ms；某一轮没有新报文时，可以暂用有效期内的上一份反馈。

## 2. 使能后，先保持当前姿态

向 `control` 发送 `ENABLE`：

```bash
rostopic pub -1 /arm/control z1_msgs/ControlCommand \
  '{command_id: 1, operation: 0}'
```

控制器在反馈有效、机械臂基本静止、模型几何间距足够时接受使能。它把**当前测得的关节位置**设为初始参考，将参考速度、加速度清零，进入 `Holding`。

因此，使能不会让机械臂回零，也不会把它移动到某个预设姿态。保持状态下，控制器已经在持续计算重力补偿和位置修正力矩，等待下一条命令。

应以 `status.mode=HOLDING`（数值 1）确认状态转换，发送命令本身不代表转换已经完成。

## 3. 收到目标后，决定接下来怎样走

运动请求分为三类：一次提交的有限运动、持续更新目标的流式跟踪，以及以位姿为平衡点的阻抗控制。

### 3.1 有限运动：先准备完整过程，再开始执行

有限运动在 `Holding` 时接收。所有有限运动共用正数、递增的 `command_id`，一次只执行一条；忙时提交的新运动会被拒绝。连续过点应放在同一条路径中提交。

不同请求决定了怎样把用户目标变成内部参考：

| 用户发来的内容 | Topic | 控制器做什么 |
| --- | --- | --- |
| 一个六轴关节目标 | `joint_goal` | 用 Ruckig 生成受速度、加速度和 jerk（加速度变化率）限制的关节运动 |
| 一个 TCP 目标位姿 | `pose_goal` | 先用逆运动学 IK 求终点关节角，再生成关节运动 |
| 已经安排好时间的关节轨迹 | `joint_trajectory` | 根据每点的时间、位置及可选导数，准备分段五次关节样条 |
| 一串关节路点 | `joint_waypoints` | 自动安排时间和内部过点速度，准备关节样条，首尾停止 |
| 一个或多个 TCP 位姿 | `linear_path` | 插值笛卡尔路径，沿途求连续 IK，再拟合关节样条 |

**`pose_goal` 约束终点位姿；需要 TCP 沿直线走时使用 `linear_path`。**后者的单点请求对应 MoveL：位置沿直线变化，姿态沿最短旋转变化。多点路径的 `blend_radius=0` 表示逐点停车；正值表示裁剪转角并圆滑通过，不精确经过被裁剪的中间点。

对于定时轨迹，`time_from_start` 是相对执行开始的时间。首点时间大于零时，自动补入当前静止参考作为起点；首点为零时必须匹配当前静止参考。首尾均要求静止。默认 `retime=false` 保留原时间，超出运动限制则拒绝；`retime=true` 允许整体放慢，但不能修复越界或碰撞路径。

`speed_scale` 的当前行为随指令不同：关节/位姿单目标只缩放关节速度上限；关节路点对速度、加速度、jerk 分别施加 `s`、`s²`、`s³`；笛卡尔路径缩放任务空间速度，必要时再整体放慢以满足关节限制。倍率不能直接理解为精确的执行时长比例。

准备期间还会检查关节限位、轨迹内部的运动限制及整条参考的几何间距。路径不可达、IK 失败、检查失败或准备期间状态发生变化，都会拒绝该请求。这里会检查给定路径，不会自动搜索绕障路线。

准备和检查通过后，才报告 `ACCEPTED` 并切入执行状态。耗时的路径准备与周期控制分开进行，期间控制器继续保持；`STOP` 或 `DISABLE` 会使尚在准备的旧计划失效。

### 3.2 流式跟踪：一边运动，一边接收新的参考

遥操作或视觉跟踪的目标会持续变化，因此先选择跟踪模式，再不断发送参考：

| 需要跟踪什么 | 先向 `control` 发什么 | 随后选择一种输入 Topic |
| --- | --- | --- |
| 关节位置 | `JOINT_STREAM`，operation=4 | `joint_stream` 或 `joint_reference` |
| TCP 位姿 | `POSE_STREAM`，operation=5 | `pose_stream` 或 `pose_reference` |

发布器应先准备好。模式进入后，读取 `status.session_id`，用该会话号、递增的 `sequence` 和非零 `header.stamp` 持续送帧；默认进入模式后 200 ms 内没有有效目标就会开始停止。

纯位置消息由控制器估计导数；`joint_reference` 和 `pose_reference` 还允许提供目标时刻、速度、加速度，提前描述一小段未来运动。`header.stamp` 是消息生成时间，`target_time` 是路点期望时间，后者为零时使用前者。它与有限轨迹的相对 `time_from_start` 不同。

接收到合法帧后，控制器把它放入单调时钟时间线。在每个控制周期中：

1. 在时间线上插值，或在有限时间内外推，得到即将跟踪的参考；默认前瞻 30 ms、最多外推 100 ms。
2. 关节参考的位置、速度、加速度进入 Ruckig；TCP 参考通过差分 IK 转成关节速度，并使用目标 TCP 速度作为前馈。
3. 从已有内部参考连续接续，生成满足运动限制的下一步关节参考，再进入共同的执行闭环。

上层可按 20–100 Hz 提供目标，内部默认按 500 Hz 执行。每帧是可以被后续帧修订的参考，不承诺逐帧到达，也没有逐帧成功事件。默认最多保存 32 个参考点、接收未来 500 ms 内的路点；更新较早的路点会替换该时刻及其后的旧计划。

过期帧、旧会话、倒序序号和无效目标不刷新有效期。普通输入按源时间计算超时；未来路点允许执行到最后有效路点的目标时间，再加默认 200 ms。超时后的处理见第 6 节。

### 3.3 阻抗：把目标位姿作为弹簧的平衡位置

在 `Holding` 时发送 `IMPEDANCE`（operation=6），控制器会以实际 TCP 位姿建立初始平衡点，创建新会话。随后持续通过 `pose_stream` 或 `pose_reference` 更新平衡位姿，遵守同样的会话和超时规则。

阻抗中的恢复作用由两部分形成：位姿误差产生的弹簧作用，以及实际 TCP 速度与目标速度之差产生的阻尼作用。控制器用雅可比转置把这些作用映射到关节力矩，再加上模型动力学补偿。

`impedance_gains` 可以在 `Holding` 或 `Impedance` 时设置六个方向的刚度、阻尼，顺序为平移 XYZ、旋转 XYZ；阻抗运行中的增益更新默认以 100 ms 时间常数平滑生效。关节参考仍用于运动边界和几何检查，阻抗力矩直接依据笛卡尔平衡参考计算。

这个过程不依赖外力传感器或外力估计，也没有目标接触力闭环。调低刚度会减小相同位姿偏差产生的恢复作用，实际柔顺效果还取决于模型、增益和设备行为，需要在真机上测量。

## 4. 执行时，每 2 ms 具体做什么

有限运动和流式跟踪最终都进入周期闭环，阻抗使用其中相同的反馈和几何检查。默认周期为 2 ms（500 Hz），一轮的实际顺序如下：

```text
发送上一轮算好的输出
  → 接收反馈，确认是否有新测量
  → 优先处理失能，检查反馈，计算当前 TCP 和几何间距
  → 处理其余控制操作，检查当前模式及超时条件
  → 根据当前模式取出或生成下一步参考
  → 检查下一段运动及制动路径
  → 结合实际反馈计算力矩，限制力矩和力矩变化率
  → 保存输出，等下一轮发送
```

第一轮从被动输出开始。当前 MCU 收发接口是先发后收，因此新反馈算出的输出会在下一轮通信时发送。

**先检查反馈和状态。**按接收序号更新新测量，再判断保存的反馈是否有效；有效期内允许复用旧测量。失能操作优先处理，其余操作需要有效反馈和时序。反馈通过检查后，控制器根据实测关节角计算当前 TCP 位姿，并查询模型中的最小几何间距。

**再决定当前参考。**有限运动按实际经过的单调时钟时间直接采样准备好的轨迹；流式模式按上一节的方法在线生成参考；保持模式沿用保持参考。有限轨迹不会再次经过流式前瞻和第二次在线整形。

**然后检查能否继续及能否停下。**运动期间同时检查下一段运动的几何间距，以及从候选参考减速到零的制动路径，并考虑实测关节与参考的偏差。无法保留合适的制动路径时尝试受控停止；制动本身也无法生成或通过检查时进入故障。几何检查只覆盖模型已描述的物体，默认固定结构配置为空，另一臂和动态环境不在单臂检查范围内。

**最后计算力矩。**普通位置跟踪先计算：

```text
期望关节加速度 = 参考加速度
                 + Kp × (参考位置 - 实测位置)
                 + Kd × (参考速度 - 实测速度)
```

再根据实测位置、速度和该期望加速度，通过动力学模型求出关节力矩，包含惯性、重力和科氏等项。阻抗模式使用上一节的弹簧阻尼计算。两种模式都限制最终力矩及其变化率。

主臂的位置反馈控制在主机完成，发送给 MCU 的主臂 Kp/Kd 为零；夹爪使用单独的位置增益。真实固件对这些命令的执行语义仍需现场验收。主机 500 Hz 循环也不代表已确认 MCU 具有相同的采样和执行频率。

## 5. 怎样知道已经执行完成

ROS 默认按 100 Hz 发布运行情况：

| Topic | 运行中主要看什么 |
| --- | --- |
| `joint_states` | 实测关节位置、速度、力矩；有夹爪时追加夹爪电机反馈 |
| `tcp_pose` | 根据有效实测关节角计算的 TCP 位姿 |
| `status` | 当前模式、原因、反馈有效性和时龄、会话号、正在执行的命令、碰撞距离及周期统计 |
| `result` | 某个 `command_id` 是否接受、成功、拒绝、取消、开始停止或失败 |

轨迹的计划时间走完后，控制器继续检查实际状态。默认要求关节位置误差小于 0.015 rad、关节速度小于 0.03 rad/s，并稳定超过 100 ms；位姿目标和笛卡尔路径还要求 TCP 位置误差小于 5 mm、姿态误差小于 0.025 rad。

满足条件才报告 `SUCCEEDED`，回到 `Holding`；超过计划时长再加默认 3 s 仍未稳定，则进入故障并报告失败。因此，`ACCEPTED` 和计划时间结束都不能代替实测到达结果。

`joint_states` 可能继续发布保存的旧测量，必须结合 `status.feedback_valid`、`feedback_age` 判断是否可用；`tcp_pose` 仅在反馈有效时发布。`result` 是有界事件流，应在发命令前订阅并按 ID 匹配，不能当永久查询记录。

## 6. 停止、恢复和故障分别怎样改变流程

用户 `STOP`、流式超时或接近几何警告距离，都可能让运动转入受控停止。停止会清除旧目标、使旧会话失效，并准备减速轨迹：

```text
正常保持或运动 → Stopping → 实测速度降到阈值内 → AwaitResume
```

`STOPPED` 结果表示开始停止，停止完成应看 `status.mode=AWAIT_RESUME`（数值 8）。进入此状态后仍保持控制输出，不会因新目标到来自动继续运动。

| 此时想做什么 | 操作 | 后续行为 |
| --- | --- | --- |
| 恢复原来的流式或阻抗模式 | `RESUME`，operation=3 | 创建新会话；重新读取会话号，发送新目标 |
| 确认有限运动已停止 | `RESUME`，operation=3 | 回到 `Holding`，需重新提交运动 |
| 结束旧模式，改做别的运动 | `HOLD`，operation=7 | 回到 `Holding`，再选择新模式或提交有限目标 |
| 请求被动输出 | `DISABLE`，operation=1 | 清除目标并进入 `Disabled` |

`RESUME` 不会接着执行已经清除的有限轨迹；流式恢复后也不会重新执行旧缓存。即使只想继续跟踪同一位置，也要使用新会话发送新帧，否则仍会再次超时。

反馈超时/异常、设备故障、几何碰撞风险、无效制动路径或控制计算超期等情况可能进入 `Fault`。此时清除目标并切换为被动输出，`status.reason` 记录原因；有活动的有限运动时报告 `FAILED`。反馈失效时，软件无法继续保证可靠的闭环减速。

故障条件解除、反馈有效且静止、几何间距足够后，`RESUME` 先将故障确认到 `Disabled`，还需要 `ENABLE` 才会重新保持。被动输出不等于硬件急停或机械制动已确认。

正常退出进程时，程序在适用状态下尝试最多 2 s 的受控停止，最后尝试发送一次被动命令；这也不是固件停止确认。异常断电、进程强杀和失联后的设备行为取决于 MCU 看门狗及硬件。

## 7. 夹爪和双臂怎样参与运行

夹爪存在时，`status.has_gripper=true`。在 `Holding` 中向 `gripper_goal` 发送 `[-1, 0] rad` 的电机位置目标，负值表示打开。周期循环限制位置更新幅度，再交给夹爪位置控制；结果只表示接受或拒绝，没有到位或夹持力结果。

启动双臂时，两只机械臂分别运行前面完整的流程：

```bash
roslaunch z1_bringup dual_arm.launch hardware_validated:=true
```

左臂使用 `/left_arm`、`left_` 前缀，右臂使用 `/right_arm`、`right_` 前缀。各自使能、接收目标、生成参考、执行和停止，没有统一的双臂指令或同步启动保证；需要协同运动时，由上层分别管理。单臂几何检查也不包含另一只正在运动的机械臂。

两臂可分别指定配置、模型、描述文件、IP 和本地端口；默认 IP 为 `192.168.123.110/111`，本地端口为 `8881/8882`，MCU 目标端口为 `8880`。安装方向不同时，应分别设置基座系重力和实测固定结构。具体参数见[双臂启动文件](z1_bringup/launch/dual_arm.launch)及[模型配置说明](INTERFACES.md#5-模型配置)。

## 8. 在单臂真机上走一遍完整过程

先完成第 1 节的现场验收，确认当前姿态及其附近的小幅运动可行。此流程会连接和驱动实际机械臂，不假定任何启动关节角。在已构建并 source 的工作区中，先在一个终端运行：

```bash
roslaunch z1_bringup arm.launch hardware_validated:=true \
  mcu_ip:=192.168.123.110 local_port:=8881
```

另开两个终端，分别持续观察状态和结果：

```bash
rostopic echo /arm/status
```

```bash
rostopic echo /arm/result
```

确认 `feedback_valid=true` 后，在命令终端使能：

```bash
rostopic pub -1 /arm/control z1_msgs/ControlCommand \
  '{command_id: 1, operation: 0}'
```

等待 `mode=1`（Holding）并让初始保持稳定后，从当前反馈构造小幅关节运动。下面将第一关节目标增加 0.01 rad，其他关节保持原位置；现场仍需确认该方向可行。新启动的控制进程中使用命令 ID 2，后续有限运动继续递增：

```python
import time
import rospy
from sensor_msgs.msg import JointState
from z1_msgs.msg import JointGoal, Status

rospy.init_node('z1_small_joint_goal')
pub = rospy.Publisher('/arm/joint_goal', JointGoal, queue_size=1)
deadline = time.monotonic() + 5
while pub.get_num_connections() == 0:
    if time.monotonic() > deadline:
        raise RuntimeError('机械臂命令订阅尚未连接')
    rospy.sleep(0.01)

status = rospy.wait_for_message('/arm/status', Status, timeout=2)
if not status.feedback_valid or status.mode != Status.HOLDING:
    raise RuntimeError('需要有效反馈并处于 Holding')
joints = rospy.wait_for_message('/arm/joint_states', JointState, timeout=2)
goal = JointGoal()
goal.command_id = 2
goal.positions = [joints.position[joints.name.index(f'arm_joint{i}')]
                  for i in range(1, 7)]
goal.positions[0] += 0.01
goal.speed_scale = 0.1
goal.header.stamp = rospy.Time.now()
pub.publish(goal)
rospy.sleep(0.1)
```

这条请求会先准备、检查关节轨迹，再报告 `command_id=2, outcome=0`（ACCEPTED）；运动中模式为 `2`（JointGoal）。实际到达并稳定后报告 `outcome=1`（SUCCEEDED），模式回到 `1`。

随后可以从保持状态观察停止和恢复：

```bash
rostopic pub -1 /arm/control z1_msgs/ControlCommand \
  '{command_id: 3, operation: 2}'
```

等待 `mode=8`（AwaitResume）后，恢复到保持：

```bash
rostopic pub -1 /arm/control z1_msgs/ControlCommand \
  '{command_id: 4, operation: 3}'
```

确认重新进入 `mode=1`，再失能：

```bash
rostopic pub -1 /arm/control z1_msgs/ControlCommand \
  '{command_id: 5, operation: 1}'
```

上述控制命令使用零时间戳，按接收处理；Python 目标使用发送时刻。主臂数组固定按 `joint1..joint6` 排列，角度用 rad、位置用 m。发送位姿时必须填写该臂基座系，例如 `/arm` 使用 `arm_link00`，左臂使用 `left_link00`；四元数顺序为 `(x,y,z,w)`，不自动通过 TF 转换。

## 运行前准备与进一步阅读

ROS 运行需要先在 ROS 1 工作区中构建并 source 消息及节点。原生依赖为 C++17、CMake、Eigen、Pinocchio 4.x/Coal、Ruckig ≥0.9.2、Boost 和 OpenSSL；ROS 侧另需 roscpp、消息生成、xacro 和 robot_state_publisher。

可以独立构建控制库并运行软件契约检查。在工作区根目录执行，依赖位置按实际环境替换：

```bash
cmake -S src/z1_ros/z1_control -B /tmp/z1-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/path/to/native/dependencies
cmake --build /tmp/z1-build -j1
ctest --test-dir /tmp/z1-build --output-on-failure
```

自动检查使用合成反馈及本机 UDP 报文回环验证算法和通信契约；这些测试夹具不作为运行模式或 SDK 后端提供。实际运动、负载、停止距离和阻抗效果以真机测量为准。

- [接口契约与消息示例](INTERFACES.md)：所有命令字段、时间、坐标系、会话和操作码。
- [默认运行参数](z1_bringup/config/control.yaml)：周期、超时、运动限制、增益和几何配置。
- [实施记录](IMPLEMENTATION_REPORT.md)：已有验证及真机待验收内容。
- [需求基线](REFACTOR_REQUIREMENTS.md)：当前能力范围和约束。
- [SDK 技术审查](SDK_TECHNICAL_REVIEW.md)及[计算检查与历史记录](tools/performance_audit/README.md)：设计依据、计算开销与历史实验。
