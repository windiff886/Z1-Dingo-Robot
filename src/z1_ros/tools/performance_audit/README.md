# 离线性能与跟踪效果基准

该工具直接调用当前 z1_core，通过 Pinocchio 仿真和合成反馈审查参考生成、计算开销及阻抗行为。不会实例化 MCU 通信或创建网络套接字，不需要 ROS。

```bash
cmake -S src/z1_ros/tools/performance_audit -B /tmp/z1-performance-audit-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/ros/humble
cmake --build /tmp/z1-performance-audit-build \
  --target performance_audit allocation_audit runtime_rate_probe -j2
/tmp/z1-performance-audit-build/performance_audit \
  src/z1_ros/z1_description/urdf/z1_control.urdf
```

`/opt/ros/humble` 仅为审查机器上原生依赖的位置，可替换；依赖与 z1_control 相同。工具使用 GNU ld 的 `--wrap` 对 Model::clearance 的普通/带位移界两个重载及 clearanceSample 计时，不改写生产算法，包装符号对应当前六轴 Eigen 接口。

测量包括：

- FK、雅可比、RNEA、差分 IK 与整臂最小距离查询，预热 100 次、测量 3000 次。距离微基准按 `q[0]=0.05*sin(i*0.001)` 改变构型，触发几何更新，不测静止缓存命中；该运动对自碰撞距离的影响有限，另有独立多轴距离对照。
- 50 Hz 目标更新、0.03 rad 幅值、0.25/0.5 Hz 的 joint1 正弦运动，比较 250/500/1000 Hz 仿真控制步。其他关节目标固定，保留默认限值与增益。
- 当前工具以 500 Hz 运行位姿流：将相同目标关节轨迹经 FK 变成完整 TCP 位姿，测量 TCP 误差。此时关节误差只用于补充观察，IK 可能使用其他关节实现任务。
- `dynamic_joint_reference`：50 Hz 提供解析 q/dq/ddq，经过完整生产控制流程；`scheduled_joint_reference`：同样信息，目标时间提前 40 ms。两者当前以 500 Hz 执行，区别于只发位置的兼容接口。
- 每条运动前保持 4 秒；运动持续 12 秒，忽略前 4 秒，统计剩余时间。仿真时间按固定步长推进，不等待墙钟时间。
- 给已建立平衡参考的 Impedance 模式注入 joint1 增加 0.01 rad 的反馈，比较几何间距和模式变化。

主要输出字段：

| 字段 | 含义 |
| --- | --- |
| simulation_rate_hz | 仿真算法步频，不是硬件收发率或调度测试 |
| rms_mrad | joint1 相对连续理想目标的 RMS 误差，mrad |
| target_ref_rms_mrad | 连续理想目标到生成参考的 joint1 RMS 误差 |
| ref_actual_rms_mrad | 生成参考到仿真实测的 joint1 RMS 误差 |
| lag_ms | 对 joint1 响应拟合 sin/cos/常数项，计算基频相位对应的时间偏移；不是网络延迟 |
| amplitude_ratio | joint1 响应基频幅值 / 目标幅值 |
| tcp_position_rms_mm / tcp_rotation_rms_mrad | TCP 平移误差范数 / SO(3) 旋转误差范数的 RMS |
| step_median_us / step_p99_us | Controller::step 自带计算计时的中位数 / P99，不含仿真积分和外部统计 |
| clearance_calls_per_step | 每个控制步调用整臂距离接口的平均次数，不是几何对数量 |
| collision_fraction | 几何查询累计墙钟耗时 / 控制计算累计墙钟耗时，包含轻量计时包装开销 |

`analytic_q_dq_ddq` 是诊断对照：用连续解析位置/速度/加速度参考，保留相同 RNEA、Kp/Kd、力矩限制和仿真，但绕过生产参考生成、几何检查与状态机。该对照有更充分的参考信息，不能把误差改善比例当作某个新 SDK 的收益，也不能作为可部署替代实现。

## 2026-10-23 改动前后

[基线输出](RESULTS_2026-10-23.txt) 与 [改动后输出](RESULTS_AFTER_2026-10-23.txt) 来自同机 Release 构建、相同模型、默认增益/运动限值与输入信号；统计时间段相同。墙钟耗时受 CPU 状态影响，未作为硬实时测试。以下均为 250 Hz 控制、50 Hz 上游、0.5 Hz 正弦：

| 指标 | 基线 | 改动后 |
| --- | --- | --- |
| 纯位置 JointStream，joint1 RMS | 7.521 mrad | 4.321 mrad |
| 纯位置 JointStream，基频滞后 | 113.84 ms | 57.79 ms |
| 同上，幅值比 | 0.984 | 1.055 |
| 纯位姿 PoseStream，TCP 平移 RMS | 1.895 mm | 0.0857 mm |
| 纯位姿 PoseStream，TCP 旋转 RMS | 16.085 mrad | 1.426 mrad |
| JointStream 控制步中位数 / P99 | 215.97 / 282.90 μs | 15.92 / 22.16 μs |
| JointStream 整臂距离调用 / 步 | 12.20 | 7.06 |
| 整臂距离微基准中位数 | 12.92 μs | 0.947 μs |
| 阻抗 0.01 rad yaw 受扰，间距均为 34.52 mm | 退出至 AwaitResume | 保持 Impedance |

新接口额外提供解析导数时，0.5 Hz 动态关节参考 RMS 为 0.233 mrad；40 ms 未来路点为 0.223 mrad。它们具有更充分的上游信息，不与旧接口改善比例混算。约 −3 ms 的拟合相位来自参考前瞻，不是负通信延迟。纯位置接口仍有滞后和幅值偏差，带噪声输入、突变与更大范围运动需要另外测量。

这些实验使用控制器与被控对象相同的动力学模型，没有真实摩擦、传感器噪声、接触或网络。结果用于定位软件链路瓶颈，不代表真机精度或任何实时保证。跨框架比较源码能力，不把其他论文的硬件成绩直接混入本机表格。

审查结论见 [SDK 技术审查](../../SDK_TECHNICAL_REVIEW.md)。

## 当前 500 Hz 复验

[本轮前](RESULTS_500_BEFORE_2026-10-23.txt)和[本轮后](RESULTS_500_AFTER_2026-10-23.txt)采用相同的算法步频与输入。本轮前是第一轮性能升级后的 SDK，显式设置 2 ms 周期和 0.5 ms 检查预算；不是最初的旧控制器。单次墙钟耗时存在噪声，另以 `taskset -c 2` 固定同一 CPU，按 before→after、after→before、before→after 交替复测各三次。完整六次输出、模型/二进制 hash 和汇总见[成对结果](RESULTS_500_PAIRED_2026-10-23.txt)。

以下为每次控制步中位数的三轮中位数，均为 500 Hz 执行、50 Hz 上游、0.5 Hz 正弦；不是池化 P99 或最坏耗时：

| 模式 | 本轮前 | 本轮后 |
| --- | --- | --- |
| 兼容 JointStream | 16.424 μs | 15.686 μs |
| 兼容 PoseStream | 14.012 μs | 12.320 μs |
| 原生 q/dq/ddq | 16.330 μs | 15.616 μs |
| 提前 40 ms 的 q/dq/ddq | 16.638 μs | 15.954 μs |

整步计算中位数有约 4%–12% 的温和改善，个别轮次和 P99 反而更慢，因此不声称普遍稳定加速或尾延迟保证。四类跟踪误差、相位、幅值、停止结果及距离调用数在输出精度内完全一致。500 Hz 原生动态参考 RMS 为 0.0933 mrad，未来路点为 0.0897 mrad；纯位置接口仍为 4.301 mrad、57.49 ms 拟合滞后，参考表达和执行方式仍是效果优化重点。

独立[Coal 距离对照](coal_distance_probe/README.md)保留持久 ComputeDistance 的源码和五轮原始输出。117,300 次逐对距离比较包含穿透，五轮最大差异均为零；该工具不包含生产 AABB 粗筛，不能把它的窄相位速度当作整个 SDK 的速度。

## 分配计数

```bash
/tmp/z1-performance-audit-build/allocation_audit \
  src/z1_ros/z1_description/urdf/z1_control.urdf
```

`allocation_audit` 用合成精确跟随反馈，测量 Controller::step 内的普通 C++ new/new[]；不计目标提交、直接 malloc、对齐分配、ROS 或官方通信库。静止目标预热后测量 200 步；连续 q/dq/ddq 输入为 50 Hz、40 ms 未来点，预热 1000 步后测量 1000 步。当前默认算法周期为 500 Hz，未按墙钟等待。

[原始前后计数](RESULTS_ALLOCATIONS_2026-10-23.txt)：静止流式参考从 5 降到 0 次/步，连续运动平均从 49.87 降到 17.95 次/步。静止的 0 次不等于整个 SDK 无分配；运动中 Coal 仍有残余分配。官方 IOUDP 每次 runFilter 内还有两处向量分配，未纳入本探针，见[固定二进制的静态反汇编](VENDOR_HOTPATH_2026-10-23.txt)。

## 墙钟调度探针与部署机复现

```bash
# 每个频率运行 10 秒，依次测 250/500/1000 Hz；只创建 Simulation。
/tmp/z1-performance-audit-build/runtime_rate_probe \
  src/z1_ros/z1_description/urdf/z1_control.urdf 10

# 可选参数：每频率时长、控制线程 CPU 编号、FIFO 优先级。
# CPU 编号应按部署机实际拓扑选择；需要相应调度权限才会应用 FIFO。
/tmp/z1-performance-audit-build/runtime_rate_probe \
  src/z1_ros/z1_description/urdf/z1_control.urdf 10 2 50
```

不带可选参数时不绑核、不设置 FIFO。探针输出实际 CPU/内核、平均频率、周期 P50/P99、绝对周期偏差 P99、最大周期、超过 1.5T 的周期数、过期槽数及线程设置结果；50 Hz 上游发送 0.01 rad/0.5 Hz 的 q/v/a 未来参考。这里的 jitter 是相邻交换开始时间之差相对标称周期的绝对偏差，不是控制器 wakeup_lateness；Runtime/ROS 另有后者统计。

[本机原始输出](RESULTS_RUNTIME_2026-10-23.txt)来自 i9-13900HX、Linux 6.8.0-138-generic，每频率 10 秒，未绑核或设置 FIFO。500 Hz 档平均 499.994 Hz，周期 P99 为 2185.498 μs、最大 2249.818 μs，missed_slots 为 0。跳槽计数与长周期不是同一指标：延迟唤醒后仍可能在下一网格前结束，此时没有跳槽，但相邻周期仍可偏长。

探针中的 Simulation 固定 dt 积分，控制参考则按实际 dt 推进；发生延迟时不能用其状态误差论证真实时间跟踪效果。它只测主机调度，不连接 MCU，也不证明机器人采样/执行频率或长期最坏时延。要判断用户的 i7-13700H，应在该电脑构建运行同一探针；再通过实际 MCU 的新反馈、交换时间、测量时龄和跟踪结果确认完整链路。4070 未参与本轮六轴控制运算。

## 定时执行与笛卡尔路径

当前新增的 `trajectory_execution_audit` 验证完整定时计划的执行保真，以及与既有动态目标追踪的区别：

```bash
cmake --build /tmp/z1-performance-audit-build --target trajectory_execution_audit -j2
/tmp/z1-performance-audit-build/trajectory_execution_audit \
  src/z1_ros/z1_description/urdf/z1_control.urdf

# 原生构建中另有路径与控制器级验证，均不创建 MCU 后端。
/tmp/z1-sdk-review-build/cartesian_path_tests \
  src/z1_ros/z1_description/urdf/z1_control.urdf
/tmp/z1-sdk-review-build/prepared_motion_tests \
  src/z1_ros/z1_description/urdf/z1_control.urdf
```

对照为固定 500 Hz 仿真，保持 2 秒后执行 4 秒完整 `q1=q0+0.02*sin(pi*t/2)^3`，首尾 v/a 为零，其余关节目标固定。定时入口一次提交 201 个间隔 20 ms 的解析 q/v/a 点；流式入口每 20 ms 提交 40 ms 未来的解析 q/v/a。控制模型、增益、限值和采样时间一致。定时入口拥有完整未来计划，流式入口仅有短时未来，因此此对照不能表示任意在线目标的通用改善倍数。

[原始输出](RESULTS_PREPARED_2026-10-23.txt)来自同一 i9-13900HX Release 环境：

| 指标 | 完整定时计划 | 40 ms 未来流式参考 |
| --- | --- | --- |
| q1 生成参考相对理想信号 RMS | 3.14e-14 rad | 4.742e-5 rad |
| q1 仿真实测相对理想信号 RMS | 0.01455 mrad | 0.04045 mrad |
| 控制 step 中位 / P99 | 7.023 / 10.942 μs | 9.361 / 12.794 μs |
| 一次完整计划准备耗时 | 239.021 μs | 不适用 |
| 执行时长 / 时间伸缩 | 4.000 s / 1 | 不适用 |

定时采样单独运行 2000 次，普通 C++ new/new[] 为 0；不包含直接 malloc 或对齐分配。上述极小参考误差用于证明软件采样没有再次整形，不是机械臂精度。仿真实测仍采用相同动力学模型，没有真实摩擦、接触或传感器噪声。单次墙钟耗时只作本机观测，不是尾延迟保证。

[笛卡尔路径输出](RESULTS_CARTESIAN_2026-10-23.txt)验证实际关节插值的 FK，而不只检查 IK 路点。10.77 mm 固定姿态直线样例准备约 0.175 ms，执行时长 1.305 s，密集采样轮廓误差最大约 0.987 μm。测试还包含平移/旋转共同进度、纯旋转、非共轴旋转转角、关节/TCP 限速与不可达路径。

12 mm + 12 mm 直角、blend 半径 4 mm 的例子，在角点附近保持约 8.62 mm/s，但全局重定时后为 3.282 s，逐点停车为 2.892 s。它验证了连续过点，未证明任务耗时必然缩短；局部转角的 jerk 峰会使当前全局伸缩较保守。下一步加工效率优化应研究沿路径局部时间参数化，而非依靠提高伺服频率。

MoveL 默认的 0.1 mm / 1 mrad 是准备期每个关节多项式段 9 点 FK 数值验收阈值，超差可加密；它不是连续时间区间的解析误差保证。实际机械臂的 TCP 轮廓及跟踪效果仍需实测。完整接口和 blend 半径定义见 [INTERFACES.md](../../INTERFACES.md)。
