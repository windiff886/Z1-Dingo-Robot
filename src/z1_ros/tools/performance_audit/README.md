# 控制计算检查与历史实验

此目录提供不连接设备的计算分配检查、几何距离对照及实验输出。

## 当前可运行的分配检查

`allocation_audit` 用合成的精确跟随反馈检查 `Controller::step` 内的普通 C++ `new/new[]`，不会创建 MCU 通信或网络套接字，不需要 ROS。

```bash
cmake -S src/z1_ros/tools/performance_audit -B /tmp/z1-allocation-audit-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/path/to/native/dependencies
cmake --build /tmp/z1-allocation-audit-build --target allocation_audit -j2
/tmp/z1-allocation-audit-build/allocation_audit \
  src/z1_ros/z1_description/urdf/z1_control.urdf
```

依赖与控制内核相同，依赖前缀按实际环境替换。计数不包含目标提交、直接 `malloc`、对齐分配、ROS 或官方通信库，因此不能把该结果解释为整个真机程序无分配。

几何算法的独立对照见 [Coal 距离检查](coal_distance_probe/README.md)。控制运行时的周期统计由真实节点的 `status` 提供，包括新反馈接收序号、测量时龄、交换耗时、唤醒迟到、漏过时隙及调度设置结果。真机周期、MCU 采样/执行节奏及跟踪效果必须在实际设备上测量。

## 历史实验输出

以下文件记录 2026-10-23 的实验环境和当时结果。部分实验使用同模型动力学计算，没有真实摩擦、传感器噪声、接触或网络；这些数据用于设计分析，不能代表真机精度、频率和实时保证。

| 原始记录 | 历史用途 |
| --- | --- |
| [最初基线](RESULTS_2026-10-23.txt)与[第一轮改动后](RESULTS_AFTER_2026-10-23.txt) | 原参考生成和跟踪对照 |
| [500 Hz 改动前](RESULTS_500_BEFORE_2026-10-23.txt)、[改动后](RESULTS_500_AFTER_2026-10-23.txt)、[成对复测](RESULTS_500_PAIRED_2026-10-23.txt) | 固定输入的计算耗时与历史跟踪对照 |
| [分配计数](RESULTS_ALLOCATIONS_2026-10-23.txt) | 合成反馈下的控制计算分配 |
| [调度记录](RESULTS_RUNTIME_2026-10-23.txt) | 原本机调度探针输出，不是 MCU 收发结果 |
| [定时轨迹对照](RESULTS_PREPARED_2026-10-23.txt) | 完整时间计划与短时未来参考的历史对照 |
| [笛卡尔路径检查](RESULTS_CARTESIAN_2026-10-23.txt) | 模型 FK 路径拟合及运动约束检查 |
| [官方库静态反汇编](VENDOR_HOTPATH_2026-10-23.txt) | 固定二进制内部调用及分配证据 |

设计分析见 [SDK 技术审查](../../SDK_TECHNICAL_REVIEW.md)，当前接口见 [INTERFACES.md](../../INTERFACES.md)。
