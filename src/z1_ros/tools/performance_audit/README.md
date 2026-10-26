# 控制计算检查

此目录提供不连接设备的计算分配检查和几何距离对照工具。

## 计算分配检查

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

控制算法见 [SDK 算法详解](../../SDK_ALGORITHMS.md)，接口见 [INTERFACES.md](../../INTERFACES.md)。
