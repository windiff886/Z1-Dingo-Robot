# 控制计算分配检查

`allocation_audit` 使用合成反馈运行控制器，统计普通 C++ `new/new[]` 调用，帮助定位控制计算中的动态内存分配。

## 构建与运行

安装与 [z1_control](../../z1_control/CMakeLists.txt) 相同的原生依赖后，在仓库根目录执行：

```bash
cmake -S src/z1_ros/tools/performance_audit -B /tmp/z1-allocation-audit-build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/z1-allocation-audit-build --target allocation_audit -j2
/tmp/z1-allocation-audit-build/allocation_audit \
  src/z1_ros/z1_description/urdf/z1_control.urdf
```

依赖安装在自定义前缀时，通过 `CMAKE_PREFIX_PATH` 提供搜索路径。

## 怎样读结果

工具统计 FK、雅可比、RNEA，以及稳定和动态 JointStream 的 `Controller::step` 分配次数。反馈精确跟随生成的参考，周期使用原生 `Config` 默认值 2 ms。

计数窗口内覆盖普通 `new/new[]`，包括经这些入口发生的第三方分配；直接 `malloc` 和对齐分配未计入。目标提交在窗口外，ROS 和 MCU 通信未参与运行。因此，这些数值用于定位被测计算的分配开销。

实际控制节点的节拍与收发开销，从 `status` 的 `loop_period`、`exchange_time`、`missed_control_slots` 和反馈序号读取。

控制计算见 [SDK 算法详解](../../SDK_ALGORITHMS.md)，字段含义见 [Topic 接口说明](../../INTERFACES.md)。
