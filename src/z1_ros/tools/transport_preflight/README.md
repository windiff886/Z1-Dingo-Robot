# Z1 MCU 通信接口离线预检

此工具独立链接官方 `IOUDP`，通过内存端口核实反馈返回值语义。

## 运行

需要 Linux、CMake、C++17 编译器、Eigen 和 Boost 头文件。直接使用仓库内与架构对应的官方动态库，不需要 ROS、Pinocchio 或真机。

在项目根目录执行：

```bash
cmake -S src/z1_ros/tools/transport_preflight \
  -B /tmp/z1-transport-preflight-build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build /tmp/z1-transport-preflight-build -j2
ctest --test-dir /tmp/z1-transport-preflight-build --output-on-failure -V
```

## 验证方式

可执行程序通过 ELF 符号替换，将 `UdpPort` 换成纯内存端口；`IOUDP` 初始化、编解码、`sendRecv` 和 `LowLevelState` 仍使用官方动态库实现。程序另设 `socket()` 中止检查，任何试图创建网络套接字的调用都会导致失败。不会连接、标定、使能或移动机械臂。

内存替换符号只属于本工具；其对象文件不能链接到控制进程，也不能据此认定真实固件已经验证。

覆盖：

- 模拟无夹爪设备的 V1、V2 握手及有效反馈。
- 接收超时、CRC 错误、长度错误、一般接收错误与发送错误。
- 新报文包含与上一帧相同的测量值。
- 不提供握手响应时初始化失败。

错误注入发生在 UDP 接口返回状态层，不执行真实 UDP 收发或 CRC 校验；模拟握手只用于测试，不是新增的生产协议实现。

## 与生产适配的关系

生产 [McuTransport](../../z1_control/src/mcu_transport.cpp) 使用可观测的真实 UDP 端口实现，保留官方 IOUDP 握手与报文处理，通过有效接收事件和主机接收时间判断反馈新鲜度。内存探针对象不链接进生产进程；生产适配的收发行为可通过 [transport_tests.cpp](../../z1_control/tests/transport_tests.cpp) 的真实 UDP 回环测试检查。

官方库与头文件集中于 [vendor/unitree](../../vendor/unitree)。替换官方库或目标架构后，需要重新核对探针假设及 ABI、初始化、编解码与收发行为。
