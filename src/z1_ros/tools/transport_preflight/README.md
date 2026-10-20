# Z1 MCU 通信接口离线预检

此工具验证官方 `IOUDP` 能否在不创建 ROS、旧控制状态机或机械臂模型的情况下链接和运行，并核实其反馈返回值语义。它不是控制节点或可部署的通信适配器。

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

## 当前结果

2026-10-11，在 Linux x86_64、GCC 11.4、CMake 3.22 环境编译和 CTest 通过。

被测库：`vendor/unitree/lib/libUnitreeArm_x86_64.so`。

SHA-256：`47861f6155c0ade03de6c19a97d3f66923921b3134a59f7b84692e475c240348`。

V1/V2 均复现以下行为：接收失败后 `sendRecv` 仍返回 `true`，原始 q/dq/tau 保留旧值；发送错误也未通过此返回值报告。新报文可能具有完全相同的 q/dq/tau，因此比较数组变化同样不能判定是否收到新数据。

此探针断言的是当前官方库的实际契约，不是理想契约。更换库后若结果变化，需重新审查，而不能把“失败不再返回 true”直接视为新库缺陷。aarch64 配置入口存在，但本次未在该架构运行。

## 与新适配的关系

生产 [McuTransport](../../z1_control/src/mcu_transport.cpp) 使用另一份可观测的真实 UDP 端口实现，保留官方 IOUDP 握手与报文处理。内存探针对象没有链接进生产进程；探针记录的是旧返回值缺口，新适配的有效接收契约由 `z1_control/tests/transport_tests.cpp` 的真实 UDP 回环验证。

官方库与头文件集中于 `vendor/unitree`，版本按 SHA-256 锁定；更换版本需重做探针和回环测试。真实固件和力矩控制能力仍须现场验证，UDP 后端默认禁止未验收接管。见 [实施记录](../../IMPLEMENTATION_REPORT.md)。
