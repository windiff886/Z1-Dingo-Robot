# Z1 MCU 通信接口离线预检

`transport_contract_probe` 将 UDP 端口替换为内存实现，运行随附官方 `IOUDP` 库，检查收发返回值与反馈更新的关系。适合更换官方库或目标架构后核对接口行为。

## 构建与运行

需要 Linux x86_64 或 aarch64、CMake、C++17 编译器、Eigen 和 Boost 头文件。在仓库根目录执行：

```bash
cmake -S src/z1_ros/tools/transport_preflight \
  -B /tmp/z1-transport-preflight-build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build /tmp/z1-transport-preflight-build -j2
ctest --test-dir /tmp/z1-transport-preflight-build --output-on-failure -V
```

工具直接链接 [vendor/unitree](../../vendor/unitree) 中对应架构的动态库，运行时无需 ROS、Pinocchio 或设备连接。

## 检查内容

通过 ELF 符号替换，将 `UdpPort` 换成内存端口；初始化、编解码、`sendRecv` 和 `LowLevelState` 仍由官方库执行。程序拦截 `socket()`，尝试创建网络套接字会使测试中止。

覆盖以下情形：

- 无夹爪设备的 V1、V2 握手和有效反馈。
- 接收超时、CRC 错误状态、长度错误、一般接收错误和发送错误。
- 新报文携带与上一帧相同的测量值。
- 缺少握手响应时初始化失败。

错误在 UDP 接口返回状态层注入，CRC 错误项用于检查上层如何处理错误返回值。模拟握手也由内存端口提供。

## 与生产收发的关系

生产 [McuTransport](../../z1_control/src/mcu_transport.cpp) 保留官方 IOUDP 的握手和报文处理，使用项目的真实 UDP 端口实现，通过有效接收事件和主机接收时间判断反馈新鲜度。内存替换仅链接到本工具。

真实 UDP 收发、来源与报文校验由 [transport_tests.cpp](../../z1_control/tests/transport_tests.cpp) 的本地 MCU 模拟器和回环测试检查。实际固件的响应需要通过设备通信测试确认；这两类离线测试分别检查库接口和主机传输实现。
