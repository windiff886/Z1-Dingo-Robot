# Z1 MCU 通信接口离线预检

`transport_contract_probe` 会把 UDP 端口替换成内存实现，然后运行随附的官方 `IOUDP` 库，检查收发返回值与反馈更新之间的关系。更换官方库或目标架构之后，可以用它核对接口行为。

## 构建与运行

需要 Linux x86_64 或 aarch64、CMake、C++17 编译器、Eigen 和 Boost 头文件。在仓库根目录执行：

```bash
cmake -S src/z1_ros/tools/transport_preflight \
  -B /tmp/z1-transport-preflight-build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build /tmp/z1-transport-preflight-build -j2
ctest --test-dir /tmp/z1-transport-preflight-build --output-on-failure -V
```

工具会直接链接 [vendor/unitree](../../vendor/unitree) 中对应架构的动态库，运行时不需要 ROS、Pinocchio，也不需要连接设备。

## 检查内容

程序通过替换 ELF 符号，把 `UdpPort` 换成内存端口；初始化、编解码、`sendRecv` 和 `LowLevelState` 仍然由官方库执行。程序会拦截 `socket()`，一旦尝试创建网络套接字，测试就会中止。

覆盖的情形包括：

- 无夹爪设备的 V1、V2 握手和有效反馈。
- 接收超时、CRC 错误状态、长度错误、一般接收错误和发送错误。
- 新报文携带与上一帧相同的测量值。
- 缺少握手响应时初始化失败。

错误是在 UDP 接口返回状态这一层注入的；CRC 错误项用来检查上层如何处理错误返回值。模拟握手也由内存端口提供。

## 与生产收发的关系

生产代码中的 [McuTransport](../../z1_control/src/mcu_transport.cpp) 保留官方 IOUDP 的握手和报文处理，使用项目的真实 UDP 端口实现，并通过有效接收事件和主机接收时间判断反馈是否新鲜。内存替换只链接到这个工具里。

真实的 UDP 收发、来源校验和报文校验由 [transport_tests.cpp](../../z1_control/tests/transport_tests.cpp) 中的本地 MCU 模拟器和回环测试检查。实际固件的响应还需要通过设备通信测试确认；这两类离线测试分别检查库接口和主机的传输实现。
