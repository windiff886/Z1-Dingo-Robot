# Z1 官方 SDK：独立双臂运行

本目录直接使用宇树官方 `z1_sdk` 和 `z1_controller`，不包含原项目的自研控制内核。两台机械臂分别运行一套官方控制器，应用通过两个官方 SDK 实例独立调用。

```text
左臂应用 → 官方 SDK → 左臂 z1_ctrl → 左臂 MCU
右臂应用 → 官方 SDK → 右臂 z1_ctrl → 右臂 MCU
```

轨迹、运动学、动力学、控制状态和增益处理全部使用官方库。双臂适配只负责端口、附件模型、负载、工作目录和进程启动，不生成新的轨迹或控制力矩。

## 构建

需要 Linux x86_64 / aarch64、CMake ≥ 3.16、C++ 编译器、Eigen3、Boost 头文件、Python3 和 PyYAML。不需要 ROS、Pinocchio 或 Ruckig。

在仓库根目录执行：

```bash
cmake -S src/z1_sdk -B /tmp/z1-build \
  -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/z1-build -j2
```

生成 `bin/z1_ctrl`、`bin/z1_sdk_client` 和 `bin/z1_official.py`。默认也编译四个上游示例，位于 `examples/`；这些示例运行时会发送运动命令。可用 `-DZ1_BUILD_EXAMPLES=OFF` 关闭示例构建。

`CATKIN_IGNORE` 使 ROS 工作区跳过本目录及上游仿真包，机械臂 SDK 单独用上述命令构建。

## 配置左右臂

编辑 [dual_arm/config/arms.yaml](dual_arm/config/arms.yaml)：

| 参数 | 左臂 | 右臂 |
| --- | --- | --- |
| MCU IP | `192.168.123.110` | `192.168.123.111` |
| PC 连接 MCU 的本地端口 | 8881 | 8882 |
| 官方控制器 SDK 监听端口 | 8071 | 8073 |
| SDK 客户端本地端口 | 8072 | 8074 |
| `has_gripper` | `true` | `false` |
| 额外负载 `load_kg` | 0.0 kg | 0.0 kg |

MCU 目标端口由官方库使用 8880。`has_gripper` 选择 SDK 模型；官方控制器仍按设备握手检测附件，两者应与实物一致。夹爪模型采用上游提供的偏置、质量、质心与惯量；额外负载同时写入控制器配置和 SDK 模型。

仅检查配置，不启动控制器或访问设备：

```bash
/usr/bin/python3 /tmp/z1-build/bin/z1_official.py check
```

## 启动两套控制器

先停止其他控制同一机械臂的程序，再执行：

```bash
/usr/bin/python3 /tmp/z1-build/bin/z1_official.py controllers
```

也可以加 `--arm left` 或 `--arm right` 单独启动。一臂进程异常退出时，启动器会报告错误，另一臂继续运行，不自动重启或联动停止。Ctrl+C 会向仍在运行的本次子进程转发 SIGINT，并回收它们。

运行文件默认保存在 `${XDG_STATE_HOME:-~/.local/state}/z1_sdk/left` 和 `right`：

```text
left/                         right/
├── config/                   ├── config/
│   ├── config.xml            │   ├── config.xml
│   └── savedArmStates.csv    │   └── savedArmStates.csv
└── run/                      └── run/
```

启动器从官方模板生成各自的 `config.xml`，并为首次启动复制官方标签。再次启动不会覆盖已保存的标签或示教轨迹。`--runtime-dir /绝对路径` 可指定运行目录。

官方模板中的碰撞配置、夹爪最大力矩等参数沿用 [controller/config/config.xml](controller/config/config.xml)；默认 `collision/open=N`。本项目没有增加双臂互碰检查。更改模板后需重新启动控制器。

## 分别调用左右臂

在另外两个终端分别打开交互客户端：

```bash
/usr/bin/python3 /tmp/z1-build/bin/z1_official.py client --arm left
/usr/bin/python3 /tmp/z1-build/bin/z1_official.py client --arm right
```

客户端连接后不会自动回 Home 或执行预设动作。每臂同时只运行一个 SDK 客户端；客户端端口固定，不要与同臂 Python/C++ 应用同时占用。

客户端输入：

```text
state
help
```

下面是会产生实际动作或状态变更的官方调用：

```text
home
label_run forward
movej 0 0 0 0.35 0 0.3 0.2
movel 0 0 0 0.35 0 0.3 0.05
passive
```

位姿参数顺序为 `roll pitch yaw x y z`，角度单位 rad，位置单位 m。MoveJ 速度为 rad/s，MoveL/MoveC 为 m/s。`movej` 第一个六维参数是末端位姿，不是六个关节角。位姿使用官方控制器的基座与末端定义，不自动转换整机坐标或显示模型的 `tool`。

`passive` 请求官方被动状态，不是主动姿态保持或平滑停止；`quit` 只关闭 SDK 通信，不发送停止或回零命令。设备失联响应沿用官方实现。`calibrate` 会改变零位定义，只应在已按官方要求摆好机械零位时显式调用。

也可以调用一次后退出，例如读取官方最近反馈：

```bash
/usr/bin/python3 /tmp/z1-build/bin/z1_official.py client --arm left -- state
```

CLI 直接报告官方调用的返回结果，不把它重新定义为独立的实测到位判据。原有 ROS Topic、Disabled/Holding 状态机、摩擦辨识和 RViz 轨迹预览入口已移除。

## C++ 应用

使用 [dual_arm/include/z1_official/sdk.hpp](dual_arm/include/z1_official/sdk.hpp) 创建指定端口的组件，再使用原生 `UNITREE_ARM::unitreeArm`：

```cpp
#include <z1_official/sdk.hpp>

// 右臂：无夹爪，额外负载为 0 kg。一个应用进程负责一臂。
UNITREE_ARM::unitreeArm arm(z1_official::components(8073, 8074, false, 0.0));
arm.sendRecvThread->start();
// 在这里调用官方接口；不自动执行运动。
arm.sendRecvThread->shutdown();
```

`unitreeArm` 拥有传入的组件，不要再次手动释放。构造和方法调用的行为来自官方库，完整接口见 [unitreeArm.h](include/unitree_arm_sdk/control/unitreeArm.h)。

安装后，外部 CMake 应用可用：

```cmake
find_package(Z1OfficialSDK REQUIRED CONFIG)
target_link_libraries(my_app PRIVATE z1_official::sdk)
```

## Python 应用

随上游附带的模块只适用于 CPython 3.8 / x86_64。推荐为实际使用的 Python 重新编译，需要该 Python 的开发头文件和 pybind11：

```bash
cmake -S src/z1_sdk -B /tmp/z1-build \
  -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DZ1_BUILD_PYTHON=ON
cmake --build /tmp/z1-build -j2
```

模块生成在 `/tmp/z1-build/dual_arm`，不会覆盖上游附带的二进制。绑定保留官方方法，只为 `ArmInterface` 构造函数增加端口和额外负载参数：

```python
import unitree_arm_interface as sdk

arm = sdk.ArmInterface(hasGripper=False, controllerPort=8073,
                       sdkPort=8074, load=0.0)
arm.loopOn()
# 调用官方接口。
arm.loopOff()
```

运行前将 `/tmp/z1-build/dual_arm` 加入 `PYTHONPATH`。未填写新增参数时仍使用官方左臂默认端口。上游 `.pyi` 与实际枚举/返回类型的差异仍见 [项目详解](PROJECT_GUIDE.zh-CN.md)。

## 测试与安装

```bash
ctest --test-dir /tmp/z1-build --output-on-failure
cmake --install /tmp/z1-build --prefix /tmp/z1-install
/usr/bin/python3 /tmp/z1-install/bin/z1_official.py check
```

测试覆盖双臂配置、运行目录与示教文件隔离、子进程清理、两套真实 SDK 的本地 UDP 通信，以及两套真实官方控制器与 SDK 的连通。最后一项只替换 MCU IO 边界，不连接设备，也不模拟机械动力学。测试专用替身不安装进运行系统。

2026-11-02 已在 Linux x86_64 / GCC 11.4 / Python 3.10 上完成上述离线检查，并验证安装目录迁移后的 C++ `find_package`、动态库加载和 Python 模型调用。

实机握手、运动效果、夹爪检测、失联响应和退出行为需要在设备上验收。本次软件迁移不自动启动实机测试。

## 目录与上游版本

- `include/`、`lib/`、`examples/`、`examples_py/`：完整官方 SDK 文件。
- `controller/`：完整官方控制器文件，包括动态库、配置、仿真适配及工具。
- `dual_arm/`：双臂初始化、启动与测试；不实现控制算法。
- [UPSTREAM.md](UPSTREAM.md)：固定提交与构建期适配范围。
- [PROJECT_GUIDE.zh-CN.md](PROJECT_GUIDE.zh-CN.md)：官方 SDK 的详细阅读指南。

官方资料：[Z1 文档](https://support.unitree.com/home/zh/Z1_developer)、[z1_sdk](https://github.com/unitreerobotics/z1_sdk)、[z1_controller](https://github.com/unitreerobotics/z1_controller)。
