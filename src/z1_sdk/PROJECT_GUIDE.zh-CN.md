# Z1 SDK 项目详解

当你调用 `arm.MoveJ(...)` 时，SDK 并不是把这几个参数直接发送给机械臂电机。它先把目标封装成带状态机指令的 UDP 消息，交给另一端的 `z1_controller`；控制器执行运动，并持续返回关节和末端状态。理解这条链路，才能正确选择接口、安排控制线程，以及判断一条命令是否真正执行完成。

本文解读宇树官方 `z1_sdk` 的头文件、示例和预编译库。本项目现已在 `controller/` 导入配套官方控制器，`dual_arm/` 只提供双实例初始化与启动。适合首次阅读项目、开发 C++/Python 应用，或准备接入自定义控制算法的读者。

**推荐路线：先编译并运行离线模型示例，再理解高层命令，最后研究 LOWCMD。** 离线计算不需要连接机械臂；LOWCMD 则把参考轨迹、前馈力矩和周期控制的责任交给应用程序。

## 目录

1. [项目定位与源码范围](#1-项目定位与源码范围)
2. [目录与推荐阅读顺序](#2-目录与推荐阅读顺序)
3. [一次命令经过哪些层](#3-一次命令经过哪些层)
4. [先分清关节角、位姿和空间速度](#4-先分清关节角位姿和空间速度)
5. [如何构建和链接](#5-如何构建和链接)
6. [核心类与对象关系](#6-核心类与对象关系)
7. [高层运动与状态机](#7-高层运动与状态机)
8. [连续跟踪与 LOWCMD](#8-连续跟踪与-lowcmd)
9. [UDP 消息与反馈字段](#9-udp-消息与反馈字段)
10. [运动学、动力学和 QP](#10-运动学动力学和-qp)
11. [控制周期、线程与共享数据](#11-控制周期线程与共享数据)
12. [逐个理解 C++ 示例](#12-逐个理解-c-示例)
13. [Python 绑定与 HTTP 示例](#13-python-绑定与-http-示例)
14. [双机械臂通信如何配置](#14-双机械臂通信如何配置)
15. [与本项目双臂集成的关系](#15-与本项目双臂集成的关系)
16. [当前代码中值得优先处理的问题](#16-当前代码中值得优先处理的问题)
17. [排查顺序与离线核验记录](#17-排查顺序与离线核验记录)

## 1. 项目定位与源码范围

### 1.1 它提供什么

`z1_sdk` 是面向 Unitree Z1 六轴机械臂的应用开发接口，提供：

- 关节路径运动 `MoveJ`、末端直线运动 `MoveL`、末端圆弧运动 `MoveC`。
- 关节连续跟踪、笛卡尔连续跟踪，以及夹爪控制。
- LOWCMD 模式下的位置、速度、力矩和增益指令。
- 关节与末端反馈、设备错误和连接状态。
- Z1 的正运动学、逆运动学、雅可比、逆动力学和速度求解接口。
- C++ 接口、pybind11 Python 绑定，以及一个 HTTP 包装示例。

### 1.2 这是“接口源码 + 预编译核心库”

当前检出的提交为 `f1af2b42f2a39a5010946e049ad3fe324e2e6f06`，提交日期为 `2024-10-29`，提交说明为 `Update ArmModel.h`。

本目录包含公开头文件、示例和少量内联实现，但核心类的大部分实现位于：

```text
lib/libZ1_SDK_x86_64.so
lib/libZ1_SDK_aarch64.so
```

例如，`unitreeArm::sendRecv()`、`MoveJ()`、`CtrlComponents` 构造函数、Z1 模型参数和具体 IK/QP 算法，均没有对应的实现 `.cpp` 随当前项目提供。因此：

- **可以直接核对**：接口签名、消息布局、绑定代码、示例调用顺序、定时器和内联工具函数。
- **按接口注释理解**：高层命令的等待规则、模型算法类型，以及 `sendRecv()` 的组包职责。
- **需要控制器源码或运行证据确认**：完整状态迁移图、轨迹插值细节、超时恢复策略、QP 的具体目标与约束、核心对象的释放关系。

## 2. 目录与推荐阅读顺序

```text
z1_sdk/
├── CMakeLists.txt                     # 本项目原生构建入口
├── README.md                         # 官方文档入口
├── LICENSE
├── include/unitree_arm_sdk/
│   ├── control/
│   │   ├── unitreeArm.h               # 面向应用的总入口
│   │   └── ctrlComponents.h           # 命令、反馈、模型、UDP 的组合
│   ├── message/
│   │   ├── arm_common.h               # 状态枚举与线上的消息结构
│   │   ├── LowlevelCmd.h              # 应用侧命令容器
│   │   ├── LowlevelState.h            # 应用侧反馈容器
│   │   └── udp.h                      # UDPPort 与 IOPort
│   ├── model/ArmModel.h               # ArmModel、Z1Model
│   ├── math/                         # Eigen 类型、数学与数据转换工具
│   ├── utilities/loop.h               # Timer、LoopFunc 的内联实现
│   └── thirdparty/
│       ├── robotics.h                 # 刚体变换、运动学与动力学函数声明
│       └── quadProgpp/                # QP 接口及数组工具
├── controller/                        # 完整官方控制器与配套动态库
├── dual_arm/                          # 双臂初始化、启动、测试
├── examples/                          # 4 个 C++ 示例
├── examples_py/                       # 绑定源码、类型提示与 Python 示例
└── lib/                               # 两种 CPU 架构的库及 CPython 3.8 模块
```

建议按下面的顺序阅读，而不是一开始就钻进数学工具：

1. [`examples/highcmd_basic.cpp`](examples/highcmd_basic.cpp)：先看应用怎样使用 SDK。
2. [`control/unitreeArm.h`](include/unitree_arm_sdk/control/unitreeArm.h)：对照接口与注释。
3. [`control/ctrlComponents.h`](include/unitree_arm_sdk/control/ctrlComponents.h)：理解对象组合。
4. [`message/arm_common.h`](include/unitree_arm_sdk/message/arm_common.h)：理解命令实际如何表示。
5. [`examples/lowcmd_development.cpp`](examples/lowcmd_development.cpp)：看自定义周期控制的完整调用顺序。
6. [`model/ArmModel.h`](include/unitree_arm_sdk/model/ArmModel.h)：再理解算法入口。
7. [`examples_py/arm_python_interface.cpp`](examples_py/arm_python_interface.cpp)：开发 Python 时，以实际绑定为准。

## 3. 一次命令经过哪些层

### 3.1 运行链路

```text
C++ 应用 / Python 应用 / HTTP 包装
                  │
                  ▼
        unitreeArm / ArmInterface
        高层运动、状态切换、跟踪参数
                  │
                  ▼
           CtrlComponents
   LowlevelCmd / LowlevelState / ArmModel
          SendCmd / RecvState
                  │
                  ▼
               UDPPort
                  │ SDK ↔ 控制器的 UDP 协议
                  ▼
          z1_controller / z1_ctrl
        状态机、运动执行、硬件或仿真接口
                  │
                  ▼
             机械臂或仿真
```

这条链路的直接依据是 `sendRecv()` 的接口注释和 `lowcmd_multirobots.cpp`：后者明确把两个 SDK 实例连接到本机的两个 `z1_ctrl` 实例，其中一个控制实机，另一个连接 ROS 仿真。

上游独立 `z1_sdk` 不包含控制器。本项目现已把官方控制器完整导入 [controller/](controller)，并构建两臂各自使用的 `z1_ctrl`；核心实现仍由官方动态库提供。

### 3.2 命令与反馈走相反方向

发送时，可以按下面的逻辑理解：

```text
应用更新目标/状态请求
  → unitreeArm 整理命令参数
  → CtrlComponents 将命令送入 SendCmd
  → UDPPort 发送
```

接收时：

```text
UDPPort 接收
  → RecvState
  → LowlevelState
  → 应用读取 lowstate 或控制器状态
```

具体字段何时复制由二进制库实现。公开接口明确指出：`unitreeArm::sendRecv()` 会调用 `CtrlComponents::sendRecv()`，并自动把自身的控制参数设置到 `lowcmd`。

**设置变量不是发送命令。** 只有通信循环继续执行，控制器才能收到更新，应用才能获得新反馈。

## 4. 先分清关节角、位姿和空间速度

### 4.1 三种六维向量不是一回事

| 数据 | 顺序 | 单位与含义 |
| --- | --- | --- |
| 关节位置 `q` | `J1, J2, J3, J4, J5, J6` | rad |
| 关节速度 `qd` / `dq` | 同上 | rad/s；两种拼写表示同类数据 |
| 关节加速度 `qdd` / `ddq` | 同上 | rad/s² |
| 关节力矩 `tau` | 同上 | 关节力矩，通常按 N·m 使用 |
| 末端 `posture` | `roll, pitch, yaw, x, y, z` | 前三项为 rad，后三项为 m |
| 空间速度 `twist` | `ωx, ωy, ωz, vx, vy, vz` | 前三项为角速度，后三项为线速度 |
| 遥操作 `directions` | 6 个运动分量，再加夹爪 | 每个分量的接口范围为 `[-1, 1]` |

最容易出错的是：**`MoveJ(Vec6 posture, ...)` 的第一个参数仍然是末端位姿，不是六个关节角。** 名字中的 J 表示按关节路径运动。若已有六轴目标，可用 JOINTCTRL/LOWCMD 下发，或先做 FK 得到 `posture`；后者经过 IK 后不保证返回原来的关节分支。

`twist` 的角速度也不能直接当作 RPY 的逐项导数。旋转参数的变化率与角速度之间有姿态相关的映射。

### 4.2 坐标系需要与所用接口匹配

- `CalcJacobian()` 注释声明返回 **空间雅可比**。
- `startTrack(CARTESIAN)` 注释把跟踪速度描述为基于 **物体坐标系** 的命令。
- `Z1Model` 构造参数中的 `endPosLocal` 是末端局部偏置；改变它会影响末端定义。
- 高层位姿应按控制器所用机械臂基座和末端模型解释，不能直接填入整机 `base_link` 或相机坐标系下的数值。

因此，`CalcJacobian(q) * qd` 与 CARTESIAN 跟踪指令不能仅凭“都是六维向量”就直接互换。需要先统一空间/物体表达和末端参考点，再用伴随变换等方式转换。

当前公开头文件没有完整说明高层位姿使用的安装变换、RPY 旋转约定及控制器工具配置。对接外部规划器时，推荐用 `postureToHomo()` / `homoToPosture()` 做转换，并用已知姿态核对控制器与规划器的坐标定义。

### 4.3 六轴与夹爪是两组命令

`getQ()`、`getQd()` 返回六轴 `Vec6`；夹爪通过 `getGripperQ()` 等接口单独访问。底层协议预留 7 个关节槽位，通常按六轴加夹爪理解。

高层运动接口注释给出的夹爪目标范围为 `[-π/2, 0] rad`。这不是夹爪开口宽度，也不是百分比；当前集成不再混入旧自研接口的范围约定。

## 5. 如何构建和链接

本项目现在统一构建官方控制器、SDK 示例、双臂调用入口和可选 Python 绑定，不需要 ROS。操作步骤见 [README](README.md)。

```bash
cmake -S src/z1_sdk -B /tmp/z1-build \
  -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/z1-build -j2
```

根目录的 [CMakeLists.txt](CMakeLists.txt) 与 [dual_arm/CMakeLists.txt](dual_arm/CMakeLists.txt) 负责当前集成。上游原始构建文件保存在 [cmake/upstream-sdk-CMakeLists.txt](cmake/upstream-sdk-CMakeLists.txt)：它按 `CMAKE_SYSTEM_PROCESSOR` 选择 `libZ1_SDK_x86_64.so` 或 `libZ1_SDK_aarch64.so`，编译四个示例，并在找到 pybind11 时构建 Python 模块。

当前构建保留四个原始示例，但把 Python 模块输出到构建目录，补齐 Eigen 包含路径，并支持安装和 `find_package(Z1OfficialSDK)`。示例可执行文件会发送实际控制命令；离线计算请用第 10.5 节的纯模型示例。

随上游提供的 Python `.so` 面向 CPython 3.8 / x86_64。其他 ABI 或架构需要用 `-DZ1_BUILD_PYTHON=ON` 重新构建。Python 查找扩展模块的路径与动态链接器查找 SDK 库的路径是两回事：前者使用 `PYTHONPATH`，后者由构建/安装 RPATH 或 `LD_LIBRARY_PATH` 处理。

安装后的 SDK 库和 Python 模块位于同一个 `lib` 目录，模块使用 `$ORIGIN` 找库。部署时不要依赖上游预编译 Python 模块中残留的开发机绝对 RUNPATH。

## 6. 核心类与对象关系

### 6.1 `unitreeArm`：应用的控制门面

它把运动命令、跟踪变量、通信线程和底层组件集中到同一个对象中：

```text
unitreeArm
├── q / qd / tau                  六轴命令参数
├── twist                         笛卡尔跟踪速度
├── gripperQ / gripperW / gripperTau
├── sendRecvThread                周期通信线程指针
├── lowcmd ───────────────────┐
├── lowstate ──────────────┐  │
└── _ctrlComp             │  │
    ├── lowstate ◀────────┘  │    与外层指针指向同一对象
    ├── lowcmd ◀─────────────┘
    ├── dt                       控制周期
    ├── sendCmd / recvState       协议消息
    ├── armModel                  运动学、动力学模型
    └── udp                       UDPPort
```

它有两种构造入口：

```cpp
unitreeArm(bool hasUnitreeGripper);
unitreeArm(CtrlComponents* ctrlComp);
```

单臂示例直接使用 `unitreeArm(true)`；多实例示例则自行准备 `CtrlComponents`，指定 UDP 和模型后传入。

构造函数注释对“默认配置”的描述不够清晰，构造实现也未随源码提供。自定义网络和工具参数时，推荐参考多实例示例明确配置，而不是依赖看不见的默认地址或工具模型。

### 6.2 `CtrlComponents`：依赖组合，不是另一套控制算法

[`ctrlComponents.h`](include/unitree_arm_sdk/control/ctrlComponents.h) 中主要是公开成员和 `sendRecv()`：

- `lowcmd` / `lowstate`：应用易于读写的命令与反馈。
- `sendCmd` / `recvState`：线上协议对象。
- `armModel`：运动学与动力学计算。
- `udp`：实际收发。
- `dt`：周期，头文件标注默认值为 `0.002 s`。

外层 `arm.lowstate` 与 `arm._ctrlComp->lowstate` 是别名关系，不是两份独立反馈缓存。

这些类型暴露了多个原始指针，析构和资源所有权的实现位于库中。扩展封装时需确认传入组件的生命周期和释放约定，避免同时由调用方和 SDK 释放同一对象。

### 6.3 `LowlevelCmd`：命令容器

其主要成员为 `q`、`dq`、`tau`、`kp`、`kd`、`posture`，以及源码中拼写为 **`twsit`** 的六维成员。

常用操作包括：

- `setQ()`、`setQd()`、`setTau()`：设置六轴位置、速度和力矩。
- `setControlGain()`：设置 LOWCMD 增益。
- `setGripperQ()`、`setGripperQd()`、`setGripperTau()`：设置夹爪。
- `setPassive()`：按注释清零速度、前馈力矩和 Kp，并把六轴 Kd 设为 10。

头文件注明无参 `setControlGain()` 对应六轴：

```text
kp = [20, 30, 30, 20, 15, 10]
kd = [2000, 2000, 2000, 2000, 2000, 2000]
```

夹爪无参增益为 `kp=20, kd=2000`。这些是本 SDK 注释中的值；增益的设备侧缩放和实际控制效果要结合所配控制器确认，不能直接跨版本替代其他接口中的增益数值。

另外，`lowcmd->setPassive()` 是修改命令内容，`arm.setFsm(PASSIVE)` 是请求控制器状态切换，两者作用层次不同。

### 6.4 `LowlevelState`：反馈容器

提供六轴位置、速度、加速度、力矩和末端位姿，以及温度、错误和电机连接信息。常用接口为：

```cpp
arm.lowstate->getQ();
arm.lowstate->getQd();
arm.lowstate->getQdd();
arm.lowstate->getTau();
arm.lowstate->endPosture;
```

这些值来自最近的接收处理。公开容器没有接收时间戳或包序号，应用不能仅凭“读到了一个数值”就判断它是新反馈。

## 7. 高层运动与状态机

### 7.1 `MoveJ`、`MoveL`、`MoveC` 的区别

| 接口 | 目标 | 路径含义 | 速度单位 |
| --- | --- | --- | --- |
| `MoveJ(posture, maxSpeed)` | 末端位姿 | 通过关节路径到达该位姿；末端路径不要求为直线 | rad/s，注释范围 `[0, π]` |
| `MoveL(posture, maxSpeed)` | 末端位姿 | 末端直线路径 | m/s |
| `MoveC(middlePosture, endPosture, maxSpeed)` | 中间与终点位姿 | 用中间位姿与终点定义圆弧运动 | m/s |

三个接口都有同时指定 `gripperPos` 的重载。它们返回 `bool`；接口注释主要将该值解释为目标是否具有 IK 解。不能把返回 `true` 等同于独立验证过“实测到位、没有跟踪误差、路径无碰撞”。

位姿如何插值、圆弧姿态如何变化，以及速度上限如何作用到整条轨迹，都需要查看对应控制器实现；当前示例没有给出这些算法。

### 7.2 状态枚举是协议的一部分

[`arm_common.h`](include/unitree_arm_sdk/message/arm_common.h) 中未显式赋值的枚举从 0 顺序递增：

| 值 | C++ 枚举 | 用途或可见入口 |
| --- | --- | --- |
| 0 | `INVALID` | 无效状态 |
| 1 | `PASSIVE` | 被动状态 |
| 2 | `JOINTCTRL` | 关节控制、轨迹完成后的返回状态 |
| 3 | `CARTESIAN` | 笛卡尔控制 |
| 4 | `MOVEJ` | 关节路径运动 |
| 5 | `MOVEL` | 直线运动 |
| 6 | `MOVEC` | 圆弧运动 |
| 7 | `TRAJECTORY` | 轨迹相关状态 |
| 8 | `TOSTATE` | 运行已保存位姿，参见 `labelRun()` |
| 9 | `SAVESTATE` | 保存位姿，参见 `labelSave()` |
| 10 | `TEACH` | 示教相关状态 |
| 11 | `TEACHREPEAT` | 示教复现相关状态 |
| 12 | `CALIBRATION` | 标定 |
| 13 | `SETTRAJ` | 轨迹相关内部状态 |
| 14 | `BACKTOSTART` | 回初始位姿 |
| 15 | `NEXT` | 当前头文件未说明应用语义 |
| 16 | `LOWCMD` | 低层命令 |

枚举只列出状态，**不等于允许从任意状态跳到其他状态**。`setFsm()` 注释明确举例：进入 LOWCMD 需要先处于 PASSIVE；该接口会等待切换，并返回是否成功。

### 7.3 等待执行结束不等于停止通信

`unitreeArm` 的 `_isWait` 初始值为 `true`。按 `setWait()` 注释，`MoveJ/MoveL/MoveC` 会发出轨迹请求，然后等待控制器执行完成；`backToStart()`、`labelRun()`、`teachRepeat()` 也受这个设置影响。

等待期间仍需后台 `sendRecvThread` 持续通信。线程没有启动或收不到反馈时，依赖状态回执的调用可能无法按预期完成；当前头文件没有暴露这些等待操作的统一超时参数。

调用 `setWait(false)` 后，应用要自行判断完成。注释给出两个思路：

1. 观察 `recvState.state`；轨迹执行结束后，控制器会自动回到 JOINTCTRL。
2. 比较反馈末端位姿与目标。

实际应用应先确认请求确实开始执行，再等待返回状态，并结合新鲜反馈、关节速度和目标误差判定到位。仅检查“当前状态不是 MOVEJ”会把尚未开始或执行失败误判为完成。

位姿误差也不宜直接长期使用六维向量的 `.norm()`：它混合 rad 和 m，并且 RPY 有跨周期和奇异性问题。更清晰的判据是分别限制位置误差和旋转误差。

### 7.4 回初始位姿、标签与标定

- `backToStart()`：请求回初始位姿，注释注明到达后进入 JOINTCTRL。
- `labelRun(label)`：运行控制器已保存的位姿。
- `labelSave(label)`：把当前位姿保存到控制器的 `config/saveArmStates.csv`。
- `teach()` / `teachRepeat()`：示教与复现相关接口；头文件在两者附近存在注释错位，需结合控制器流程确认录制与回放行为。
- `calibration()`：把当前物理位置作为标定参考，改变零位含义；它不是“运动到零位”。

标签存储在 `char name[10]` 中，HTTP 客户端示例要求 `len(label) < 10`。应用宜采用最多 **9 个 ASCII 字节**的标签，为字符串结束符留空间；中文字符不能按 Python 字符数直接估算字节长度。

`highcmd_basic.cpp` 中使用的 `"forward"` 必须已经存在于控制器配置。SDK 目录本身不包含这些保存文件。

## 8. 连续跟踪与 LOWCMD

### 8.1 三种模式的责任划分

| 方式 | 应用持续提供什么 | 适合什么任务 |
| --- | --- | --- |
| 高层 `Move*` | 一次位姿与速度请求 | 点到点、直线、圆弧等离散动作 |
| `JOINTCTRL` / `CARTESIAN` 跟踪 | 周期更新关节参考或空间速度 | 遥操作、外部参考跟踪 |
| `LOWCMD` | 六轴 q/dq/tau、增益和夹爪命令 | 自定义轨迹与控制算法 |

### 8.2 `startTrack(JOINTCTRL)`

接口注释说明，首次进入跟踪时会从反馈初始化 `q`、`qd` 及夹爪位置/速度，随后应用可更新这些参数。

`jointCtrlCmd(directions, jointSpeed)` 的设计意图可以写为：

```text
qd = directions × jointSpeed
q  = q + qd × dt
```

其中 `directions` 有 7 项，最后一项控制夹爪。它适合键盘或手柄输入，但也意味着调用频率、`dt` 和持续时间必须一致，否则累计目标会与预期不同。

当应用直接给定轨迹时，应同时提供一致的 `q` 与 `qd`，而不是让控制器用与位置轨迹相矛盾的速度参考。

### 8.3 `startTrack(CARTESIAN)`

首次进入时，`twist` 被清零。`cartesianCtrlCmd()` 的输入顺序为：

```text
roll方向, pitch方向, yaw方向, x方向, y方向, z方向, 夹爪方向
```

头文件给出的速度参数范围为：

- `oriSpeed`：`[0, 0.6]`，对应角运动速度。
- `posSpeed`：`[0, 0.3]`，对应位置运动速度。
- 夹爪关节速度设置为 `1.0`。

注释中的过程是先更新目标位姿，再由相邻旋转矩阵的对数映射等计算 `twist`。因此，这个辅助函数并不是把输入 RPY 方向原封不动当作角速度发送。

### 8.4 LOWCMD 的基本控制顺序

基于官方 C++ 示例，建议使用以下生命周期，并补上状态确认：

```text
启动后台通信
  → 请求 PASSIVE
  → 请求 LOWCMD，并确认切换成功
  → 设置/确认低层增益
  → 停止后台通信
  → 在自己的周期循环中：
       读取反馈
       生成参考 q、qd
       计算前馈 tau
       setArmCmd / setGripperCmd
       sendRecv
       等待下一周期
  → 重新启动后台通信
  → 切回高层状态，处理收尾
```

示例先切状态、再停止后台线程，是因为状态切换依赖持续通信。停止后台线程后，由用户循环接管收发；不要让两个线程同时调用同一实例的 `sendRecv()`。

### 8.5 `setArmCmd()` 实际只做赋值

它在头文件中的完整逻辑就是：

```cpp
lowcmd->setQ(q);
lowcmd->setQd(qd);
lowcmd->setTau(tau);
```

`setGripperCmd()` 同样只是更新夹爪命令。两者都不负责执行通信。LOWCMD 循环漏掉 `sendRecv()` 时，调用这些函数不会把新参考送到控制器。

`tau` 的接口注释注明只在 LOWCMD 使用；不能假定在 JOINTCTRL 中写入 `tau` 就会获得自定义力矩控制。

### 8.6 示例里的前馈力矩包含什么

`lowcmd_development.cpp` 调用：

```cpp
tau = armModel->inverseDynamics(q, qd, Vec6::Zero(), Vec6::Zero());
```

这表示给模型提供目标位置、目标速度、零目标加速度和零末端外力。模型计算相应的逆动力学力矩；它不等于“所有动态过程都已补偿”，尤其不包含未建模摩擦，也没有补偿参考速度瞬间建立时的实际加速度。

如需纯模型重力项，可把三个向量参数都设为 `Vec6::Zero()`。如需完整轨迹前馈，应把轨迹的 `q/qd/qdd` 一起送入模型。

## 9. UDP 消息与反馈字段

### 9.1 消息是紧凑结构体

[`arm_common.h`](include/unitree_arm_sdk/message/arm_common.h) 使用 `#pragma pack(1)`。在本次 Linux x86_64 / GCC 环境中，实测布局为：

| 类型 | 字节数 | 主要字段 |
| --- | --- | --- |
| `JointCmd` | 20 | 5 个 float：`T, W, Pos, K_P, K_W` |
| `Motor_State` | 3 | 温度、错误位、连接状态 |
| `JointState` | 22 | `T, W, Acc, Pos` + 两组电机状态 |
| `Posture` | 48 | 6 个 double：`rx, ry, rz, x, y, z` |
| `TrajCmd` | 128 | 轨迹类型、两个位姿、夹爪、速度、停止时间、顺序 |
| `ValueUnion` | 140 | 标签 / 7 个关节命令 / 轨迹命令 |
| `SendCmd` | 147 | 包头、状态、跟踪标志、联合体 |
| `RecvState` | 208 | 包头、状态、7 个关节反馈、末端位姿 |

`SendCmd` 的布局可以直接计算：

```text
2 字节 head
+ 4 字节 ArmFSMState
+ 1 字节 track
+ 140 字节 ValueUnion
= 147 字节
```

协议固定长度由 `SENDCMD_LENGTH`、`RECVSTATE_LENGTH` 等 `sizeof` 常量导出。关节命令上线后用 float 表示，而应用侧 Eigen 与容器主要用 double，不是全链路双精度。

### 9.2 `ValueUnion` 每次只能按一种含义解释

```cpp
union ValueUnion {
    char name[10];
    JointCmd jointCmd[7];
    TrajCmd trajCmd;
};
```

这三种表示共享同一段内存：

- 标签操作使用 `name`。
- 跟踪或 LOWCMD 使用关节/控制相关内容。
- 高层运动使用 `trajCmd`。

解析方必须结合 `state` 和 `track` 决定如何解释，不能把三组数据看成同时存在的独立字段。

结构体声明本身还没有给出包头取值、字节序转换和完整有效包校验规则。跨语言重写协议时，单独对齐 147/208 字节不够，还要核对枚举尺寸、bool 表示、位域布局和控制器实际编码。

### 9.3 电机状态不是单一 bool

`Motor_Connected.state` 是 2 位状态：

- `0`：正常。
- `1`：连接异常。
- `2`：CRC 异常。

错误字节的注释列出 `0x01` 电流异常、`0x02` 漏电、`0x04` 过热、`0x20` 参数跳变；`0x40` 在反馈头文件中标为忽略项。应按位处理，而不是把错误字节当作唯一枚举值。

`LowlevelState` 的说明把连接/CRC 异常描述为曾发生的事件，头文件没有给出自动清除语义。判断当前设备是否健康时，还需要结合连续反馈和实际控制器行为。

### 9.4 UDPPort 参数要按方向读

构造函数为：

```cpp
UDPPort(toIP, toPort, ownPort,
        recvLength = 0,
        blockYN = BlockYN::NO,
        timeOutUs = 20000);
```

其中 `toPort` 是目标控制器端口，`ownPort` 是 SDK 本地端口，超时单位是微秒。`BlockYN::NO` 是这个封装的模式选择，内部具体如何使用 `select()` 或其他系统调用由库实现；不要仅凭名字把整个调用的耗时当作零。

`IOPort` 暴露 `isDisConnect`，但公开头文件没有说明它的阈值或更新时间。它可以作为诊断线索，不能替代带时间戳的新反馈判定。

## 10. 运动学、动力学和 QP

### 10.1 `ArmModel` 与 `Z1Model`

`ArmModel` 是六轴模型基类，保存关节限位、速度限制、重力、螺旋轴、连杆质量、质心和惯量等数据。`solveQP()` 与 `checkInSingularity()` 为纯虚函数。

`Z1Model` 提供具体 Z1 模型，并覆盖 IK、奇异判断与 QP。它的构造参数包括末端局部偏置、末端质量、质心和惯量，适合把工具对运动学和动力学的影响纳入计算。

`addLoad(load)` 的单位为 kg，注释说明它按比例把负载作用到末端关节并更新动力学参数。该简化接口不等于提供了任意工具的完整惯性标定。

### 10.2 正运动学与逆运动学

```cpp
HomoMat forwardKinematics(Vec6 q, int index = 6);
bool inverseKinematics(HomoMat TDes, Vec6 qPast,
                       Eigen::Ref<Vec6> q_result,
                       bool checkInWorkSpace = false);
```

- FK 把关节角映射为 4×4 齐次变换；`index=6` 表示末端，其他可选索引为 `0..5`。
- `qPast` 是参考关节构型，用于求取与其相关的解。
- `q_result` 是输出参数；只有求解成功后才应使用。
- `checkInWorkSpace=false` 时，注释要求解在 `qPast` 附近；设为 `true` 时可用于检查目标是否存在解，而不要求接近参考构型。

基类 IK 的注释描述迭代方法，`Z1Model` 的覆盖函数描述解析方法。覆盖函数后面仍残留“最大迭代次数”的模板文字，不能据此认为 Z1 的解析 IK 一定采用该迭代流程。

做连续轨迹 IK 时，通常把上一时刻成功的解作为下一时刻 `qPast`，并检查解的连续性和关节边界；仅逐点存在 IK 解不能证明整段轨迹可执行。

### 10.3 雅可比与 QP

`CalcJacobian(q)` 返回 6×6 空间雅可比，其关系为：

```text
V_space = J_space(q) × qd
```

`solveQP(twist, qPast, qd_result, dt)` 则根据目标速度、当前关节和周期求关节速度。注释说明它使用 QuadProg++，用于处理直接求逆不可用的情况。

可以用“在一定约束下寻找合适的关节速度”理解其职责；当前源码没有提供实际构造的代价矩阵、正则项或约束矩阵，不能把某个常见最小二乘公式直接认定为此库的实现。

`example_model.py` 使用 `np.linalg.inv(J)` 作演示。实际开发不宜默认雅可比总可逆；即便可逆，接近奇异位形时也可能放大速度。QP 也不会让机械臂获得奇异方向上原本不存在的运动能力。

`checkInSingularity()` 的注释只列出一个 Z1 几何判断：J1/J5 近似共轴时，用 `x5² + y5² < 0.1²` 判定。它不是完整的雅可比条件数检查，更不是碰撞检测。

### 10.4 逆动力学与关节保护

```cpp
Vec6 inverseDynamics(Vec6 q, Vec6 qd, Vec6 qdd, Vec6 Ftip);
void jointProtect(Eigen::Ref<Vec6> q, Eigen::Ref<Vec6> qd);
```

按模型接口及 `robotics.h` 的定义，逆动力学可理解为：

```text
tau = M(q) qdd + c(q, qd) + g(q) + 末端外力对应的关节力矩
```

这里 `Ftip` 是空间力向量，不是 RPY 位姿。它应按力矩/力分量理解；用于非零外力时，还需核对模型包装层采用的表达坐标系和符号约定。

`jointProtect()` 会就地限制 q 与 qd 至模型边界。它没有提供整条轨迹的碰撞、加速度、jerk 或力矩验证；把越界采样简单截断还可能破坏轨迹的导数连续性。规划阶段应先确保参考满足约束。

### 10.5 一个不创建通信对象的离线 C++ 示例

下面仅构造 `Z1Model`，完成 FK → IK，并计算静止时的模型力矩。它不创建 `unitreeArm`，不启动通信，也不下发动作。

将代码保存为 `/tmp/z1_model_demo.cpp`：

```cpp
#include "unitree_arm_sdk/model/ArmModel.h"
#include <iostream>

int main() {
    UNITREE_ARM::Z1Model model;
    Vec6 q;
    q << 0.0, 1.5, -1.0, -0.54, 0.0, 0.0;

    const HomoMat T = model.forwardKinematics(q, 6);
    Vec6 solution = Vec6::Zero();
    const bool ok = model.inverseKinematics(T, q, solution, false);
    if (!ok) {
        std::cerr << "IK failed\n";
        return 1;
    }

    const Vec6 tau = model.inverseDynamics(
        q, Vec6::Zero(), Vec6::Zero(), Vec6::Zero());
    std::cout << "FK:\n" << T << '\n';
    std::cout << "IK: " << solution.transpose() << '\n';
    std::cout << "Static model torque: " << tau.transpose() << '\n';
    std::cout << "FK-IK error: "
              << (model.forwardKinematics(solution, 6) - T).norm() << '\n';
}
```

在仓库根目录编译并运行：

```bash
g++ -std=c++14 -O2 -pthread \
  -I "$PWD/src/z1_sdk/include" /tmp/z1_model_demo.cpp \
  -L "$PWD/src/z1_sdk/lib" -Wl,-rpath,"$PWD/src/z1_sdk/lib" \
  -lZ1_SDK_"$(uname -m)" -o /tmp/z1_model_demo
/tmp/z1_model_demo
```

这个示例使用默认的 `Z1Model` 工具参数；结果用于检查软件链路，不代表已经匹配当前实物工具、负载或安装姿态。

### 10.6 数学工具分别承担什么职责

- `mathTypes.h`：把 Eigen 固定维度矩阵命名为 `Vec3`、`Vec6`、`Vec7`、`Mat6`、`HomoMat` 等，基础标量为 double。这些类型别名定义在全局命名空间，不是 `UNITREE_ARM` 的成员。
- `robotics.h`：声明旋转、齐次变换、指数/对数映射、伴随变换、空间/物体运动学及动力学函数，位于 `robo` 命名空间；文件说明其接口改编自 Modern Robotics。
- `mathTools.h`：提供区间判断、限幅、死区、归一化、窗口、均值和协方差等工具；其中 `min()`、`angleError()` 的问题见第 16 节。
- `typeTrans.h`：将标量和矩阵展开到 `std::vector<double>`，矩阵按“先行、再列”遍历；提取时消耗向量前部的数据。它是应用数据转换工具，不是 `SendCmd` 的 UDP 字节编码器。
- `quadProgpp`：提供 QP 求解接口与数组类型；头文件列出标准二次目标、等式约束和不等式约束形式。Z1 如何组装这些矩阵，仍由核心库实现。

## 11. 控制周期、线程与共享数据

### 11.1 `0.002 s` 是目标周期

SDK 注释和多个示例使用 `dt=0.002 s`，对应目标频率：

```text
f = 1 / dt = 500 Hz
```

但控制代码、收发耗时和系统调度都会占用周期。`dt` 只是算法和定时器的参数，不能证明实际一直运行在 500 Hz。

### 11.2 `Timer` 和 `LoopFunc` 的实现含义

[`loop.h`](include/unitree_arm_sdk/utilities/loop.h) 提供完整可读实现：

- `Timer` 使用 `std::chrono::steady_clock` 测量耗时。
- `Timer::sleep()` 只休眠当前周期剩余时间，超时则不休眠，然后重置起点。
- `LoopFunc::spinOnce()` 先重置计时、调用回调，再等待剩余时间。
- 回调超时会累加计数，但相关日志已注释，也没有公开统计读取接口。
- 线程由普通 `std::thread` 创建，未设置实时调度策略或 CPU 亲和性。

因此这是一个实用的周期线程工具，而不是绝对时隙调度器。循环中同步打印、HTTP 处理或长时间阻塞接收都会影响周期。

Python 低层示例的 `time.sleep(dt)` 更简单：它在本轮计算和收发结束后再额外休眠 `dt`，实际周期会包含两部分时间，不是严格的 2 ms。

### 11.3 一份通信状态应有一个活动写入者

低层示例在用户循环前关闭后台线程，正是为了把“生成命令 + 收发”交给一个循环。自行设计应用时，推荐：

```text
规划 / UI / 网络线程
        │ 提交目标或快照
        ▼
单一控制线程
更新命令 → sendRecv → 生成反馈快照
```

不要让多个回调直接并发修改同一个 `arm.q`、`lowcmd` 或 `recvState`。头文件里的公开字段没有相应的锁或快照机制；`LoopFunc` 的运行标志也只是普通 bool，跨线程访问未显式同步。

高层示例一边修改命令、一边让后台线程收发，展示的是基本用法，不能直接当作具备完整线程安全保证的生产架构。

### 11.4 退出程序与被动状态是不同操作

`sendRecvThread->shutdown()` 只停止并 join 通信线程；它没有在内联实现中发送 PASSIVE。程序退出后的控制器行为取决于控制器的通信失联处理。

`PASSIVE` 又不等于受约束的减速停止，也不保证主动保持当前位置。退出策略应由应用明确安排，并在切换成功后再停止通信；不能用强制终止进程替代正常控制收尾。

## 12. 逐个理解 C++ 示例

### 12.1 `highcmd_basic.cpp`：覆盖主要高层接口

它定义 `Z1ARM : public unitreeArm`，依次演示：

1. 启动通信并回初始位姿。
2. `labelRun("forward")`，再执行 MoveJ、MoveL 和 MoveC。
3. JOINTCTRL 下，给 J4 和夹爪持续发送负方向输入。
4. CARTESIAN 下，给末端 Z 方向和夹爪持续发送负方向输入。
5. 回初始位姿、请求 PASSIVE、关闭通信。

注意主函数中的 `size_t demo = 3` 被后面的 `for(size_t demo = 1; demo < 4; demo++)` 遮蔽：程序实际会运行三段演示，而不是只执行第 3 段。

此外，代码没有处理 `Move*()`、`setFsm()` 的失败返回。示例位姿和速度也属于其测试场景，不应直接当作当前整机的安全目标。

### 12.2 `highcmd_development.cpp`：自行生成关节参考

进入 JOINTCTRL 后，读取 `lastPos`，并在 1000 次循环里线性插值到：

```text
[0.0, 1.5, -1.0, -0.54, 0.0, 0.0] rad
```

数学形式为：

```text
q(i)  = q0 × (1 - i/N) + q1 × (i/N)
qd(i) = (q1 - q0) / (N × dt)
```

在 `dt=0.002 s` 的目标周期下，1000 个周期约为 2 s。循环条件是 `i < N`，最后一帧比例为 `999/1000`，没有显式发送终点帧，也没有设置终点零速度。

这段示例适合说明接口，但不是首尾速度连续的轨迹生成器。用于实际动作时，应生成满足边界速度/加速度的轨迹，并处理终点保持。

### 12.3 `lowcmd_development.cpp`：接管低层控制循环

它在 LOWCMD 中执行类似的插值，并增加逆动力学前馈和夹爪位置变化：

```text
插值 q、qd
  → inverseDynamics(q, qd, 0, 0)
  → setArmCmd(q, qd, tau)
  → setGripperCmd(...)
  → sendRecv()
```

其中 `KP`、`KW` 是从已有 `lowcmd` 读出后原样写回；这几行本身没有展示一套新调过的增益。默认成员如何初始化，需要结合实际库版本核对。

### 12.4 `lowcmd_multirobots.cpp`：展示两套独立组件

这个示例最有价值的部分是：每个机械臂都有独立的 `CtrlComponents`、UDP 端口、模型和状态。它不应被理解为完整的双臂协调控制器。

需要特别注意三处：

- 第二臂速度使用了 `targetQ-lastArm1Q`，按其自身插值位置应使用 `targetQ-lastArm2Q`。
- 两臂 `sendRecv()` 顺序执行，不具备同时发送或同时采样保证。
- 结束时直接删除对象，没有显式请求 PASSIVE；退出策略不能照搬到实机任务。

第二个模型还显式填入工具偏置 `0.0382 m`、质量 `0.80225 kg`、质心和惯量，旁边却仍写着 `no UnitreeGripper`。阅读时应以传入参数为准，不要由注释推断两臂模型相同。

## 13. Python 绑定与 HTTP 示例

### 13.1 Python API 来自绑定源码

`ArmInterface` 继承 `unitreeArm`，额外提供：

- `loopOn()` / `loopOff()`：启动和停止后台收发线程。
- `setFsmLowcmd()`：启动通信 → 请求 PASSIVE → 请求 LOWCMD → 关闭后台通信。
- `getCurrentState()`：返回 `recvState.state`。

`setFsmLowcmd()` 没有向 Python 返回两次 `setFsm()` 的成功与否；调用后仍需确认实际状态，不能只以“函数没抛异常”判断已经进入 LOWCMD。

C++ 的 `gripperW` 在 Python 中暴露为 **`gripperQd`**。其他常用字段包括 `q`、`qd`、`tau`、`gripperQ`、`gripperTau` 和 `lowstate`。

### 13.2 C++ 的默认参数不会自动成为 Python 默认参数

绑定源码只有显式写了 `py::arg()` 的地方才声明 Python 默认值。当前：

- `ArmInterface()` 的 `hasGripper` 默认是 `true`；本项目构建的绑定另支持 `controllerPort`、`sdkPort` 和 `load`，默认仍兼容上游单臂用法。
- `forwardKinematics(q, index)` 在 Python 侧需要提供 `index`。
- `setArmCmd(q, qd, tau)` 需要提供 `tau`。
- `setGripperCmd(q, qd, tau)` 需要提供第三个参数。
- 直接创建 `Z1Model` 需要提供四个构造参数，C++ 构造函数的默认值未在绑定中声明。

下面的模型创建方式不依赖控制器：

```python
import numpy as np
import unitree_arm_interface as sdk

model = sdk.Z1Model(np.zeros(3), 0.0, np.zeros(3), np.zeros((3, 3)))
q = np.array([0.0, 1.5, -1.0, -0.54, 0.0, 0.0])
T = model.forwardKinematics(q, 6)
ok, solution = model.inverseKinematics(T, q, False)
if not ok:
    raise RuntimeError("IK failed")
print(solution)
```

### 13.3 输出参数在 Python 中变成返回值

| 接口 | C++ | Python 绑定 |
| --- | --- | --- |
| `inverseKinematics` | 返回 bool，通过引用写出 q | 返回 `(hasIK, q_result)` |
| `solveQP` | 通过引用写出 qd | 返回 qd |
| `jointProtect` | 原地修改 q、qd | 返回 `(q, qd)` |

因此 Python 的 `jointProtect()` 应接收返回结果：

```python
q, qd = model.jointProtect(q, qd)
```

`LowlevelState` 绑定了 `getTau()`，同时保留误拼写别名 `getQTau()`。但 C++ 中可见的 `getQdd()`、末端位姿和全部错误数组，没有完整地暴露到这份 Python API。

### 13.4 `.pyi` 不是协议枚举的可靠来源

[`unitree_arm_interface.pyi`](examples_py/unitree_arm_interface.pyi) 与 C++ 头文件有明显不同：

| 状态 | C++ / 按源码重建的绑定 | `.pyi` |
| --- | --- | --- |
| `TRAJECTORY` | 7 | 11 |
| `TOSTATE` | 8 | 9 |
| `TEACH` | 10 | 7 |
| `LOWCMD` | 16 | 12 |

pybind11 的 `.value()` 引用 C++ 枚举，因此重新构建后的模块值以 C++ 为准。类型提示文件主要影响编辑器，但如果有人据此手写数字协议，就会发错状态。

类型提示还把 IK 标成只返回 bool、把夹爪位置 getter 标成数组等，与实际绑定不同。开发时应使用枚举成员名，并以 `arm_python_interface.cpp` 为接口依据。

Python 枚举也没有完整导出 C++ 的所有成员，例如 `CALIBRATION`、`SETTRAJ`、`BACKTOSTART`、`NEXT`；但 `calibration()`、`backToStart()` 这些便捷方法本身已绑定。

### 13.5 Python 示例的各自作用

- `example_highcmd.py`：对应高层运动与两种连续跟踪。
- `example_lowcmd.py`：自行维护 LOWCMD 循环；速度计算硬编码 `0.002`，修改周期时需同步处理。
- `example_model.py`：演示模型计算，但仍通过 `ArmInterface` 取模型，不是最小的纯模型入口。

这些脚本使用 `sys.path.append("../lib")`，相对的是**启动时的工作目录**。按原样运行时应从 `examples_py` 启动，或改用前面的绝对环境路径配置。

Python 低层示例结束后执行 `backToStart()` 与 `loopOff()`，没有显式进入 PASSIVE；高层示例也没有像 C++ 主示例那样发送最终 PASSIVE。异常中断路径同样没有完整清理流程。

### 13.6 HTTP 示例是进程内包装，不是另一套控制协议

[`example_http_service.py`](examples_py/example_http_service.py) 用 FastAPI 提供：

```text
POST http://主机:12000/unitree/z1
{
  "func": "MoveJ",
  "args": { ... }
}
```

服务端只有一个全局 `arm`，启动后台通信后交给 Uvicorn 接收请求。它额外依赖 FastAPI、Pydantic、Uvicorn；客户端示例依赖 Requests，相关脚本还使用 NumPy。

这里的 `MoveJ(q=...)` 是 **HTTP 包装层**的扩展：服务先把关节角做 FK，转换成 `posture`，再调用 SDK `MoveJ`。SDK 的原始 MoveJ 接口没有因此变成关节角接口，也不能保证 IK 最后选择原关节分支。

要把该服务用于真实任务，应先补齐三个会直接影响控制结果的点：

1. **单一命令执行者**：同步请求处理函数可能并发访问全局机械臂，需要串行调度控制操作。
2. **输入与任务状态**：现在参数主要用无类型字典表达，没有完整的范围校验、任务 ID、取消或到位结果模型；未知 `func` 也没有明确拒绝。
3. **访问与生命周期**：服务监听 `0.0.0.0`，没有鉴权；`uvicorn.run()` 后面的 `loopOff()` 也不是完整的异常退出清理机制。

HTTP 适合下发任务和读取状态，不适合替代 500 Hz 周期闭环。

## 14. 双机械臂通信如何配置

### 14.1 示例端口的方向

`lowcmd_multirobots.cpp` 中的配置为：

| 实例 | SDK 目标地址 | SDK 本地端口 | 控制器端应对应配置 |
| --- | --- | --- | --- |
| arm1 | `127.0.0.1:8071` | 8072 | 控制器本地 8071，回发 SDK 8072 |
| arm2 | `127.0.0.1:8073` | 8074 | 控制器本地 8073，回发 SDK 8074 |

文件注释提到要修改第二个控制器的 `ARMSDK(..., 8074, 8073, ...)`。它与 SDK 构造参数的方向不同，核对时要按“谁绑定、向谁发送”理解，不能机械照抄顺序。

这些端口是该示例明确写出的配置。单参数 `unitreeArm(true)` 的默认网络设置由库构造函数决定，头文件没有列出其数值。

### 14.2 两套实例不等于双臂同步

两臂需要分别维护端口、反馈、模型和控制器状态。工具质量不同、是否安装夹爪不同，也应分别配置，不能只复制同一个模型。

示例在同一个 2 ms 循环中依次执行两臂收发。任一收发耗时增加都会挤占共同周期；它没有统一启动时间、反馈时间戳对齐、协调轨迹或双臂碰撞检查。

需要双臂协调时，应该在上层统一规划和时间基准，同时保留每臂独立的通信健康与故障处理。

## 15. 与本项目双臂集成的关系

旧自研 Z1 ROS 控制栈已从当前工作树移除。现在左右臂分别运行完整的官方链路：

```text
每臂应用 → 官方 SDK → 对应官方 z1_ctrl → 对应 MCU
```

[dual_arm/config/arms.yaml](dual_arm/config/arms.yaml) 定义两臂地址、独立端口、是否安装夹爪和负载；[双臂启动器](dual_arm/scripts/z1_official.py) 隔离官方控制器工作目录。原始 `main.cpp` 的状态注册和控制周期保持不变，构建时只参数化其中写死的 SDK 端口。

[初始化辅助函数](dual_arm/include/z1_official/sdk.hpp) 根据官方构造函数和多机器人示例创建组件；原始 Python 绑定只增加相同的双臂端口、负载构造参数。算法仍使用官方库，不复用旧自研轨迹、动力学、摩擦补偿和控制状态机。

显示用机械臂资源独立位于 [z1_description](../z1_description)，不参与官方控制器的模型计算。显示坐标系 `tool` 的偏置不能直接作为官方 SDK 的末端定义。

旧 ROS Topic、Disabled/Holding 状态、RViz 轨迹预览和自研标定/辨识工具不再是当前运行接口。双臂控制、C++/Python 用法和实际退出语义以 [README](README.md) 为准。

## 16. 当前代码中值得优先处理的问题

下面是上游原始源码中的具体问题。当前集成保留原始示例和库，不修改其中算法；本项目构建已补齐 Eigen 包含路径，并隔离运行目录与端口。

| 位置 | 具体问题 | 影响与建议 |
| --- | --- | --- |
| `lowcmd_multirobots.cpp` | 第二臂 qd 误用 `lastArm1Q` | q/qd 不一致；二次开发前改用第二臂起点 |
| `highcmd_development.cpp`、`lowcmd_development.cpp` | `i<N` 没有终点采样，且首尾速度未平滑 | 补齐轨迹边界和终点保持，不直接作为成熟轨迹生成器 |
| `highcmd_basic.cpp` | `demo` 变量遮蔽 | 修改外层 `demo=3` 不会只运行第三段；使用显式演示选择入口 |
| 多个示例 | 不检查状态切换/运动返回值，退出处理不一致 | 把状态确认、执行结果和正常退出纳入应用逻辑 |
| `unitree_arm_interface.pyi` | 枚举值、返回类型与绑定不一致 | 根据头文件和绑定源码更新提示文件；业务代码不用手写状态数字 |
| `CMakeLists.txt` | Python 目标未显式获取 Eigen 包含目录 | 在常见系统目录布局下会编译失败；使用 `Eigen3::Eigen` |
| HTTP 服务 | 全局共享 arm、开放监听、缺少任务模型 | 先串行化、鉴权和定义结果，再用于实机远程控制 |
| `loop.h` | 普通 bool 跨线程读写，公开状态缺少同步 | 生产封装应明确线程所有权及同步方式 |

数学工具中还有两个可直接复现的错误，复用时尤其需要注意：

### 16.1 `min()` 实际选择了较大值

[`mathTools.h`](include/unitree_arm_sdk/math/mathTools.h) 的多参数 `min()` 比较方向写反；本次直接调用：

```cpp
UNITREE_ARM::min(2.0, 3.0);  // 实际为 3.0
```

新代码应使用 `std::min()`，或先修正并测试该工具。不要在关节边界和速度限幅中复用这个错误结果。

### 16.2 `angleError()` 没有实现默认的最短角差

默认分支比较 `fabs(a-b)` 与 `fabs(b-a)`，两者相等，因而无法据此选择最短环绕方向。本次：

```cpp
UNITREE_ARM::angleError(3.0, -3.0);  // 实际为 6.0 rad
```

如果目标是最短角差，应将差值归一化，例如采用 `atan2(sin(delta), cos(delta))`，同时明确 ±π 边界约定。这里的复现证明的是当前头文件内联函数的问题；二进制核心库是否调用了同版本工具，当前源码无法直接确认。

## 17. 排查顺序与离线核验记录

### 17.1 遇到问题先定位所在层

| 现象 | 优先检查 |
| --- | --- |
| 找不到 `Z1_SDK_*` | CPU 架构名称、`lib` 目录、链接参数 |
| Python 编译找不到 `Eigen/Core` | Python 目标是否有 Eigen 包含路径 |
| Python 无法导入模块 | Python ABI、模块文件位置、`PYTHONPATH` |
| 导入时报 SDK `.so` 不存在 | `LD_LIBRARY_PATH` 或部署 RPATH |
| 高层函数等待不返回 | 控制器是否启动、收发循环是否运行、端口方向与状态回执 |
| `labelRun("forward")` 失败 | 控制器的标签文件是否有此名称 |
| 能设置 q，但机械臂不更新 | 当前状态、跟踪是否开启、是否真的执行 `sendRecv()` |
| MoveJ 目标异常 | 是否把六个关节角误当成 `[r,p,y,x,y,z]` 位姿 |
| 轨迹末端抖动或有误差 | q/qd 一致性、终点帧、边界速度、实际周期与反馈 |
| 双臂第二臂速度不对 | 是否照搬了示例中 `lastArm1Q` 的错误引用 |

### 17.2 本文写作时完成的核验

核验日期：`2026-11-01`。环境为 Linux x86_64、GCC 11.4；未启动 SDK 控制示例，也未连接实机。

- **C++ 构建**：关闭可选 Python 模块后，四个官方 C++ 示例均生成可执行文件。
- **协议布局**：用当前头文件编译探针，确认 `SendCmd=147`、`RecvState=208` 字节，以及上文列出的中间类型尺寸。
- **状态值**：确认 `TRAJECTORY=7`、`TEACH=10`、`LOWCMD=16`，与 `.pyi` 不一致。
- **纯模型计算**：默认 `Z1Model` 对 `q=[0,1.5,-1,-0.54,0,0]` 完成 FK → IK；回代齐次矩阵的 Frobenius 范数误差约为 `2.01×10⁻¹⁶`，静止模型力矩为有限值。
- **位姿转换**：该姿态的 `homoToPosture()` → `postureToHomo()` 回代矩阵误差约为 `1.57×10⁻¹⁶`。
- **工具函数复现**：确认 `min(2,3)=3`，以及 `angleError(3,-3)=6`。
- **Python 绑定构建与调用**：使用系统 Python 3.10、pybind11 2.9.1 的隔离构建，复现了缺少 Eigen 包含路径的错误；给隔离目标补上 `Eigen3::Eigen` 后成功生成并导入扩展模块。直接创建 `Z1Model` 后，FK → IK、静止力矩和 `jointProtect()` 返回值检查通过，同时确认 Python FK 的 `index` 参数必须显式传入。构建产物位于临时目录，没有覆盖随项目提供的 CPython 3.8 模块。

本次未验证设备侧状态切换、实际频率、轨迹精度、失联响应和实机退出行为。

### 17.3 主要源码索引

- [构建入口](CMakeLists.txt)
- [应用控制接口](include/unitree_arm_sdk/control/unitreeArm.h)
- [组件组合](include/unitree_arm_sdk/control/ctrlComponents.h)
- [协议与状态枚举](include/unitree_arm_sdk/message/arm_common.h)
- [命令容器](include/unitree_arm_sdk/message/LowlevelCmd.h)与[反馈容器](include/unitree_arm_sdk/message/LowlevelState.h)
- [UDP 接口](include/unitree_arm_sdk/message/udp.h)
- [模型接口](include/unitree_arm_sdk/model/ArmModel.h)
- [定时与线程实现](include/unitree_arm_sdk/utilities/loop.h)
- [Python 绑定源码](examples_py/arm_python_interface.cpp)
- [Python 类型提示](examples_py/unitree_arm_interface.pyi)
- [原始 README 中的官方文档入口](README.md)
