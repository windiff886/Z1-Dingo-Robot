# Z1 SDK：应用接口、MCU 通信与内部实现

应用调用 `MoveJ()`，给出的是“末端要到哪里”；MCU 收到的则是每个控制周期的关节位置、速度、力矩和增益。把前者变成后者的工作，由应用侧 SDK 与独立运行的官方控制器 `z1_ctrl` 共同完成。

先把整条链路分清楚：

```text
应用程序（C++ / Python）
        │ 函数调用、命令参数、反馈读取
        ▼
应用侧 SDK：unitreeArm / ArmInterface
        │ UDP：状态请求、运动目标或连续控制参考
        ▼
PC 上的官方控制器：z1_ctrl
        │ 状态机 → 轨迹 / 模型计算 → 关节命令
        │ UDP：周期关节指令与关节反馈
        ▼
机械臂 MCU
        │ 设备内部电机通信
        ▼
六轴机械臂，以及可选夹爪
```

**应用侧 SDK 不直接连接 MCU。** 本目录同时包含 SDK、配套控制器和双臂适配，所以下文分别说明三件事：

1. [向上：应用能调用什么接口](#1-向上应用能调用什么接口)
2. [向下：控制器怎样与 MCU 通信](#2-向下控制器怎样与-mcu-通信)
3. [中间：接口怎样变成周期控制](#3-中间接口怎样变成周期控制)

## 1. 向上：应用能调用什么接口

### 1.1 先选左臂或右臂：一个 SDK 实例对应一台机械臂

C++ 的总入口是 [`UNITREE_ARM::unitreeArm`](include/unitree_arm_sdk/control/unitreeArm.h)。Python 的入口是 `unitree_arm_interface.ArmInterface`，它继承同一套 C++ 功能。

**左右臂使用相同的方法，通过不同的 SDK 实例区分。** `MoveJ()`、`setArmCmd()` 等方法没有“左 / 右”参数；实例初始化时绑定哪套控制器，后续方法就作用于哪台机械臂。

本项目的 Python 接口按当前左右臂配置创建实例：

```python
import unitree_arm_interface as sdk

left = sdk.ArmInterface(hasGripper=True, controllerPort=8071,
                        sdkPort=8072, load=0.0)
right = sdk.ArmInterface(hasGripper=False, controllerPort=8073,
                         sdkPort=8074, load=0.0)
```

之后 `left.MoveJ(...)` 向左臂控制器下发目标，`right.MoveJ(...)` 向右臂控制器下发目标；`left.lowstate` 和 `right.lowstate` 分别保存两臂反馈。实际调用前需分别启动对应实例的收发循环。

C++ 使用相同的绑定方式，通过本项目的初始化辅助函数指定端口：

```cpp
#include <z1_official/sdk.hpp>

UNITREE_ARM::unitreeArm left(
    z1_official::components(8071, 8072, true, 0.0));
UNITREE_ARM::unitreeArm right(
    z1_official::components(8073, 8074, false, 0.0));
```

`left`、`right` 只是应用变量名，真正决定物理机械臂的是 **SDK 端口 → 对应控制器 → 控制器配置的 MCU IP**。当前左臂对应 `192.168.123.110`，右臂对应 `192.168.123.111`。`hasGripper` / `has_gripper` 只选择是否带夹爪的模型，不能用它区分左右臂。

### 1.2 选定机械臂后，选择控制方式

按应用希望承担多少控制工作，选择三种方式：

| 方式 | 应用提供什么 | 主要接口 | 控制器负责什么 |
| --- | --- | --- | --- |
| 离散运动 | 目标末端位姿、速度，可选夹爪目标 | `MoveJ()`、`MoveL()`、`MoveC()` | 生成并执行整段轨迹 |
| 连续跟踪 | 持续更新关节位置 / 速度，或末端空间速度 | `startTrack(JOINTCTRL)`、`startTrack(CARTESIAN)` | 将连续参考转成关节控制命令 |
| 低层控制 | 每周期的关节位置、速度、前馈力矩和增益 | `setFsm(LOWCMD)`、`setArmCmd()`、`lowcmd->setControlGain()` | 接收低层参考，经设备 IO 下发 |

**普通动作优先用 `Move*`；已有外部轨迹时用连续跟踪；需要自行计算前馈力矩和增益时再用 LOWCMD。** LOWCMD 仍然经过 `z1_ctrl`，不是绕过控制器直接向 MCU 发包。

#### 离散运动

```cpp
bool MoveJ(Vec6 posture, double maxSpeed);
bool MoveL(Vec6 posture, double maxSpeed);
bool MoveC(Vec6 middlePosture, Vec6 endPosture, double maxSpeed);
```

- `MoveJ`：沿关节空间路径到达目标末端位姿，速度参数单位为 rad/s。
- `MoveL`：末端沿直线运动，速度参数单位为 m/s。
- `MoveC`：由起点、中间点和终点确定圆弧，速度参数单位为 m/s。

三者都有带 `gripperPos` 的重载，夹爪参数位于速度参数之前，例如 `MoveJ(posture, gripperPos, maxSpeed)`。

**`MoveJ` 的六维输入是末端位姿，不是六个关节角。** “J”表示路径在关节空间生成。已有目标关节角时，连续跟踪或 LOWCMD 才是直接表达该目标的入口。

#### 连续跟踪

- `startTrack(JOINTCTRL)`：先用当前反馈初始化 `q`、`qd` 和夹爪参考，随后应用持续更新这些字段。
- `startTrack(CARTESIAN)`：先清零 `twist`，随后应用持续更新末端空间速度。接口注释规定其基于物体坐标系。
- `jointCtrlCmd(directions, jointSpeed)`：把七维方向输入转换成六轴及夹爪的运动参考，适合手柄或键盘。
- `cartesianCtrlCmd(directions, oriSpeed, posSpeed)`：把姿态、平移及夹爪方向输入转换成跟踪参考。

这些方法更新参考；实际传输依赖周期调用 `sendRecv()`。

#### LOWCMD

```cpp
void setArmCmd(Vec6 q, Vec6 qd, Vec6 tau = Vec6::Zero());
void setGripperCmd(double gripperPos, double gripperW,
                   double gripperTau = 0.0);
```

`setArmCmd()` 只把参数写入 `lowcmd`，`setGripperCmd()` 同理。六轴增益通过 `lowcmd->setControlGain(KP, KW)` 设置，夹爪增益通过 `setGripperGain()` 设置。`tau` 参数在 LOWCMD 中使用；JOINTCTRL 不是通过写入 `tau` 接管力矩的模式。

### 1.3 数据的顺序、单位与坐标含义

| 数据 | 顺序 | 单位 / 含义 |
| --- | --- | --- |
| `q` | J1～J6 | rad |
| `qd` / `dq` | J1～J6 | rad/s |
| `qdd` / `ddq` | J1～J6 | rad/s² |
| `tau` | J1～J6 | 关节力矩，N·m |
| `posture` | `roll, pitch, yaw, x, y, z` | 前三项 rad，后三项 m |
| `twist` | `ωx, ωy, ωz, vx, vy, vz` | 角速度 rad/s、线速度 m/s |
| `gripperPos` | 单个夹爪关节角 | rad；高层接口范围为 `[-π/2, 0]` |
| `directions` | 六个运动方向分量，再加夹爪 | 每项 `[-1, 1]` |

末端位姿使用官方控制器的机械臂基座和末端模型定义。整机 `base_link`、相机坐标或显示模型的 `tool` 不能直接替代它们；应用需先完成坐标变换。夹爪角度也不是开口宽度或百分比。

左右臂目标分别在各自机械臂的基座坐标系下解释；选择右臂实例不会自动把左臂目标镜像过去，也不会自动转换为整机坐标。

`twist` 的角速度不是 RPY 三个角各自的导数。模型接口 `CalcJacobian()` 返回空间雅可比，而 CARTESIAN 跟踪接口注明使用物体坐标系；两者相接时需统一表达坐标系和末端参考点。

### 1.4 状态、动作管理与反馈

| 接口 | 作用 |
| --- | --- |
| `setFsm(state)` | 请求控制器切换状态，等待切换结果，返回是否成功 |
| `backToStart()` | 回初始位姿，完成后进入 JOINTCTRL |
| `labelRun(name)` / `labelSave(name)` | 运行已保存位姿 / 保存当前位姿 |
| `teach(name)` / `teachRepeat(name)` | 示教录制 / 示教复现 |
| `calibration()` | 以当前物理位置进行零位标定，改变零位定义 |
| `setWait(bool)` | 设置 `Move*`、回初始位姿、标签运行和示教复现是否等待完成 |

状态切换受控制器约束。例如，进入 LOWCMD 需要先进入 PASSIVE。`PASSIVE` 是被动状态，不是主动姿态保持或平滑减速停止。

标签由控制器保存。当前位姿文件是 `config/savedArmStates.csv`；双臂各有独立副本。协议的标签缓冲区为 `char name[10]`，本项目客户端限制为 1～9 个 ASCII 字符。

C++ 应用从 `arm.lowstate` 读取最近反馈：

```cpp
arm.lowstate->getQ();          // 六轴位置
arm.lowstate->getQd();         // 六轴速度
arm.lowstate->getQdd();        // 六轴加速度
arm.lowstate->getTau();        // 六轴力矩
arm.lowstate->getGripperQ();   // 夹爪位置
arm.lowstate->endPosture;      // 末端位姿
```

容器还提供 `temperature`、`errorstate`、`isMotorConnected`。当前 FSM 状态位于 `arm._ctrlComp->recvState.state`；Python 提供 `getCurrentState()`。

运动函数默认等待控制器执行完成。`setWait(false)` 后，应用应确认动作已开始，再观察状态回到 JOINTCTRL，并结合位置、姿态和速度误差判断到位。`Move*` 的 `bool` 返回值主要表示目标的 IK 可解性，不是独立的实测到位或碰撞检查结果。

### 1.5 模型计算也作为接口向上开放

[`ArmModel` / `Z1Model`](include/unitree_arm_sdk/model/ArmModel.h) 提供：

| 接口 | 输入 → 输出 |
| --- | --- |
| `forwardKinematics(q, index)` | 关节角 → 指定连杆 / 末端的 4×4 齐次变换；末端索引为 6 |
| `inverseKinematics(T, qPast, q_result, checkInWorkSpace)` | 末端变换及参考构型 → 是否成功、关节解 |
| `CalcJacobian(q)` | 关节角 → 6×6 空间雅可比 |
| `inverseDynamics(q, qd, qdd, Ftip)` | 运动状态及末端外力 → 模型关节力矩 |
| `solveQP(twist, qPast, qd_result, dt)` | 目标空间速度、关节构型和周期 → 关节速度 |
| `jointProtect(q, qd)` | 将位置和速度限制到模型范围 |
| `addLoad(load)` | 按 kg 增加模型末端负载 |

这些计算可以直接使用独立的 `Z1Model`，不需要创建机械臂通信对象。`Z1Model` 的 IK 接口注明采用解析法，速度求解使用 QuadProg++。模型还接受工具偏置、质量、质心和惯量。

`jointProtect()` 是位置 / 速度限幅，`checkInSingularity()` 是特定几何奇异条件判断；两者都不提供整机或双臂碰撞规划。

### 1.6 Python、HTTP 与双臂入口的关系

Python 是同一套 C++ SDK 的绑定，主要差异见 [`arm_python_interface.cpp`](examples_py/arm_python_interface.cpp)：

- `loopOn()` / `loopOff()` 对应后台收发线程的启动 / 停止。
- C++ 的 `gripperW` 在 Python 中命名为 `gripperQd`。
- IK 返回 `(hasIK, q_result)`，`solveQP()` 返回 `qd`，`jointProtect()` 返回 `(q, qd)`。
- Python 的 `setArmCmd()`、`setGripperCmd()` 需要显式给出力矩参数，`forwardKinematics()` 需要给出 `index`。
- Python 的 `LowlevelState` 只绑定了部分 getter，没有完整暴露 C++ 的加速度、末端位姿和错误数组。

上游 `.pyi` 中部分枚举值及返回类型与实际绑定不一致，例如它将 LOWCMD 写为 12，而当前 C++ 枚举是 16。应用使用枚举成员名，接口含义以头文件和绑定源码为准。

上面的 `controllerPort`、`sdkPort`、`load` 是本项目扩展的 Python 构造参数；不指定端口时使用 8071 / 8072，不会自动选择空闲机械臂。C++ 的端口初始化见 [`z1_official::components()`](dual_arm/include/z1_official/sdk.hpp)。

启动脚本的 `--arm left` / `--arm right` 是另一层的配置选择：脚本读取 [`arms.yaml`](dual_arm/config/arms.yaml)，再把该臂的端口、夹爪和负载参数传给客户端。直接创建 C++ / Python SDK 对象不会自动读取这份 YAML，需要像第 1.1 节那样传入对应参数。

[`HTTP 示例`](examples_py/example_http_service.py) 是 FastAPI 对 Python SDK 的包装，入口为 `POST /unitree/z1`。其中 `MoveJ(q=...)` 会先将关节角做 FK，再调用原生的末端位姿接口；它不是 SDK 新增的关节角版 MoveJ。HTTP 用于任务调用，不承担周期控制。

## 2. 向下：控制器怎样与 MCU 通信

### 2.1 两段 UDP 的地址与职责不同

应用 SDK 的 UDP 对端是 PC 上的 `z1_ctrl`；只有控制器的 `IOUDP` 连接机械臂 MCU。当前双臂配置见 [`arms.yaml`](dual_arm/config/arms.yaml)：

| 通信端点 | 左臂 | 右臂 |
| --- | --- | --- |
| SDK 本地端口 | `127.0.0.1:8072` | `127.0.0.1:8074` |
| 控制器面向 SDK 的端口 | `127.0.0.1:8071` | `127.0.0.1:8073` |
| PC 面向 MCU 的本地端口 | `8881` | `8882` |
| MCU 目标地址 | `192.168.123.110:8880` | `192.168.123.111:8880` |

发送方向以左臂为例：

```text
SDK :8072  ──运动请求 / 连续参考──▶  z1_ctrl :8071
PC  :8881  ──周期关节控制指令────▶  MCU 192.168.123.110:8880
```

反馈沿相反方向返回。配置中的 `mcu_local_port` 是 **PC 的本地端口**；MCU 目标端口 8880 由官方控制器库使用，不是把 `8881` 当作 MCU 监听端口。

### 2.2 向 MCU 发什么：关节参考，而不是 MoveJ

设备侧接口是 [`IOUDP::sendRecv(const LowlevelCmd*, LowlevelState*)`](controller/include/interface/IOUDP.h)。它把控制器的 `q / dq / tau / kp / kd` 转为 [`JointCmd`](controller/include/message/arm_common.h)：

```cpp
struct JointCmd {
    float T;      // 前馈力矩，对应 tau
    float W;      // 目标关节速度，对应 dq
    float Pos;    // 目标关节位置，对应 q
    float K_P;    // 位置增益，对应 kp
    float K_W;    // 速度增益，对应 kd
};
```

每个逻辑关节 20 字节，六轴加可选夹爪最多占 7 个槽位。位置、速度、力矩和两类增益共同表达关节控制意图；具体设备侧增益缩放及电机控制算法不在这些头文件中。

**这一段没有 `MOVEJ`、末端目标或路径速度字段。** 高层运动必须先由 PC 控制器变成一串关节指令，MCU 才能执行。LOWCMD 也发送同样类型的关节指令，只是参考与前馈力矩改由应用提供。

### 2.3 从 MCU 收什么：关节反馈与电机状态

设备反馈有两种布局：

| 结构 | 每个逻辑关节的内容 | 每关节字节数 |
| --- | --- | --- |
| `JointStateOld` | `T, W, Acc, Pos`，均为 float | 16 |
| `JointState` | 同上，再加 `Motor_State state[2]` | 22 |

`T / W / Acc / Pos` 分别对应力矩、速度、加速度和位置。新版每个逻辑关节预留两组电机状态，每组包含温度、错误字节和连接状态；逻辑关节数量与物理电机数量不能直接等同。

正常周期交换的 UDP 负载长度随逻辑关节数变化：

| 设备配置 | 下发关节命令 | 旧版反馈 | 带电机状态的反馈 |
| --- | --- | --- | --- |
| 六轴，无夹爪 | `6 × 20 = 120` 字节 | `6 × 16 = 96` 字节 | `6 × 22 = 132` 字节 |
| 六轴，带夹爪 | `7 × 20 = 140` 字节 | `7 × 16 = 112` 字节 | `7 × 22 = 154` 字节 |

这些是正常周期的数据负载，不含 IP / UDP 首部，也不适用于初始化探测和标定命令。**147 / 208 字节是另一段“应用 SDK ↔ 控制器”协议的长度，不能用于 MCU 端口。**

`Motor_State` 的错误字节按位解释：`0x01` 为相电流过大，`0x02` 为漏电，`0x04` 为过热，`0x20` 为参数跳变。连接状态为 `0` 正常、`1` 断连、`2` CRC 错误。SDK 对连接字段的注释描述的是 MCU 与电机之间曾发生的通信事件，不是应用 SDK 的 UDP 连接状态。

MCU 反馈结构没有末端位姿。应用最终看到的末端位姿由 PC 控制器根据关节反馈和机械臂模型做 FK 得到。

### 2.4 先探测设备，再开始周期交换

`IOUDP` 初始化阶段通过 `UDPSendCmd.checkCmd` 探测 / 自检设备，按返回信息识别附件和关节数量，并区分旧版反馈与带电机状态的反馈。之后周期发送关节命令、接收对应布局的状态。`calibration()` 是单独的设备操作入口，不是普通运动目标。

以上结构见控制器的 [`arm_common.h`](controller/include/message/arm_common.h) 和 [`IOUDP.h`](controller/include/interface/IOUDP.h)；目标端口、初始化探测及按关节数收发的行为还核对了随仓库提供的 x86_64 控制器库。MCU 固件及其内部电机总线实现不在本目录中。

## 3. 中间：接口怎样变成周期控制

### 3.1 应用侧 SDK：组织命令、模型与通信

SDK 对象关系如下：

```text
unitreeArm
├── q / qd / tau / twist / gripper*     应用更新的控制参考
├── sendRecvThread                    周期执行 sendRecv()
├── lowcmd ───────────────────────┐
├── lowstate ──────────────────┐  │
└── _ctrlComp                  │  │
    ├── lowcmd  ◀──────────────┼──┘   命令容器
    ├── lowstate ◀─────────────┘      反馈容器
    ├── sendCmd / recvState           SDK 协议消息
    ├── armModel                     运动学 / 动力学模型
    ├── udp                          面向 z1_ctrl 的 UDPPort
    └── dt                           SDK 周期参数
```

`arm.lowcmd` 与 `arm._ctrlComp->lowcmd` 指向同一对象，`lowstate` 同理。

`unitreeArm::sendRecv()` 整理自身控制参数并调用 `CtrlComponents::sendRecv()`；后者负责把命令编码到 `SendCmd`、发送 UDP、接收 `RecvState` 并更新 `LowlevelState`。因此，“修改 `arm.q`”或“调用 `setArmCmd()`”与“控制器收到更新”之间，还差一次实际收发。

### 3.2 第一段 UDP：把应用意图交给控制器

SDK 侧的 [`arm_common.h`](include/unitree_arm_sdk/message/arm_common.h) 定义紧凑结构，使用 `#pragma pack(1)`。按当前 ABI，正常消息为：

```text
SendCmd，147 字节
├── head[2]                2 字节
├── state                  4 字节，目标 FSM 状态
├── track                  1 字节，是否使用连续跟踪参考
└── valueUnion           140 字节，按状态解释的命令内容

RecvState，208 字节
├── head[2]                2 字节
├── state                  4 字节，控制器当前 FSM 状态
├── jointState[7]        154 字节，六轴及夹爪反馈槽位
└── cartesianState        48 字节，六个 double 表示末端位姿
```

`valueUnion` 的存储由三种表示复用：

- `name[10]`：标签操作。
- `jointCmd[7]`：关节控制相关数据。
- `trajCmd`：轨迹类型、两个位姿、夹爪目标、最大速度、停留时间和轨迹顺序。

接收端结合 `state` 与 `track` 解释同一块内存，不能把它们当成同时有效的三个独立字段。关节数据在线上主要是 float，末端位姿是 double；应用侧使用 double 并不意味着整条链路都是双精度。

消息中没有左右臂编号。左右臂的命令和反馈通过各自独立的 UDP 端口收发，接收端口已经确定这条消息属于哪台机械臂。

控制器用 [`ARMSDK`](controller/include/control/armSDK.h) 接收这段协议，并向 SDK 回送当前状态和反馈。它是控制器的命令输入端，与控制器向 MCU 通信的 `IOUDP` 是两个独立对象。

### 3.3 控制器：状态机决定如何解释命令

[`controller/main.cpp`](controller/main.cpp) 创建控制组件、设备 IO、SDK 命令入口，并注册各个状态。核心调度由 [`FiniteStateMachine`](controller/include/FSM/FiniteStateMachine.h) 执行：每个状态都有 `enter()`、`run()`、`exit()` 和 `checkChange()`。

| 状态 | 输入如何变为控制参考 | 对应实现入口 |
| --- | --- | --- |
| MOVEJ | 目标末端位姿经 IK 得到关节目标，再生成关节轨迹 | `State_MoveJ`、`JointSpaceTraj` |
| MOVEL | 生成末端直线轨迹，再转换成逐周期关节参考 | `State_MoveL`、`EndLineTraj` |
| MOVEC | 根据起点、中间点和终点生成末端圆弧，再转换成关节参考 | `State_MoveC`、`EndCircleTraj` |
| JOINTCTRL | 使用关节位置 / 速度跟踪参考 | `State_JointSpace` |
| CARTESIAN | 使用末端运动参考，经模型求解关节运动 | `State_Cartesian` |
| LOWCMD | 使用应用提供的位置、速度、力矩和增益 | `State_LowCmd` |
| PASSIVE | 执行官方被动状态的命令设置 | `State_Passive` |

轨迹模块提供 `getJointCmd()`，输出 `q / qd` 及可选夹爪参考。`JointSpaceTraj`、`EndHomoTraj` 使用 `SCurve` 相关组件处理时间参数；`EndLineTraj`、`EndCircleTraj` 处理路径几何。也就是说，`maxSpeed` 参与整段轨迹的生成，而不是简单地附在每个 MCU 包后面。

普通运动状态的公共 `_armCtrl()` 路径还调用逆动力学和摩擦力矩计算，再写入控制器的 `LowlevelCmd`。LOWCMD 则由应用提供前馈力矩和增益，不能假定控制器会替应用补齐一套高层轨迹或相同的动力学前馈。设备 IO 最后统一将关节命令交给 MCU。

控制器和 SDK 各自都有名为 `CtrlComponents`、`LowlevelCmd`、`LowlevelState` 的类型，但它们属于不同侧的组件，存在于不同进程，靠 UDP 交换数据，不共享内存。控制器侧还有滤波、错误检查、碰撞检测相关入口；模板配置的 `collision/open=N`，且本项目没有双臂互碰规划。

### 3.4 用两条完整链路理解责任划分

#### 调用一次 MoveJ

```text
应用给出末端 posture、maxSpeed，可选 gripperPos
  → SDK 组织 MOVEJ 状态和轨迹参数
  → 第一段 UDP：SendCmd 到达 ARMSDK
  → 控制器进入 State_MoveJ，利用模型与 JointSpaceTraj 生成关节参考
  → 每个控制周期生成 q / dq / tau / kp / kd
  → IOUDP 转为 JointCmd 数组，发给 MCU
  → MCU 返回关节反馈与设备状态
  → 控制器更新状态，并用 FK 计算末端位姿
  → ARMSDK 回送 RecvState
  → SDK 更新 lowstate，应用据此读取状态或结束等待
```

一个高层函数调用，对应的是控制器持续执行的一段运动，而不是一个包发完就结束。

#### 应用自行执行 LOWCMD

参考 [`lowcmd_development.cpp`](examples/lowcmd_development.cpp)，通信所有权按下面的顺序交接：

```text
启动 SDK 后台收发
  → 请求 PASSIVE，再请求 LOWCMD，并检查切换结果
  → 设置六轴及夹爪增益
  → 停止 SDK 后台收发，改由应用周期循环接管
  → 每周期：
       使用最近反馈
       生成相互一致的 q / qd / qdd
       按需要计算 tau
       更新 arm.q / arm.qd / arm.tau 及夹爪参考
       setArmCmd() / setGripperCmd()
       sendRecv()
       等待本周期剩余时间
  → 收尾时恢复后台收发，完成所需状态切换，再停止通信
```

示例的 `inverseDynamics(q, qd, 0, 0)` 使用零目标加速度和零末端外力；若要按完整轨迹做模型前馈，应传入对应的 `qdd`。它不能代替应用的轨迹边界处理，也不会自动补偿所有未建模效应。

### 3.5 两个周期、一个通信写入者

这条链路并非所有层都运行在 500 Hz：

| 循环 | 当前周期参数 | 职责 |
| --- | --- | --- |
| SDK 收发 | `0.002 s`，目标 500 Hz | 应用与控制器交换命令和反馈 |
| 控制器 ARMSDK 通信 | `0.002 s`，目标 500 Hz | 收取 SDK 请求、回送控制器反馈 |
| 控制器 FSM / 设备控制 | `1.0 / 250 s = 0.004 s`，目标 250 Hz | 状态执行及 MCU 周期 IO |

控制器的两种周期均可在 `main.cpp` 中看到：`ctrlComp->dt = 1.0/250.`，而 `ARMSDK` 使用 `0.002`。所以 SDK 多收发一次，不等于 MCU 多执行一个新控制周期；MCU 内部电机控制频率也不能由这两个参数推算。

[`LoopFunc` / `Timer`](include/unitree_arm_sdk/utilities/loop.h) 在回调后等待周期剩余时间，超时则不再休眠。这些是目标周期，实际间隔受计算、收发与调度影响。

同一个 SDK 实例应只有一个活动的收发执行者。使用自建周期循环时先停后台线程，不能让两者同时调用 `sendRecv()`；跨线程规划或 UI 输入应通过同步后的目标和反馈快照交接。公开命令字段本身没有完整的线程同步机制。

`RecvState` 没有包序号或接收时间戳；读取 `lowstate` 得到的是最近缓存，不能据此认定反馈刚刚更新。`loopOff()` / `shutdown()` 只停止通信线程，不发送 PASSIVE，也不等同于机械臂停止；退出动作应在通信仍有效时明确完成。

### 3.6 双臂适配与可继续追踪的实现位置

当前左右臂各运行一套完整链路：独立 SDK 对象、独立 `z1_ctrl` 进程、独立 UDP 端口及配置目录。两套实例提供独立控制，不自动提供同步采样、协调轨迹或双臂避碰。

[`dual_arm/`](dual_arm) 只负责端口、附件模型、额外负载、工作目录和进程管理。`has_gripper` 决定 SDK 使用的模型，控制器侧则由设备握手识别附件；两侧定义应与实物一致。启动器为每臂保存独立的位姿和示教文件，不实现另一套轨迹、动力学或控制状态机。

需要继续追代码时，按责任查找：

- **应用接口与组包入口**：[`unitreeArm.h`](include/unitree_arm_sdk/control/unitreeArm.h)、[`ctrlComponents.h`](include/unitree_arm_sdk/control/ctrlComponents.h)。
- **两段协议**：SDK 的 [`arm_common.h`](include/unitree_arm_sdk/message/arm_common.h) 与控制器的 [`arm_common.h`](controller/include/message/arm_common.h)。后者额外定义 MCU 的 `UDPSendCmd`、`UDPRecvState` 和旧版反馈。
- **控制器调度与控制**：[`main.cpp`](controller/main.cpp)、[`FSM/`](controller/include/FSM)、[`trajectory/`](controller/include/trajectory)、[`IOUDP.h`](controller/include/interface/IOUDP.h)。
- **双臂初始化与运行隔离**：[`sdk.hpp`](dual_arm/include/z1_official/sdk.hpp)、[`z1_official.py`](dual_arm/scripts/z1_official.py)。

接口、消息结构、启动流程和线程工具可直接阅读源码。SDK 的主要方法由 `lib/libZ1_SDK_*.so` 实现；控制器的状态执行、轨迹、模型和设备 IO 主要由 `controller/lib/libZ1_*.so` 实现，头文件没有展开所有算法细节。上游版本记录见 [`UPSTREAM.md`](UPSTREAM.md)。
