# Z1 重构实施记录

记录日期：2026-10-11。已完成本轮约定的软件实现与离线验证；尚未完成实际 Z1 的调试和验收。本记录区分已运行的测试与仍需现场确认的设备行为。

## 1. 已交付

| 模块 | 交付内容 |
| --- | --- |
| `z1_control` | ROS 无关 C++17 内核；Pinocchio FK、雅可比、有限/差分 IK、RNEA、ABA；Ruckig 参考生成；Coal 模型距离和轨迹检查 |
| 执行控制 | 有限关节/末端目标、两类流式目标、笛卡尔阻抗；实测到达判定；模式切换、受控停止、故障锁存与显式恢复 |
| 通信 | 官方 IOUDP 的握手和编解码；补充可观测 UDP 接收、来源/长度检查、测量时龄、积压上限和 V2 电机状态检查 |
| ROS 1 | 九种自定义消息及单臂 topic 节点；两臂使用同一组件的独立进程，无全身 API |
| 启动与模型 | 单臂/双臂 launch；逐臂配置、模型、描述、IP、端口；关节及 TF 前缀；模型配置指纹 |
| 仿真 | 同一内核接 Pinocchio ABA 仿真；默认失能；独立示例程序，不会自动改连真机 |
| 文档 | 简洁 README、完整 topic 契约、当前需求基线和本实施记录 |

未引入力传感器、外力估计、接触力闭环或整机遥操。末端有限目标是 IK 后的关节运动；流式位姿采用局部差分 IK，不承诺直线、全局可达或绕障。固定安装结构支持配置盒状几何；另一臂和动态环境未纳入本轮模型。

## 2. 旧链路删除

已删除 `z1_controller`、`z1_sdk`、`z1_hw`、两套 MoveIt 配置、`z1_examples`、`z1_rviz`、`unitree_ros` 和旧控制说明/启动文件。原描述网格及供整机描述使用的 xacro 常量迁入 `z1_description`。必要 MCU 头文件、许可证及 x86_64/aarch64 官方动态库集中保留在 `vendor/unitree`。

新进程不运行旧 FSM、不使用 SDK 本机 UDP 中转、不依赖 MoveIt/ros_control/Gazebo 控制链路。其余组件中引用旧机械臂 launch 的整机入口留给后续整机重构；本轮没有改写其他组件。

## 3. 实施中修正的契约

- 官方 `sendRecv()` 的布尔值不能证明收到新状态；通过真实套接字事件增加反馈序号。相同测量值的新报文仍是新反馈。
- V2 的状态字段直接从有效接收报文检查，避免公开温度数组被连接字段覆盖的问题；V1 缺失的诊断不伪造为可用。
- 有限轨迹检查整段与关节解析极值；实际生成的停止轨迹也检查。Ruckig 0.9.2 零时长段未初始化的中间值不参与极值判断，静止流式目标不会因此被错误拒绝。
- 在线检查加入参考步的运动包络、测量与参考偏差、候选参考的制动路径和查询预算。查询耗尽预算按失败处理。
- 流式目标保留源时间戳对应的剩余有效期；有效的稍早目标不会因控制线程并发推进而被全部拒绝。会话初值随机生成，停止/恢复更新会话，旧 token 和缓存不能恢复运动。
- 非有限测量、过期测量、超限速度及设备错误按无效反馈处理；ROS 的 `feedback_valid` 使用内核的完整判断，避免无效测量被标成有效并发布旧 TCP。
- 主关节 MCU Kp/Kd 为零，位置控制由上位机计算力矩，动力学补偿只计入一次。主臂力矩限幅及限变化率；阻抗停止清除旧目标并生成制动参考。
- ROS 节点先完成模型/参数验证，再初始化通信。正常退出尝试有时限的受控停止及最终被动报文，不声称获得固件停止确认。
- 安装后的可执行程序和 MCU 适配库设置运行库搜索路径，官方库在构建及实际加载时检查 SHA-256。

## 4. 已运行的验证

本机 Linux x86_64、GCC 11.4、CMake 3.22、Eigen 3、Pinocchio 4.0.0、Coal 3.0.3、Ruckig 0.9.2、OpenSSL 3。原生算法依赖来自本机已有的 `/opt/ros/humble`，控制内核未使用 ROS 2 通信接口。

本机没有完整 ROS 1 发行版。ROS 1 验证使用 Ubuntu Jammy 的 roscpp/rospy 1.15.14、catkin 0.8.10、robot_state_publisher 1.15.2 及消息/运行依赖，解包到 `/tmp` 后编译和运行；xacro 展开使用本机已有版本，通过临时入口供 ROS 1 launch 调用。未安装或替换系统软件。通过该测试不等于已验证任意 Noetic 工作区或部署机。

| 验证 | 结果及覆盖 |
| --- | --- |
| 独立 Release 构建 | 通过：控制内核、MCU 适配、standalone、两个 CTest 程序 |
| `arm_core` | 通过：FK/IK、不可达目标、RNEA/ABA 重力一致性、端点安全但中段碰撞的拒绝、有限目标、参考 v/a/jerk、力矩变化率、两类流式和阻抗、时效/会话、无效反馈、实例隔离、参数先验验证 |
| `vendor_transport_contract` | 通过：实际官方动态库与纯内存端口；确认失败返回值缺口；不创建网络套接字 |
| `mcu_udp_loopback` | 通过：真实 localhost UDP 和官方编解码；V1/V2、六/七电机、握手、原始反馈、静止新包、丢包、错误长度/来源、电机错误/过温/断连、端口冲突和真机门槛 |
| 安装后执行 | 通过：安装目录 standalone 完成小幅关节目标并报告实测成功；安装后的 MCU 库完成回环及加载指纹验证 |
| ROS 1 构建 | 通过：九种消息生成 C++/Python/Lisp；真实 roscpp topic 节点编译链接 |
| `ros_topics_test.py` | 通过：隔离 master、两个独立仿真进程、命名空间/关节前缀、有限目标、NaN 拒绝、两类连续目标、阻抗、超时停止、显式恢复和新会话；一臂退出另一臂继续反馈 |
| `ros_launch_test.py` | 通过：实际 dual_arm.launch 启动四个 ROS 1 节点；两臂默认失能且反馈有效，分别加载 robot_description，robot_state_publisher 发布左右前缀的 TF |
| 模型与 XML | 通过：package/launch/model XML；实际 xacro 左右前缀展开；显示模型与控制 URDF 一致；网格路径存在 |

ROS topic 测试退出时，临时 master 对已经退出的订阅者可能报告 `ConnectionRefusedError`；控制测试断言和退出码通过。仿真没有接触模型或真实 MCU 看门狗，其失能状态使用理想制动。aarch64 动态库已保留并校验指纹，本轮未在该架构编译运行。

## 5. 复现

独立构建命令见 [README.md](README.md)，官方返回值探针见 [tools/transport_preflight/README.md](tools/transport_preflight/README.md)。在正常 ROS 1 工作区构建并 source 后，可运行：

```bash
python3 src/z1_ros/z1_control/tests/ros_topics_test.py \
  /path/to/workspace/devel/lib/z1_control/z1_topic_node \
  src/z1_ros/z1_description/urdf/z1_control.urdf

python3 src/z1_ros/z1_control/tests/ros_launch_test.py \
  src/z1_ros/z1_bringup/launch/dual_arm.launch
```

第一条需将节点路径替换为实际 catkin devel/install 可执行程序路径；测试创建临时 master，只启动仿真，不接管现有机器人。第二条需要可发现的 Z1 包、ROS 1 robot_state_publisher、xacro 及 tf2_msgs。

## 6. 真机待验收

UDP 启动默认 `hardware_validated=false`，明确验收后才由现场人员启用。以下无法通过当前离线环境完成：

1. 确认固件版本、原始 q/dq/tau 的单位与轴方向、主关节零增益/力矩执行语义，以及设备是否已经进行其他补偿。
2. 测量工具/夹爪/负载质量、质心与惯性，核实 TCP 及完整张合包络。默认工具 0.25 kg 和简化碰撞几何为待核实的工程初值。
3. 按实际安装填写基座重力、支架/底盘 `fixed_boxes`，审查碰撞排除对和半径界。默认固定结构为空，不提供未建模的支架防护。
4. 验证实际 250 Hz 收发、新反馈率、时延、最坏计算耗时及调度抖动；必要时调整周期和预算，不能把仿真成功当作硬实时保证。
5. 逐步调试位置控制及阻抗增益、力矩限制，测量跟踪误差、实测停止距离、输入/反馈中断及正常退出的设备行为。
6. 核实 MCU 失联看门狗、被动命令、机械制动和硬件急停。反馈失效时软件不能保证可靠的闭环减速。

部署接口和参数规则见 [INTERFACES.md](INTERFACES.md)；本轮范围见 [REFACTOR_REQUIREMENTS.md](REFACTOR_REQUIREMENTS.md)。
