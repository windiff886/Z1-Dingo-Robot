# RViz 双臂启动与末端目标测试

本包用于启动并测试左右 Z1 机械臂，在 RViz 中提供左右 TCP 的六自由度交互手柄。用户拖动手柄设置目标位姿，由 Z1 SDK 控制对应机械臂运动，模型根据双臂关节反馈更新。底盘、云台和 D455 作为静态安装参考显示，测试期间这些组件的驱动不运行。

## 唯一启动入口

本包的唯一启动文件是 [launch/rviz_tcp_test.launch](launch/rviz_tcp_test.launch)。在已构建并加载环境的 ROS 1 工作区中执行：

```bash
source devel/setup.bash
roslaunch unidingo_rviz_test rviz_tcp_test.launch
```

启动命令无需附加参数。此命令同时启动左右机械臂控制器、连接各自的真实 MCU，并启动整机显示和目标发送节点。运行选项从 YAML 读取，由本入口统一发布整机模型和 TF。即使独立机械臂配置中的 `publish_description` 设为 `true`，本入口也不会启动单臂模型或 TF 发布器。

双臂通信参数来自 [communication.yaml](../z1_bringup/config/communication.yaml)，需按实际设备配置 IP 和本地端口。连接成功后机械臂处于 Disabled；首条有效目标从实测位置直接开始执行，无需先通过菜单使能。

当前控制周期为 250 Hz，UDP 接收预算为 3 ms，实测依据和适用范围见 [通信节拍说明](../COMMUNICATION_TIMING.md)。

几何距离仅作诊断；已删除固定净空对使能、运动、恢复和制动的强制拦截，以及对应开关和阈值。正常折叠贴合可以直接保持和运动。关节位置、速度/加速度/jerk、输出力矩及变化率、反馈有效性、电机故障和受控制动约束保留。详见 [控制约束审核](../SAFETY_AUDIT.md)。

## 这个启动文件做了什么

### 1. 加载整机模型

通过 Xacro 展开 [uni_dingo_dual_arm.urdf.xacro](../../unidingo_description/urdf/uni_dingo_dual_arm.urdf.xacro)，将结果写入 `/robot_description`。模型包含底盘、左右机械臂及夹爪、云台和 D455；启动文件固定启用 `UnitreeGripper=true` 和 `PanTilt=true`。

整机显示以 `base_link` 为固定参考系。底盘、云台和相机仅显示模型，本测试只启动双臂控制器。

### 2. 根据反馈发布整机 TF

启动 `/whole_robot_state_publisher`，读取 `/robot_description` 和 `/rviz_tcp_test/joint_states`，发布各连杆的 TF。

交互节点仅订阅左右臂的反馈，将有效、新鲜的六轴关节位置转发到 `/rviz_tcp_test/joint_states`，并保留源消息时间戳。

轮子、云台和可视化夹爪固定使用零位显示，以补全静态模型的 TF。夹爪电机反馈与模型夹爪关节之间没有直接换算。机械臂手柄需要有效的状态、TCP 反馈及 TF 才能初始化。

### 3. 启动 TCP 目标交互节点

启动 `/rviz_tcp_test`，运行 [interactive_tcp_test.py](scripts/interactive_tcp_test.py)。该节点订阅各臂的 `joint_states`、`tcp_pose`、`status` 和 `result`，在当前实测 TCP 处创建目标手柄，并显示控制状态及命令结果。

左右手柄均在 `base_link` 中编辑。发送时，节点通过 TF 将目标转换到对应机械臂基座，并发布 `z1_msgs/PoseGoal`：

| 内容 | 左臂 | 右臂 |
| --- | --- | --- |
| 显示模型中的 TCP | `left_tool` | `right_tool` |
| 控制器接收的目标参考系 | `left_link00` | `right_link00` |
| 目标 Topic | `/left_arm/pose_goal` | `/right_arm/pose_goal` |
| 控制操作 Topic | `/left_arm/control` | `/right_arm/control` |

默认 TCP 相对各臂 `link06` 沿 X 正方向平移 0.051 m，姿态相同。目标沿用 SDK 的有限位姿运动接口：先求终点逆解，再执行关节轨迹；TCP 中间路径由该关节轨迹决定。

拖动编辑目标，松开鼠标提交一次目标；右键菜单 `Send target once` 可再次提交。发送节点只负责目标格式和 TF 转换，不重复拦截控制器状态或等待旧结果回执。控制器统一判断反馈、模式、IK、关节轨迹和力矩约束：Disabled 下有效且静止时直接执行首条目标，运动中提交会拒绝，Fault 和 AwaitResume 仍需先处理对应状态。具体接口见 [INTERFACES.md](../INTERFACES.md)。

### 4. 打开 RViz

启动 RViz，加载 [rviz/tcp_test.rviz](rviz/tcp_test.rviz)，显示网格、随反馈更新的整机模型、TCP 目标手柄和控制状态文字。

手柄表示期望目标，机器人模型表示反馈状态。完整整机模型用于显示，SDK 的几何距离诊断按单臂模型及配置的固定结构计算，显示模型本身不会增加双臂互碰或整机绕障能力。

### 5. 启动双臂控制器

启动文件复用 SDK 的内部 [controllers.launch](../z1_bringup/launch/controllers.launch)，启动 `/left_arm/controller` 和 `/right_arm/controller`，读取 SDK 的通信和控制 YAML。该内部文件只启动控制器，整机入口始终独占模型和 TF 发布职责。

两臂分别加载 SDK 控制配置、连接各自 MCU，并发布关节、TCP、控制状态与运动结果。左右臂的使能、目标和停止分别管理。

## 界面操作

控制器启动后保持 Disabled，直到收到明确目标或 ENABLE；打开界面不会自行建立保持力矩。首次有限目标要求反馈有效且各轴速度小于 `velocity_tolerance`，不要求任何几何间距。

显示的 TCP、逆解和几何距离都基于当前反馈角，绝对末端精度需要独立验证。TCP 目标使用关节空间轨迹，手柄的小幅位移不保证每个关节都只移动小角度。

1. 等待对应臂的 TCP 手柄出现；初始化需要有效反馈及 TF。状态文字会提示 `FEEDBACK STALE / INVALID`。
2. 拖动平移箭头或旋转圆环，松开鼠标提交目标，默认速度比例为 `0.05`。无需先 ENABLE。`ACCEPTED` 表示计划被接受，`SUCCEEDED` 表示实测到达并稳定。
3. 需要只保持时，可选择 `ENABLE (Disabled -> Holding)`；需要停止时选择 `STOP`。
4. 进入 AwaitResume 后，选择 `HOLD after stop` 回到 Holding，再通过 `Reset target to measured TCP` 重置手柄。
5. Fault 原因处理后，选择 `Acknowledge FAULT`；进入 Disabled 后可直接提交新目标。`DISABLE (passive output)` 用于失能。

## YAML 配置

运行参数统一从 YAML 读取。修改配置后，重新运行同一条无参数启动命令即可。

| 文件 | 内容 |
| --- | --- |
| [config/rviz_tcp_test.yaml](config/rviz_tcp_test.yaml) | 速度比例和显示反馈时效 |
| [z1_bringup/config/communication.yaml](../z1_bringup/config/communication.yaml) | 两臂 MCU IP、本地 UDP 端口和接收预算；`publish_description` 仅用于独立启动机械臂，默认 `false` |
| [z1_bringup/config/control.yaml](../z1_bringup/config/control.yaml) | SDK 控制周期、运动限制、增益和几何诊断配置 |

测试配置加载到 `/rviz_tcp_test` 的私有参数空间：

```yaml
speed_scale: 0.05
telemetry_timeout: 0.3
```

`speed_scale` 为有限位姿目标的速度比例，范围 `(0,1]`；`telemetry_timeout` 单位为秒，用于手柄初始化、实测位置重置及显示状态的时效判定；命令接受由控制器使用自身反馈判断。启动后修改这些值，需要重启测试节点才生效。

本包的 ROS 依赖见 [package.xml](package.xml)。
