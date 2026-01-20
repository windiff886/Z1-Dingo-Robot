# dingo_base/base.launch 说明

本文档说明 `roslaunch dingo_base base.launch` 启动内容、参数来源与组件位置。

## 文件位置

- 启动文件：`src/dingo/dingo_base/launch/base.launch`
- 工作区内包路径：`dingo_base` 对应 `src/dingo/dingo_base`

## 启动命令

```bash
roslaunch dingo_base base.launch
```

## 参数与环境变量

- `motor_params`：从环境变量 `DINGO_MOTOR_PARAMS` 读取，默认 `base`；用于选择电机参数 YAML 文件名。
- `teleop`：从环境变量 `DINGO_TELEOP` 读取，默认 `joystick`；传递给遥操作 launch。
- `DINGO_OMNI`：是否全向底盘（`1` 为全向，`0` 为差速）；影响底盘参数与诊断配置。
- `DINGO_WIRELESS_INTERFACE`：无线网卡名，默认 `wlp2s0`。
- `DINGO_MAG_CONFIG`：磁场配置文件路径，可覆盖默认磁配置 YAML。

## 启动组件与位置（按顺序）

1) 机器人模型描述  
- 组件：`<include file="$(find dingo_description)/launch/description.launch" />`  
- 位置：`src/dingo/dingo_description/launch/description.launch`  
- 作用：加载机器人描述（URDF、TF 等）。

2) rosserial 消息信息服务  
- 组件：`<node pkg="rosserial_python" type="message_info_service.py" name="rosserial_message_info" />`  
- 位置：外部 ROS 包 `rosserial_python`（非本仓库内文件）  
- 作用：提供 rosserial 消息信息服务。

3) 底盘驱动节点  
- 组件：`<node pkg="dingo_base" type="dingo_node" name="dingo_node" />`  
- 位置：`dingo_base` 包内的 `dingo_node` 可执行文件  
- 关键参数：
  - `dingo_omni`：`$(optenv DINGO_OMNI 0)`
  - `wireless_interface`：`$(optenv DINGO_WIRELESS_INTERFACE wlp2s0)`
- 配置文件位置：`src/dingo/dingo_base/config/$(arg motor_params).yaml`  
  - 默认实际路径：`src/dingo/dingo_base/config/base.yaml`

4) IMU 滤波（Madgwick）  
- 组件：`<node pkg="imu_filter_madgwick" type="imu_filter_node" name="imu_filter" />`  
- 位置：外部 ROS 包 `imu_filter_madgwick`  
- 磁场配置文件位置：  
  - 默认：`src/dingo/dingo_base/config/mag_config_default.yaml`  
  - 覆盖：`DINGO_MAG_CONFIG` 指定任意 YAML 路径  
- 关键参数（launch 内固定）：`gain=0.1`，`zeta=0.001`，`publish_tf=false`，`use_magnetic_field_msg=true`

5) 底盘控制与基础定位  
- 组件：`<include file="$(find dingo_control)/launch/control.launch" />`  
- 位置：`src/dingo/dingo_control/launch/control.launch`  
- 作用：差速控制与基础定位相关启动。

6) 遥操作与交互控制  
- 组件：`<include file="$(find dingo_control)/launch/teleop.launch">`  
- 位置：`src/dingo/dingo_control/launch/teleop.launch`  
- 入参：`teleop=$(arg teleop)`，支持 `joystick` 或 `keyboard`，详见下文。

7) 诊断聚合（按底盘类型分支）
- 组件：`diagnostic_aggregator/aggregator_node`  
- 位置：外部 ROS 包 `diagnostic_aggregator`  
- 公共配置：`src/dingo/dingo_base/config/diagnostic_analyzers_common.yaml`  
- 分支配置（由 `DINGO_OMNI` 决定）：  
  - 全向底盘：`src/dingo/dingo_base/config/diagnostic_analyzers_omni.yaml`  
  - 差速底盘：`src/dingo/dingo_base/config/diagnostic_analyzers_diff.yaml`

## teleop.launch 详解

### 文件与入口

- 启动文件：`src/dingo/dingo_control/launch/teleop.launch`
- 主要用途：提供摇杆/键盘遥操作与 RViz 交互式标记控制
- 控制模式：`teleop` 参数（`joystick`/`keyboard`），由 `base.launch` 传入

### 参数与环境变量

- `teleop`：默认 `joystick`，用于决定是否进入键盘分支
- `joy_dev`：`DINGO_JOY_DEV`，默认 `/dev/input/ps4`
- `joy_config`：`DINGO_JOY_CONFIG` 可覆盖；否则按 `DINGO_OMNI` 选择  
  - 全向底盘：`src/dingo/dingo_control/config/teleop_omni.yaml`  
  - 差速底盘：`src/dingo/dingo_control/config/teleop_diff.yaml`
- `keyboard_speed`：`DINGO_KEYBOARD_SPEED`，默认 `0.4`
- `keyboard_speed_turbo`：`DINGO_KEYBOARD_SPEED_TURBO`，默认 `2.0`
- `keyboard_turn`：`DINGO_KEYBOARD_TURN`，默认 `1.4`

### 交互式标记服务器（始终启用）

- 节点：`interactive_marker_twist_server/marker_server`，名称 `twist_marker_server`
- 配置文件（按 `DINGO_OMNI` 分支）：  
  - 全向底盘：`$(find interactive_marker_twist_server)/config/planar.yaml`  
  - 差速底盘：`$(find interactive_marker_twist_server)/config/linear.yaml`
- 位置：外部 ROS 包 `interactive_marker_twist_server`

### Joystick 模式（默认）

- 条件：`teleop != keyboard`，命名空间 `bluetooth_teleop`
- 配置：`rosparam` 加载 `$(arg joy_config)`
- 关键参数：`joy_node/dev=$(arg joy_dev)`
- 启动节点：  
  - `joy/joy_node`（手柄输入）  
  - `teleop_twist_joy/teleop_node`（速度指令）
- 默认映射摘要（来自配置文件）：  
  - 差速：`axis_linear=1`，`axis_angular=0`，`scale_linear=0.4`，`scale_linear_turbo=2.0`，`scale_angular=1.4`
  - 全向：`axis_linear.x=1`、`axis_linear.y=0`，`axis_angular.yaw=3`，`scale_linear.x/y=0.4`，`scale_linear_turbo.x/y=2.0`，`scale_angular.yaw=1.4`
  - 公共：`enable_button=4`，`enable_turbo_button=5`，`joy_node.deadzone=0.1`，`joy_node.autorepeat_rate=20`
- 相关配置文件位置：  
  - `src/dingo/dingo_control/config/teleop_diff.yaml`  
  - `src/dingo/dingo_control/config/teleop_omni.yaml`

### Keyboard 模式（当前为占位）

- 条件：`teleop == keyboard`
- 说明：launch 中的 `dingo_control/simple_keyboard_teleop.py` 节点配置被注释，默认不启动；如需键盘遥控，请在终端手动启动对应节点并传入速度参数（以实际安装包为准）。

## 条件逻辑说明

- 当 `DINGO_OMNI=1`：底盘驱动参数 `dingo_omni` 为真，诊断配置使用 `diagnostic_analyzers_omni.yaml`。
- 当 `DINGO_OMNI=0` 或未设置：诊断配置使用 `diagnostic_analyzers_diff.yaml`。

## 小结

`base.launch` 负责串联机器人模型、底盘驱动、IMU 滤波、控制、遥操作与诊断，并通过环境变量与 YAML 文件实现差异化配置与组件选择。
