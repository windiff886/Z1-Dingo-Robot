# Z1-Dingo-Robot

Dingo 全向移动底盘、双 Unitree Z1 机械臂、Pan-Tilt 云台及 Intel D455 相机的 ROS 集成工作区。各组件通过各自的 ROS Topic 接口接收命令和发布反馈。

当前按实际设备分别启动组件。Z1 运行入口仅连接真机 MCU；现场确认固件、控制方式、模型和参数后才能接管，启动成功后仍需显式使能。机械臂从启动到执行的完整流程见 [Z1 README](src/z1_ros/README.md)。

## 组件启动

先构建并 source ROS 1 工作区：

```bash
source devel/setup.bash
```

机械臂完成现场验收后启动：

```bash
# 双臂：/left_arm、/right_arm
roslaunch z1_bringup dual_arm.launch hardware_validated:=true

# 或只启动单臂，IP 和本地端口按实际设备填写
roslaunch z1_bringup arm.launch hardware_validated:=true \
  mcu_ip:=192.168.123.110 local_port:=8881
```

`hardware_validated` 默认 false，未确认时在连接前拒绝启动。机械臂没有后端选择参数。双臂默认 IP 为 `192.168.123.110/111`，本地端口为 `8881/8882`，可逐臂覆盖。

其余组件分别启动：

```bash
# Dingo 底盘
roslaunch dingo_control dingo.launch

# Pan-Tilt 云台
roslaunch pan_tilt_bringup panTilt_bringup.launch

# Intel D455 相机
roslaunch d455_camera d455_camera_nodelet.launch
```

设备连接与各自的控制方法见 [Dingo](src/dingo/README.md)、[Pan-Tilt](src/pan_tilt_ros/README.md)和 [D455](src/d455_camera/README.md)。各组件仍按独立接口运行。

## RViz 末端目标测试

[unidingo_rviz_test](src/z1_ros/unidingo_rviz_test/README.md) 提供包含双臂、底盘、云台和相机的完整模型，通过拖动 TCP 手柄、松开鼠标发送末端目标。目标从 `base_link` 经 TF 转换到对应的 `left_link00` / `right_link00`，进入原有机械臂控制链路。该入口不自动使能，也不提供双臂互碰或整机绕障规划。

详细坐标系分析、ROS 1 部署机启动步骤及离线验证边界见该包说明。本次未在本机启动 ROS 1 或连接硬件。

## 代码与文档

| 路径 | 内容 |
| --- | --- |
| `src/z1_ros/` | Z1 真机控制、消息、启动及模型 |
| `src/dingo/` | Dingo 底盘驱动与控制 |
| `src/pan_tilt_ros/` | 云台驱动、消息及启动 |
| `src/d455_camera/` | D455 相机 nodelet 驱动 |
| `src/unidingo_description/` | 组合机器人描述与网格 |
| `src/z1_ros/unidingo_rviz_test/` | 整机 RViz 显示、TCP 拖动目标发送及离线验证 |

- [机械臂运行逻辑](src/z1_ros/README.md)
- [机械臂接口契约](src/z1_ros/INTERFACES.md)
- [机械臂需求基线](src/z1_ros/REFACTOR_REQUIREMENTS.md)
- [机械臂采用独立控制内核和 ROS 1 接口](docs/adr/0001-arm-core-and-ros1-interface.md)
- [保留官方 MCU 通信适配](docs/adr/0002-retain-z1-mcu-transport.md)
- [Z1 运行入口仅面向真机](docs/adr/0003-z1-hardware-only-runtime.md)
