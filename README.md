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

设备连接与各自的控制方法见 [Dingo](src/dingo/README.md)、[Pan-Tilt](src/pan_tilt_ros/README.md)和 [D455](src/d455_camera/README.md)。当前仓库没有 `unidingo_bringup` 或 `whole_body_example` 包，整机联合启动和遥操作应在后续编排中接入这些组件入口。

## 代码与文档

| 路径 | 内容 |
| --- | --- |
| `src/z1_ros/` | Z1 真机控制、消息、启动及模型 |
| `src/dingo/` | Dingo 底盘驱动与控制 |
| `src/pan_tilt_ros/` | 云台驱动、消息及启动 |
| `src/d455_camera/` | D455 相机 nodelet 驱动 |
| `src/unidingo_description/` | 组合机器人描述与网格 |

- [机械臂运行逻辑](src/z1_ros/README.md)
- [机械臂接口契约](src/z1_ros/INTERFACES.md)
- [机械臂需求基线](src/z1_ros/REFACTOR_REQUIREMENTS.md)
- [机械臂采用独立控制内核和 ROS 1 接口](docs/adr/0001-arm-core-and-ros1-interface.md)
- [保留官方 MCU 通信适配](docs/adr/0002-retain-z1-mcu-transport.md)
- [Z1 运行入口仅面向真机](docs/adr/0003-z1-hardware-only-runtime.md)
