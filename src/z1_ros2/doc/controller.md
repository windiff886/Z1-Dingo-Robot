# Z1 Controller（ROS2）

本目录对应 ROS1 `src/z1_ros/doc/controller.md` 的 ROS2 迁移说明。

当前 ROS2 控制链保持以下 ROS1 风格参数名不变：

- `udp.mcu_ip`
- `udp.port_to_mcu`
- `udp.port_to_sdk`
- `udp_to_controller/controller_ip`
- `udp_to_controller/own_port`

其中：

- `z1_controller_ros2` 负责 MCU 通信与 FSM 控制
- `z1_hardware` 负责 `ros2_control` 硬件接口
- `z1_bringup` 负责将两者拼装成真机启动链
