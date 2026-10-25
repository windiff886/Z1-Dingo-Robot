# Z1 运行入口仅面向真机

状态：已确认，2026-10-25。

`roslaunch z1_bringup arm.launch` 启动左右两个机械臂控制进程，通过官方 MCU UDP 接口连接真实设备。两臂通信参数统一由 `z1_bringup/config/communication.yaml` 提供。

连接或握手失败时退出并报告原因。连接成功后处于 Disabled，通过 ENABLE 进入 Holding。

控制算法和通信的软件契约继续由合成反馈及仅绑定本机地址的 MCU 协议夹具验证。测试夹具不安装到 SDK，不作为 ROS 启动模式，不计算被控机械臂的动力学。真实的跟踪、停止、负载和阻抗性能由真机测量确认。

已有仿真实验输出保留为历史记录，不能作为当前功能说明或真机验收依据。当前运行说明以 [README](../../src/z1_ros/README.md)、[接口契约](../../src/z1_ros/INTERFACES.md)和[需求基线](../../src/z1_ros/REFACTOR_REQUIREMENTS.md)为准。
