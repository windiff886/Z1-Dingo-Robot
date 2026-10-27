# 双臂机械零点标定

`z1_calibrate` 是独立 C++ 维护程序，直接调用项目随附的宇树 `IOUDP::calibration()`，将当前机械姿态设为零位参考。程序读取 YAML，支持只标定左臂、只标定右臂，以及按顺序标定两臂。

机械零位由操作者定位：按官方零位姿态摆放机械臂，尤其对齐 J1、J6 的刻线。操作说明见[宇树零位说明](https://dev-z1.unitree.com/brief/poweron.html)和[官方 Calibration 状态实现](https://github.com/unitreerobotics/z1_ros/blob/d3bafd38fb48e805096da7abb0c19a15b636975d/z1_controller/include/FSM/State_Calibration.h)。

## 使用

1. 在启动正常控制器或 RViz 整机入口的终端按 `Ctrl+C`，停止两个机械臂控制器。Disabled 状态仍持续占用 UDP 端口，因此需要退出进程。
2. 支撑机械臂，将其摆到规定的机械零位。
3. 在已构建并加载环境的 ROS 1 工作空间中执行：

   ```bash
   source devel/setup.bash
   rosrun z1_control z1_calibrate
   ```

4. 程序显示所选机械臂的 IP 和本地端口，并连接全部所选机械臂。任意一臂连接失败时，两臂均不发送标定请求。
5. 左臂摆正、保持静止后，输入 `left ZERO` 并回车。程序检查静止反馈，调用官方标定接口，等待约 2 秒，读取回执并检查新的六轴零位反馈。
6. 左臂成功后，按提示输入 `right ZERO` 标定右臂。任意步骤失败或取消时，停止后续标定并输出逐臂结果。

输入其他文字或结束输入会取消尚未执行的标定。`Ctrl+C` 也会停止后续操作；若已进入官方阻塞调用，需要等待其返回才能退出。程序始终发送被动控制帧，不发送使能或运动目标，结束后也不会自动启动正常控制器。

## 配置

标定选项位于 [z1_control/config/calibration.yaml](z1_control/config/calibration.yaml)。两臂 IP、本地 UDP 端口及接收预算继续复用 [z1_bringup/config/communication.yaml](z1_bringup/config/communication.yaml)，避免重复维护设备地址。

| 选项 | 默认值 | 含义 |
| --- | --- | --- |
| `communication_config` | `../../z1_bringup/config/communication.yaml` | 相对标定 YAML 的通信配置路径 |
| `arms` | `[left, right]` | 本次标定对象，也可设为 `[left]` 或 `[right]` |
| `sample_count` | `20` | 标定前、标定后需要连续通过的反馈样本数 |
| `sample_interval_ms` | `10` | 被动反馈采样间隔，单位 ms |
| `feedback_timeout_ms` | `2000` | 每次反馈检查的最长等待时间，单位 ms |
| `reply_timeout_ms` | `200` | 官方约 2 秒等待结束后的回执接收超时，单位 ms |
| `max_joint_speed` | `0.02` | 六个主关节的静止速度阈值，单位 rad/s |
| `max_position_drift` | `0.01` | 连续采样窗口中的最大关节位置漂移，单位 rad |
| `zero_tolerance` | `0.05` | 标定后六个主关节距离零位的允许误差，单位 rad |

修改 YAML 后重新执行相同命令。默认配置根据程序所在位置查找，适用于源码构建和安装目录，不要求从指定工作目录启动。使用另一份配置文件时可执行 `z1_calibrate /absolute/path/calibration.yaml`，运行选项仍由该 YAML 提供。

## 结果

| 结果 | 含义 |
| --- | --- |
| `SUCCEEDED` | 收到 MCU 成功回执，且新的六轴反馈满足静止和零位容差 |
| `DEVICE_REJECTED` | MCU 返回标定错误 |
| `UNKNOWN` | 请求已进入发送流程，但缺少有效成功/错误回执，不能确定设备结果 |
| `ACKNOWLEDGED_UNVERIFIED` | MCU 已确认标定，但反馈验证失败或在验证前被中断 |
| `NOT_ATTEMPTED` | 未发出标定请求，包括取消、连接失败和标定前检查未通过 |

所有所选机械臂均为 `SUCCEEDED` 时退出码为 `0`；失败、取消、中断和部分完成时退出码为 `1`。程序不会自动重复标定，也不会撤销另一臂已完成的标定。

通信层同时检查回执来源及长度，拒绝错误源端口、错误长度和未知回执。每个连接最多执行一次标定。正常控制循环使用的接收预算在标定调用结束后恢复。
