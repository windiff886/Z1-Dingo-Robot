# 双臂机械零点标定

`z1_calibrate` 是独立 C++ 维护程序，直接调用项目随附的宇树 `IOUDP::calibration()` 发送官方零点标定请求，再独立验证六轴关节反馈。MCU 成功回执不保证每一轴的反馈都已满足零位容差。程序读取 YAML，支持只标定左臂、只标定右臂，以及按顺序标定两臂。

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
5. 默认先标定右臂。右臂摆正、保持静止后，输入 `right ZERO` 并回车。程序检查静止反馈，调用官方标定接口，等待约 2 秒，读取回执并检查新的六轴零位反馈。
6. 右臂成功后，按提示输入 `left ZERO` 标定左臂。任意步骤失败或取消时，停止后续标定并输出逐臂结果。执行顺序由 YAML 中的 `arms` 列表顺序决定。

输入其他文字或结束输入会取消尚未执行的标定。`Ctrl+C` 也会停止后续操作；若已进入官方阻塞调用，需要等待其返回才能退出。程序始终发送被动控制帧，不发送使能或运动目标，结束后也不会自动启动正常控制器。

需要先验证握手、反馈和静止状态而不写入零位时，执行 `rosrun z1_control z1_calibrate --check-only`。此模式检查所有所选机械臂，不接受标定确认、不发送标定请求；全部检查通过时退出码为 `0`，各臂仍显示 `NOT_ATTEMPTED`，并注明 `passive feedback check passed`。它不验证操作者是否摆到了正确机械零位。

## 配置

标定选项位于 [z1_control/config/calibration.yaml](z1_control/config/calibration.yaml)。两臂 IP 和本地 UDP 端口复用 [z1_bringup/config/communication.yaml](z1_bringup/config/communication.yaml)，避免重复维护设备地址。标定的接收预算独立配置为 5 ms；正常控制循环当前配置为 4 ms 周期、3 ms 接收预算。当前双臂被动反馈回复实测约 2.5 ms。

| 选项 | 默认值 | 含义 |
| --- | --- | --- |
| `communication_config` | `../../z1_bringup/config/communication.yaml` | 相对标定 YAML 的通信配置路径 |
| `arms` | `[right, left]` | 按列表顺序标定，也可设为 `[left, right]`、`[left]` 或 `[right]` |
| `sample_count` | `20` | 标定前、标定后需要连续通过的反馈样本数 |
| `sample_interval_ms` | `10` | 被动反馈采样间隔，单位 ms |
| `receive_budget_us` | `5000` | 标定被动反馈的接收预算，单位 μs，必须小于采样间隔 |
| `feedback_timeout_ms` | `2000` | 每次反馈检查的最长等待时间，单位 ms |
| `reply_timeout_ms` | `200` | 官方约 2 秒等待结束后的回执接收超时，单位 ms |
| `max_joint_speed` | `0.02` | 官方滤波速度和相邻位置反馈差分速度的静止阈值，单位 rad/s |
| `max_position_drift` | `0.01` | 连续采样窗口中的最大关节位置漂移，单位 rad |
| `zero_reference` | `[0, 0, 0, -0.07435, 0, 0]` | 标定后 J1～J6 的反馈检查中心，单位 rad；必须包含六个有限数值 |
| `zero_tolerance` | `0.05` | 标定后每个主关节相对 `zero_reference` 的允许误差，单位 rad |

当前按[官方 home 姿态](https://github.com/unitreerobotics/z1_ros/blob/d3bafd38fb48e805096da7abb0c19a15b636975d/z1_moveit_config/config/z1_description.srdf)设置检查中心，验证公式为 `abs(q[j] - zero_reference[j]) <= zero_tolerance`。因此 J4 的允许区间为 `[-0.12435, -0.02435] rad`，其他五轴仍为 `[-0.05, 0.05] rad`。这是本次调试采用的姿态参考，不代表已核实固件内部的标定规则；更改只影响标定后判定，不修改原始反馈、运动学坐标或发往 MCU 的标定命令。程序启动时打印参考值，标定通过后打印实际反馈及相对参考的误差。

修改 YAML 后重新执行相同命令。默认配置根据程序所在位置查找，适用于源码构建和安装目录，不要求从指定工作目录启动。使用另一份配置文件时可执行 `z1_calibrate /absolute/path/calibration.yaml`，运行选项仍由该 YAML 提供。

标定前检查分别报告接收超时、非有限数值、速度超限和电机状态异常。`MCU motor fault` 会列出首个异常电机的关节编号、温度、原始错误字节及连接状态。错误位按[随附官方版本的协议头文件](https://github.com/unitreerobotics/z1_ros/blob/d3bafd38fb48e805096da7abb0c19a15b636975d/z1_controller/include/IOInterface/IOUDP/msgUdpToSerial.h)及 `LowLevelState::checkError()` 识别 `0x01/0x02/0x04/0x20`，另检查过温和连接异常；`0x40` 在这版官方库中不触发电机错误。当前、旧版整机和官方的 x86_64 通信库 Git blob 哈希均为 `0a001a81d4baee265e228cf1ebe35d2cc375260e`。更换库或固件后应重新核对状态语义。

静止检查保留原始六轴位置和速度的有限值检查，速度阈值使用官方 `LowLevelState::getDqFiltered()` 提供的滤波结果，同时按实际接收时间计算相邻位置变化速度，防止滤波掩盖真实运动。窗口内的位置漂移、连续有效样本数和标定后的零位容差仍须全部满足；原始速度噪声不会通过直接放宽速度阈值处理。

## 结果

| 结果 | 含义 |
| --- | --- |
| `SUCCEEDED` | 收到 MCU 成功回执，且新的六轴反馈满足静止和零位容差 |
| `DEVICE_REJECTED` | MCU 返回标定错误 |
| `UNKNOWN` | 请求已进入发送流程，但缺少有效成功/错误回执，不能确定设备结果 |
| `ACKNOWLEDGED_UNVERIFIED` | MCU 已确认标定，但反馈验证失败或在验证前被中断 |
| `NOT_ATTEMPTED` | 未发出标定请求，包括取消、连接失败和标定前检查未通过 |

所有所选机械臂均为 `SUCCEEDED` 时退出码为 `0`；失败、取消、中断和部分完成时退出码为 `1`。程序不会自动重复标定，也不会撤销另一臂已完成的标定。

出现 `ACKNOWLEDGED_UNVERIFIED` 且提示 `zero_tolerance` 时，结果会列出超差关节的原始值、参考值、误差和最后的六轴反馈。先保持姿态并用 `--check-only` 只读复核，不要把 MCU 的成功打印等同于全部关节已满足参考容差。若姿态未变、重新连接后的有效反馈仍偏离配置参考，应核对实际机械姿态、参考值及设备固件的标定行为；不能仅凭某轴非零判断失败。某臂保持 `NOT_ATTEMPTED` 表示本次未向该臂发送标定请求。

通信层同时检查回执来源及长度，拒绝错误源端口、错误长度和未知回执。每个连接最多执行一次标定。正常控制循环使用的接收预算在标定调用结束后恢复。
