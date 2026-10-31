# 双臂机械零点标定

机械臂已经摆到机械零位，需要把这个位置写入 MCU 时，运行 `z1_calibrate`。程序逐臂确认，调用随附宇树库的 `IOUDP::calibration()`，再用新的六轴反馈检查标定结果。

机械零位按[宇树零位说明](https://dev-z1.unitree.com/brief/poweron.html)摆放，J1、J6 两侧刻线应对齐。程序使用被动帧采样，不提供姿态保持，摆放和标定期间需要托住机械臂。

## 1. 执行标定

1. 退出双臂控制器或整机 RViz 入口，释放 UDP 端口。`DISABLE` 会停止主动输出，但进程仍占用端口，需要在启动终端按 `Ctrl+C` 退出。
2. 托住机械臂，摆到规定的机械零位。
3. 在已构建的 ROS 1 工作区执行：

   ```bash
   source devel/setup.bash
   rosrun z1_control z1_calibrate
   ```

4. 程序打印地址并连接全部所选机械臂；连接失败时结束本次操作，尚未发送标定请求。
5. 默认先标右臂。摆正并保持静止，输入 `right ZERO` 回车。程序检查静止反馈，调用标定接口，等待约 2 s，读取回执，再验证新的六轴反馈。
6. 右臂成功后，输入 `left ZERO` 标左臂。任一步失败或取消都会停止后续操作，并输出逐臂结果。

输入其他文字或结束输入会取消后续标定。`Ctrl+C` 也会中断后续操作；已经进入的官方阻塞调用需等待返回。结束后，手动重新启动正常控制器。

### 先检查通信和静止状态

```bash
rosrun z1_control z1_calibrate --check-only
```

此模式检查所有所选机械臂的握手、被动反馈和静止状态，不写零位。全部通过时退出码为 `0`，结果显示 `NOT_ATTEMPTED` 和 `passive feedback check passed`，并打印六轴角度。机械零位姿态仍由操作者对照刻线确认。

## 2. 选择机械臂和配置

配置文件为 [calibration.yaml](z1_control/config/calibration.yaml)。`arms` 列表决定对象和顺序：`[right, left]`、`[left, right]`、`[left]` 或 `[right]`。地址和本地 UDP 端口从 [communication.yaml](z1_bringup/config/communication.yaml) 读取。

修改 YAML 后重新运行。需要使用另一份配置时执行：

```bash
rosrun z1_control z1_calibrate /absolute/path/calibration.yaml
```

默认配置按可执行文件的安装位置查找，源码构建使用编译时记录的配置路径。

| 选项 | 默认值 | 含义 |
| --- | --- | --- |
| `communication_config` | `../../z1_bringup/config/communication.yaml` | 相对标定 YAML 的通信配置路径 |
| `arms` | `[right, left]` | 标定对象与执行顺序 |
| `sample_count` | `20` | 标定前后各需连续通过的反馈样本数 |
| `sample_interval_ms` | `10` | 每次被动交换后的等待时间，ms；实际采样间隔还包含收发耗时 |
| `receive_budget_us` | `5000` | 被动采样接收预算，μs；小于 `sample_interval_ms × 1000` |
| `feedback_timeout_ms` | `2000` | 每轮反馈检查的最长等待，ms |
| `reply_timeout_ms` | `200` | 官方约 2 s 等待结束后的回执接收超时，ms |
| `max_joint_speed` | `0.02` | 滤波速度和相邻位置差分速度的静止阈值，rad/s |
| `max_position_drift` | `0.01` | 连续采样窗口内的最大关节位置漂移，rad |
| `zero_reference` | `[0, 0, 0, -0.07435, 0, 0]` | 六个有限角度，作为标定后反馈的检查中心，rad |
| `zero_tolerance` | `0.05` | 各轴相对检查中心的允许误差，rad |

标定使用独立的 5 ms 接收预算，正常 ROS 控制器使用 4 ms 周期、3 ms 接收预算。标定回执接收结束后，传输层恢复调用前的预算，即本工具默认的 5 ms。

整数参数按 `unsigned` 读取，`sample_count` 至少为 2，其余采样与等待参数为正数。配置还需满足 `sample_count × sample_interval_ms < feedback_timeout_ms` 和接收预算小于采样间隔；`reply_timeout_ms × 1000` 需能由传输接口的 `unsigned` 微秒数表示。采样数量与等待时间由这些关系和类型容量决定。

## 3. 零位反馈怎样判定

默认检查中心采用[宇树 SRDF 的 Home 关节值](https://github.com/unitreerobotics/z1_ros/blob/d3bafd38fb48e805096da7abb0c19a15b636975d/z1_moveit_config/config/z1_description.srdf)。逐轴判定为：

```text
abs(q[j] - zero_reference[j]) <= zero_tolerance
```

因此 J4 的允许区间为 `[-0.12435, -0.02435] rad`，其余五轴为 `[-0.05, 0.05] rad`。`zero_reference` 决定本项目的标定后验收中心；修改它只改变反馈判定，MCU 标定请求和运动学坐标保持不变。

采样检查六轴位置、速度的有限性，用官方 `getDqFiltered()` 结果和按接收时间计算的位置差分速度判断静止，同时检查位置漂移。标定后的采样再增加零位容差条件。成功时打印实际角度和相对参考的误差。

V2 反馈出现电机异常时，`MCU motor fault` 日志列出首个异常电机的关节编号、温度、错误字节和连接状态。当前适配按随附[协议头文件](vendor/unitree/include/IOInterface/IOUDP/msgUdpToSerial.h)检查 `0x01/0x02/0x04/0x20` 错误位、温度达到 80 °C 和连接异常；`0x40` 在这版库中未归为电机错误。V1 未提供这些电机状态字段。更换库或固件时，需要重新核对字段含义。

## 4. 读结果并处理异常

| 结果 | 含义与下一步 |
| --- | --- |
| `SUCCEEDED` | MCU 成功回执与新的静止零位反馈均通过，可结束该臂标定 |
| `DEVICE_REJECTED` | MCU 返回标定错误，检查设备状态 |
| `UNKNOWN` | 已进入请求发送流程，但未取得有效回执；保持姿态，先读取反馈并确认设备结果，再决定是否重试 |
| `ACKNOWLEDGED_UNVERIFIED` | MCU 已确认，反馈验证失败或被中断；按日志核对超差关节、姿态和参考值 |
| `NOT_ATTEMPTED` | 尚未发送请求，原因包括取消、连接失败、前置采样未通过或 `--check-only` |

标定模式下，所有所选臂均为 `SUCCEEDED` 时退出码为 `0`；失败、取消、中断或部分完成时为 `1`。程序在每个连接上最多尝试一次标定，重试与另一臂的处理由操作者决定。

出现零位超差时，日志会列出原始值、参考值、误差和最后的六轴反馈。保持姿态，运行 `--check-only` 重新读取角度；该模式不自动比较 `zero_reference`，需要对照上表的允许区间。重新连接后的有效反馈仍超差时，检查机械姿态、配置参考和固件标定行为。

## 5. 脱离 ROS 构建

安装 CMake、C++17 编译器、Eigen3、yaml-cpp 和随附库所需的 Boost 头文件后，在仓库根目录执行：

```bash
cmake -S src/z1_ros/z1_control -B /tmp/z1-calibration-build \
  -DZ1_BUILD_ROS=OFF -DZ1_BUILD_CORE=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/z1-calibration-build --target z1_calibrate -j2
/tmp/z1-calibration-build/z1_calibrate --check-only
```

去掉最后一条命令中的 `--check-only` 即进入交互标定。支持 Linux x86_64 和 aarch64，动态库位于 [vendor/unitree](vendor/unitree)。
