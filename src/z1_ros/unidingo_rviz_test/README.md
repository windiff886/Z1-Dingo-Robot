# RViz 整机末端目标测试

在一份整机模型中显示 Dingo 底盘、左右 Z1、云台和 D455，拖动左右 TCP 的六自由度手柄，松开鼠标后发送一次末端目标位姿。实际机器人模型跟随关节反馈更新；手柄表示目标。这里没有 MoveIt 规划器或目标姿态的虚拟关节预览。

本包是 ROS 1 功能包。代码与离线验证可在无 ROS 1 的电脑上完成；下文的启动、Topic 和真机验收命令供部署机使用，本次没有执行。

## 当前机械臂控制链路

```text
RViz InteractiveMarkerFeedback（目标在 base_link 中）
  -> interactive_tcp_test.py：TF 转换、检查状态、生成递增 command_id
  -> /left_arm/pose_goal 或 /right_arm/pose_goal（z1_msgs/PoseGoal）
  -> z1_topic_node：校验 frame_id、时间戳、单位四元数
  -> z1_core：以当前参考关节角为种子求局部 IK
  -> Ruckig：生成关节运动轨迹 + 单臂几何路径检查
  -> Runtime：默认 500 Hz，计算关节参考和力矩
  -> McuTransport：官方 libUnitreeArm 的 IOUDP/LowLevelCmd
  -> 各臂 MCU

MCU -> joint_states / tcp_pose / status / result
    -> RViz 实际机器人模型、初始目标位置和状态文字
```

这里的 SDK 执行入口是官方底层通信库。位姿 IK、轨迹、动力学及反馈 PD 在主机的 `z1_core` 中实现。主臂 MCU 的位置 Kp/Kd 为零，由主机发送 q/dq/tau；连接后处于 Disabled，通过 ENABLE 进入保持状态。

控制器的 `pose_goal` 只在 Holding 接收一个有限目标。内部 IK 失败、超出限位、几何路径检查失败都会 REJECTED；ACCEPTED 只表示已经接受，SUCCEEDED 还要等待实际关节和 TCP 到位且稳定。末端到目标之间沿关节轨迹运动，这个入口没有指定末端沿直线行进。

RViz 鼠标反馈不是稳定的周期流，因此本测试选择松手发送有限目标。它不会向 `pose_stream` 持续供帧；拖动过程中不发运动目标，运动中再次松手也不排队。要做拖动时实时跟随，需要单独按现有 PoseStream 的会话、sequence、20–100 Hz 持续供帧和显式恢复契约开发。

## 目标相对谁，控制哪个点

当前 ROS 适配器严格比较 `header.frame_id == joint_prefix + "link00"`，不隐式查 TF：

| 内容 | 左臂 | 右臂 |
| --- | --- | --- |
| 控制 namespace | `/left_arm` | `/right_arm` |
| 控制器接受的位姿参考系 | `left_link00` | `right_link00` |
| 默认控制 TCP 对应的整机 link | `left_tool` | `right_tool` |
| RViz 手柄参考系 | `base_link` | `base_link` |
| 目标 Topic | `/left_arm/pose_goal` | `/right_arm/pose_goal` |

默认控制模型 `z1_description/urdf/z1_control.urdf` 只有六个标量关节，以 `link00` 为根，`tool_frame: tool`。TCP 通过固定变换连接到 `link06`：

```text
T(link06 -> tool): xyz = (0.051, 0, 0) m, rpy = (0, 0, 0)
```

所以目标位置是这个固定工具坐标原点的位置，目标 quaternion 是工具坐标轴相对该臂基座的姿态。它不是 `link06` 的原点，也不是夹爪活动指尖或两指之间另行定义的抓取中心。夹爪开合不改变默认 TCP。整机模型已增加同样的 `left_tool` 和 `right_tool`；它们与夹爪静态安装点重合。

整机模型的固定安装变换目前为：

```text
base_link -> leftarm_link -> left_link00
  xyz = (0.15091,  0.11708, 0.57228) m, rpy = (0, 0, 0)

base_link -> rightarm_link -> right_link00
  xyz = (0.15091, -0.11708, 0.57228) m, rpy = (0, 0, 0)
```

两臂都是相同朝向，右臂没有额外镜像或旋转。发送节点使用 TF 做完整的刚体位姿转换：

```text
T(arm_base -> target) = inverse(T(base_link -> arm_base)) * T(base_link -> target)
```

在当前零旋转安装配置中，位置分别减去上面的安装偏移，姿态保持相同。例如在 `base_link` 中的 `(0.45091, 0.11708, 0.77228)`，转换到左臂基座是 `(0.3, 0, 0.2)`。这仅用于说明坐标计算，不保证该示例位姿连同任意姿态可达。

手柄从当前有效 `/.../tcp_pose` 转换到 `base_link` 后初始化，发送时再转换回该臂基座，并填入新的 ROS 时间戳。TF 查询使用最新变换，因为这两个基座与 `base_link` 固定连接；本入口的目标也随底盘运动，不能直接解释为 `map` 或 `odom` 中固定的目标。

当前控制配置 `gravity: [0, 0, -9.81]` 同样表达于各臂基座。当前安装 rpy 为零时与底盘方向一致；若真机安装有旋转，除了模型安装变换，也需要重新核对各臂重力参数。若修改了控制器的 `tool_frame` 或工具安装偏移，必须同步修改整机显示模型；本次一致性检查针对仓库的默认模型与参数。

## ROS 1 部署机上的使用方法

在包含 ROS 1 的部署工作区构建并加载 `devel/setup.bash`。本包运行依赖 `rviz`、`interactive_markers`、`tf2_ros`、`tf2_geometry_msgs`、`robot_state_publisher`、`xacro` 和本项目的 Z1 消息/控制/描述包，具体见 `package.xml`。

默认只接入已经运行的控制器，不连接 MCU：

```bash
roslaunch unidingo_rviz_test rviz_tcp_test.launch
```

由测试入口一并启动双臂：

```bash
roslaunch unidingo_rviz_test rviz_tcp_test.launch start_arms:=true
```

也可以分开启动双臂与电脑端可视化。双臂入口需要关闭独立单臂描述发布：

```bash
# 机器人端
roslaunch z1_bringup arm.launch publish_description:=false

# 电脑端，与机器人端使用同一 ROS_MASTER_URI，网络配置确保相互可达
roslaunch unidingo_rviz_test rviz_tcp_test.launch
```

整机 `robot_state_publisher` 统一发布本测试中的机器人 TF。`publish_description` 在机械臂入口中默认 true，本测试统一发布模型时设为 false。

若同时使用其他组件驱动，Dingo 应使用 `load_description:=false publish_imu_tf:=false`，避免覆盖整机 `robot_description` 或重复发布 `imu_link` 的父变换；云台入口保持默认 `pub_urdf:=false`。相机驱动只发布图像，光学帧由本整机模型提供。

界面操作：

1. 等待左右手柄出现；只有对应臂的 `status.feedback_valid` 为 true，status 和 TCP 的源时间戳及接收时龄都小于 300 ms，并且 TF 可用时才初始化。不会用假定的零关节姿态初始化目标。
2. 右键对应末端的球形手柄，选择 `ENABLE`，等待文字变为 Holding。
3. 拖动平移箭头或旋转圆环，松开鼠标后发送一个目标。默认 `speed_scale=0.15`。左、右臂分别操作。
4. 通过 RViz 状态文字和 `/.../result` 查看 ACCEPTED、SUCCEEDED 或拒绝原因。运动中再次松手会被本节点拒绝，目标不排队，也不会在运动结束后补发。
5. `STOP` 请求受控停止，等待 AwaitResume；选择 `HOLD after stop` 回到 Holding，再 `Reset target to measured TCP` 后重新拖动。故障先解决原因，再选 `Acknowledge FAULT`，等待 Disabled 后重新 ENABLE。

如希望拖动只编辑目标，由菜单显式发送：

```bash
roslaunch unidingo_rviz_test rviz_tcp_test.launch send_on_release:=false
```

其他参数：`rviz:=false` 仅关闭 GUI 启动；`arm_config` 指定双臂通信 YAML，默认使用 `z1_bringup/config/communication.yaml`；`aux_joint_states` 默认 `/joint_states`，用于接入底盘/云台状态。使能、恢复和新目标发送由用户操作触发。

观察命令示例（部署机使用）：

```bash
rostopic echo /left_arm/pose_goal
rostopic echo /left_arm/status
rostopic echo /left_arm/result
rostopic echo /left_arm/tcp_pose
```

只允许一个上层程序拥有每臂的命令发布权，避免与其他目标发送器竞争递增 ID 和控制状态。本节点等待本次目标的最终 result 才能继续发送；如果该结果因连接中断丢失，不会推测成功或自动重发。应先停止并确认控制状态，再重新启动测试发送器、将目标重置到当前反馈。

## 整机显示与碰撞范围

- 整机 Xacro 原先调用了未定义的 `realsense_sensor`。现在使用本包已有描述资源目录中的 `d455_visual.urdf.xacro`，提供相机外壳示意与驱动使用的 `d455_color_optical_frame`；外壳尺寸和光学中心为示意，不能用作测量或手眼标定。
- 底盘、云台和相机包含在模型中，本入口不启动它们的硬件驱动，也不向它们发送命令。底盘坐标固定为 `base_link`；已有云台反馈可通过 `/joint_states` 更新模型。
- 机械臂六个关节在状态和 TCP 有效、新鲜时转发到 `/rviz_tcp_test/joint_states`，保留源 Topic 的 ROS 发布时间戳（不是 MCU 硬件采样时刻），不合成关节角或反复重打时间戳。反馈断开后停止转发，RViz 可能保留上一次姿态，状态文字会提示 `FEEDBACK STALE / INVALID`，并拒绝新目标。
- 缺少反馈的轮子与云台显示零位；收到实际辅助关节反馈后使用该值。两个可视化夹爪默认显示固定零位，因为 `gripper_motor` 到 `jointGripper` 的换算尚未标定，不能直接将二者等同。
- `z1_core` 仍为独立单臂控制：控制 URDF 必须是六关节模型，不能将完整机器人 URDF 直接作为 `model_file`。完整模型显示在 RViz 中不会自动加入现有碰撞检查。
- 默认几何检查覆盖该臂与工具的自碰撞，`fixed_boxes` 可加入该臂基座系中的固定结构。默认配置的 `fixed_boxes` 为空，**没有双臂互碰、底盘、支撑柱、云台、相机或外界的完整碰撞检查，也没有绕障规划**。本功能验证目标发送与现有执行链路，整机碰撞控制需要另行实现。
- 两臂安装位置、工具和相机外参仍需要按实际设备校准；模型之间数值一致不等于已经完成真机标定。

## 本次验证与边界

以下检查不使用 ROS master、不启动节点、不打开 RViz、不连接 MCU：

```bash
# 仅 Python 标准库，消息和 ROS API 使用内存替身
python3 src/z1_ros/unidingo_rviz_test/tests/test_sender_offline.py -v

# 使用已有 xacro 和 numpy；通过仓库内包路径映射展开，无需 ROS 1
python3 src/z1_ros/unidingo_rviz_test/tests/test_model_contract.py -v
```

发送逻辑 12 项测试通过，覆盖双臂目标转换接口与 frame_id、时间戳、松手单次发送、手动发送、状态与新鲜度限制、无订阅者/无 TF、非法四元数、待处理目标限制、结果匹配、无自动重试、控制菜单与关节反馈转发。

模型 5 项测试通过：默认模型展开为 45 个 link / 44 个 joint；所有 link 连通且网格在仓库中存在；4 种云台/夹爪开关组合均能展开；两臂分别以 25 组随机关节角核对整机 FK 与默认控制模型；安装变换、关节轴/限位、工具偏移和相机帧命名一致。

**本次尚未验证** ROS 1 catkin 构建、真实消息序列化和 Topic 通信、tf2 的运行行为、RViz 显示和鼠标事件、SDK/MCU 联调及真机运动。离线 API 替身测试不代表这些运行检查已通过。部署机验收需依次观察：完整模型/TF 正常、目标正确转换到各臂基座、Disabled 无运动、ENABLE 后 Holding、有效小幅目标的 ACCEPTED 与 SUCCEEDED、忙碌/断反馈时拒绝发送、STOP 后 AwaitResume 及显式 HOLD 恢复。

交互控件 API 参考：[ROS InteractiveMarker 示例](https://docs.ros.org/en/melodic/api/interactive_marker_tutorials/html/simple__marker_8py_source.html)。
