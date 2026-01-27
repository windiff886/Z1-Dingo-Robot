# 双臂 Gazebo 启动与安全位姿逻辑说明

本文档描述 `dual_arm_gazebo_moveit.launch` 与 `set_dual_arm_safe_pose.py` 当前实现的**严格串行 + 重试**启动流程，以及“确认安全位姿后再 unpause”的控制逻辑。

## 相关文件

- 启动文件：`src/unidingo_description/launch/dual_arm_gazebo_moveit.launch`
- 逻辑脚本：`src/unidingo_description/scripts/set_dual_arm_safe_pose.py`

## 总体目标

1. 模型在 Gazebo 中以安全姿态初始化。
2. 控制器按顺序加载并启动。
3. 只有当安全姿态确认到位后才解除 Gazebo 物理暂停。
4. 若任一步失败（strict=true），立即停止，避免进入不一致状态。

## 启动流程（严格顺序）

以下步骤由 `set_dual_arm_safe_pose.py` 严格按顺序执行：

1) **读取参数**
   - 包括安全位姿、控制器列表、超时、是否严格、是否预先 unpause 等。

2) **创建发布器**
   - `/left_arm/left_z1_joint_traj_controller/command`
   - `/right_arm/right_z1_joint_traj_controller/command`

3) **（可选）Spawn 模型**
   - 检查模型是否存在（`/gazebo/get_model_state`）。
   - 若 `delete_existing_model=true`，先删除旧模型。
   - 读取 `/robot_description` 并调用 `/gazebo/spawn_urdf_model`。
   - 失败且 `strict=true` → 直接退出。

4) **（可选）Set Model Configuration**
   - 调用 `/gazebo/set_model_configuration` 直接将关节设置到安全位姿。
   - 失败且 `strict=true` → 直接退出。

5) **（可选）预先 Unpause（仅为让 controller_manager 响应）**
   - 条件：`unpause=true`、`unpause_after_set_model=false`、`pre_unpause_for_controllers=true`。
   - 目的：让 `/controller_manager/*` 服务正常响应。
   - 失败且 `strict=true` → 直接退出。

6) **加载控制器（严格串行 + 重试）**
   - 按 `controllers_to_load` 列表逐个加载：
     - 先 `list_controllers` 判断是否已存在。
     - 若不存在，`load_controller`。
     - 再次 `list_controllers` 确认可见。
   - 任一失败且 `strict=true` → 退出。

7) **启动控制器（严格串行 + 重试）**
   - 按 `start_controllers` 列表逐个启动：
     - `list_controllers` 读取状态。
     - 非 `running` 则 `switch_controller` 启动。
     - 再次确认状态为 `running`。
   - 任一失败且 `strict=true` → 退出。

8) **等待控制器订阅**
   - 等 `/command` 有订阅者。
   - 若无订阅者且 `strict=true` → 退出。

9) **发布安全位姿并确认到位**
   - 反复发布 `JointTrajectory`。
   - 读取 `/state`，使用 `actual`（优先）或 `desired` 判断是否在容差 `tol` 内。
   - 超过 `max_wait` 仍未到位：
     - `strict=true` → 退出；
     - 如果之前预先 unpause 且 `pause_on_failure=true`，则自动 pause。

10) **最终 Unpause（确认安全位姿后）**
   - 条件：`unpause=true` 且 `unpause_after_set_model=false` 且安全位姿已确认。
   - 若失败且 `strict=true` → 退出。

## 为什么需要“预先 unpause”

当 Gazebo 处于 pause 状态时，`/controller_manager/list_controllers` 可能无法响应（服务挂在 /gazebo 上但回调队列靠 Gazebo 更新驱动），导致查询阻塞。

因此逻辑中加入 **“预先 unpause”**：

- 仅用于让 controller_manager 服务可用。
- 安全位姿未确认前仍然不会“最终 unpause”。
- 若安全位姿确认失败且 `pause_on_failure=true`，会自动再次 pause。

## 关键参数（launch 已设置）

- `strict`：失败即退出
- `retry_interval`：重试间隔
- `delete_existing_model`：是否删除旧模型后再 spawn
- `unpause_after_set_model=false`：**确认安全位姿后再 unpause**
- `pre_unpause_for_controllers=true`：控制器加载前短暂 unpause
- `pause_on_failure=true`：安全位姿失败后自动 pause
- `left_positions` / `right_positions`：当前默认 home 姿态
- `stow_only=true`：**自动 stow 模式**，不再调用 `set_model_configuration`，仅在控制器启动后发送 home 轨迹

## 常见现象与判断

- 日志出现 `Safe pose accepted by controllers.`：安全位姿确认成功。
- 日志出现 `Timed out waiting for arms to reach safe pose.`：控制器未到位或 /state 未更新。
- `/controller_manager/list_controllers` 卡住：多半是 Gazebo pause 或 gazebo_ros_control 未正常工作。

## 调试建议（快速定位）

- `rosservice call /controller_manager/list_controllers "{}"`
- `rostopic echo -n1 /left_arm/left_z1_joint_traj_controller/state`
- `rostopic echo -n1 /right_arm/right_z1_joint_traj_controller/state`
- `rostopic echo -n1 /joint_states`

若 state 里出现 `nan` 或 controller 非 `running`，会导致安全位姿确认失败。

---

如需调整流程或参数，请修改上述两个文件。必要时可提高 `controller_wait`、`max_wait` 或 `state_timeout` 以适应较慢的启动过程。
