# Z1 SDK RViz 拖动测试

在 RViz 中显示整机模型（底盘、双臂、云台、相机），拖动每条机械臂末端的橙色球形手柄，经官方 Z1 SDK 让机械臂运动。FK、IK、`MoveJ`、`MoveL`、`backToStart`、`setFsm` 都直接调用官方 Python 接口，本包不包含控制算法。

## 准备（ROS 1 Noetic 部署机）

1. 按 [z1_sdk/README.md](../z1_sdk/README.md) 用 `-DZ1_BUILD_PYTHON=ON` 编译带端口参数的 `unitree_arm_interface`，默认路径为 `/tmp/z1-build/dual_arm`。
2. 在另一个终端启动官方控制器：

   ```bash
   /usr/bin/python3 /tmp/z1-build/bin/z1_official.py controllers
   ```

3. 构建并启动：

   ```bash
   catkin_make && source devel/setup.bash
   roslaunch z1_rviz_drag z1_rviz_drag.launch            # 双臂
   roslaunch z1_rviz_drag z1_rviz_drag.launch arms:=[left]
   ```

本节点为每条臂占用 `z1_official.py client` 使用的同一把锁，因此不能与该交互客户端同时控制同一条臂。

## 操作

- 拖动箭头 / 圆环编辑目标；状态文字显示 `IK ok` 或 `NO IK`（官方 IK 结果）。
- 默认松开鼠标即以 `movej_speed`（0.5 rad/s）发送 `MoveJ`。设 `send_on_release:=false` 后，只能用右键菜单发送。
- 右键菜单包括：Send MoveJ、Send MoveL、Reset target to measured、backToStart (home) 和 PASSIVE。
- 每条臂基座后方有蓝色 **HOME** 按钮，点击即调用官方 `backToStart()`，让该臂回到控制器的 home 位姿。
- 没有编辑目标时，手柄跟随实测 TCP，模型关节来自 SDK 反馈；底盘轮子和云台保持零位。

## 坐标说明

手柄位于各臂 `left_link00` / `right_link00` 坐标系，位姿就是官方 SDK 的 `posture`，末端点也由官方模型 `forwardKinematics(q, 6)` 定义。它与 URDF 中显示用的 `tool` 坐标系不一定重合。首次上机时请先确认：手柄在实测状态下与模型末端对齐，且臂基座坐标系与 SDK 一致。
