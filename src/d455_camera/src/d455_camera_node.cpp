/**
 * @file d455_camera_node.cpp
 * @brief D455 相机独立可执行节点包装器（Standalone Node Wrapper）
 * 
 * 功能定位：
 * 本文件编译生成独立可执行程序 `d455_camera_node`。
 * 允许用户无需 launch 文件或外部 Nodelet Manager，直接通过 `rosrun d455_camera d455_camera_node` 启动相机。
 * 
 * 核心原理：
 * 采用 ROS 官方推荐的 `nodelet::Loader` 模式，在当前独立进程中动态载入 `D455CameraNodelet` 插件。
 * 命名空间通过 `std::map` 重映射表绑定为 C++ 变量 `kNamespace`，无需对 argc/argv 进行数组篡改。
 */

#include <nodelet/loader.h>
#include <ros/ros.h>

#include <map>
#include <string>

// ============================================================================
// 命名空间配置：若需更改相机的命名空间，直接在此处修改 C++ 变量值即可
// ============================================================================
const std::string kNamespace = "/d455";

int main(int argc, char **argv) {
  // 1. 使用 std::map 键值对重映射表指定命名空间，避免对 argc/argv 做指针数组操作
  std::map<std::string, std::string> remappings;
  remappings["__ns"] = kNamespace;

  // 2. ROS 节点初始化（使用支持 map 的官方重载接口，节点名注册为 "d455_camera_node"）
  ros::init(remappings, "d455_camera_node");

  // 3. 在当前进程内加载 Nodelet 插件
  nodelet::Loader loader;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;

  // 动态加载 D455CameraNodelet 插件并自动调用其 onInit() 函数
  if (!loader.load(ros::this_node::getName(), "d455_camera/D455CameraNodelet", remap, nargv)) {
    ROS_ERROR("Failed to load d455_camera nodelet.");
    return 1;
  }

  // 4. 事件主循环：阻塞等待 Ctrl+C 信号，后台子线程持续采集与发布图像
  ros::spin();
  return 0;
}
