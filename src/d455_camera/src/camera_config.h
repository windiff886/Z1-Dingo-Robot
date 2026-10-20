#pragma once

#include <string>

namespace d455_camera {

/**
 * @brief D455 相机核心配置结构体
 * 
 * 所有相机配置均以纯 C++ 变量的形式组织，直接作为类成员在程序中生效。
 * 如需调整参数，可直接在此结构体中修改对应字段的初始值。
 * 
 * 【系统环境前提说明】
 * 本驱动严格采用 Librealsense 的 GLOBAL_TIME 协议（将相机硬件时间戳关联至主机物理系统时钟），
 * 因此系统运行环境必须为真实物理时钟，不得启用 ROS 仿真时钟（即 /use_sim_time 必须为 false）。
 */
struct CameraConfig {
  /**
   * @brief 相机硬件序列号 (Serial Number)
   * 规则：
   * - 为空字符串 "" 时：自动枚举并绑定系统检测到的首个 D455 设备，并在运行期锁定该设备；
   * - 传具体字符串（如 "141322251234"）时：精准绑定指定硬件，未检测到则报错退出。
   */
  std::string serial_no = "";

  /**
   * @brief 数据流使能开关
   * 规则：
   * - enable_color 和 enable_depth 均必须为 true；
   * - 本驱动基于 SDK rs2::align 实现深度图向彩色视角的空间对齐，不支持单流运行模式。
   */
  bool enable_color = true;
  bool enable_depth = true;

  /**
   * @brief 彩色图像流规格
   * 规则：
   * - color_width 与 color_height：必须为正整数，D455 常用组合为 640x480、1280x720 等；
   * - color_fps：采样帧率，必须为正整数，D455 常用帧率为 15、30、60；
   * - 编码格式固定为 BGR8。
   */
  int color_width = 640;
  int color_height = 480;
  int color_fps = 30;

  /**
   * @brief 深度图像流规格
   * 规则：
   * - depth_width 与 depth_height：必须为正整数，常用为 640x480、848x480、1280x720 等；
   * - depth_fps：采样帧率，必须为正整数，D455 常用帧率为 15、30、60、90；
   * - 强烈建议将 depth_fps 与 color_fps 设为一致（如均为 30 FPS），以保证空间对齐的时间一致性，避免动态场景重影；
   * - 编码格式固定为 16UC1（单位：毫米，0 表示无效值）。
   */
  int depth_width = 640;
  int depth_height = 480;
  int depth_fps = 30;

  /**
   * @brief 目标坐标系标识 (TF Frame ID)
   * 规则：
   * - 必须为非空字符串；
   * - 严禁以 '/' 开头（遵循 ROS TF 标准规范）；
   * - 严禁包含空格、制表符或换行符；
   * - 因深度图已实时对齐至彩色相机视角，彩色图、对齐深度图以及 CameraInfo 统一共用此 frame_id；
   * - 驱动自身不发布 TF，需由机器人的 URDF / robot_state_publisher 提供该 frame 到 base_link 的变换。
   */
  std::string color_frame_id = "d455_color_optical_frame";

  /**
   * @brief 等待 GLOBAL_TIME 时钟域就绪的预热超时时间 (单位: 秒)
   * 规则：
   * - 必须为有限正浮点数（如 5.0）；
   * - 设备启动或重连后，驱动会等待彩色与深度流的采样时间域均稳定进入 GLOBAL_TIME；
   * - 若在超时时间内未收到有效对齐帧，则判定预热失败。
   */
  double global_time_timeout = 5.0;

  /**
   * @brief 采集主循环中单次等待新帧的最长时间 (单位: 毫秒)
   * 规则：
   * - 必须为正整数（如 100）；
   * - 控制 poll_for_frames 协作式轮询的单次阻塞上限，兼顾低延迟取帧与对外停机信号的即时响应。
   */
  int frame_wait_timeout_ms = 100;

  /**
   * @brief 采集停滞监测看门狗超时时间 (单位: 秒)
   * 规则：
   * - 必须为有限正浮点数（如 5.0）；
   * - 当彩色流或对齐深度流持续超过此时间未产出新的有效帧时（如 USB 异常、带宽卡死），
   *   触发自动故障自愈流程。
   */
  double capture_stall_timeout = 5.0;

  /**
   * @brief 故障重连重试之间的退避等待间隔 (单位: 秒)
   * 规则：
   * - 必须为有限正浮点数（如 1.0）；
   * - 重连第 1 次尝试会立即执行，后续连续失败尝试之间按此间隔进行延时退避，防止密集轮询打满 CPU。
   */
  double reconnect_interval = 1.0;

  /**
   * @brief 故障恢复最大连续重启尝试次数
   * 规则：
   * - 必须为正整数（如 5）；
   * - 若连续重试次数耗尽仍无法恢复流水线，则安全停机并报错退出，避免死循环占用系统资源。
   */
  int max_reconnect_attempts = 5;
};

// 默认相机参数实例
static const CameraConfig kDefaultCameraConfig{};

}  // namespace d455_camera
