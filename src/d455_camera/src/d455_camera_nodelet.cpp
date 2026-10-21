#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <librealsense2/rs.hpp>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "camera_config.h"
#include "driver_logic.h"

namespace d455_camera {

/**
 * @brief Intel RealSense D455 深度相机 ROS 1 Nodelet 驱动核心类
 * 
 * 核心功能：
 * 1. 硬件流管理：基于 Librealsense2 配置并采集 D455 的彩色图与深度图。
 * 2. 空间视差对齐：实时将深度图对齐到彩色相机视角（aligned_depth_to_color）。
 * 3. 话题发布：输出 BGR8 彩色图、16UC1 毫米级对齐深度图以及共享的 CameraInfo。
 * 4. 高可用自愈：内置看门狗与采集停滞监测，支持设备异常掉线时的自动重连恢复。
 */
class D455CameraNodelet : public nodelet::Nodelet {
 public:
  /// @brief 构造函数，初始化相机底层流水线（配置项直接使用 CameraConfig 的默认值）
  D455CameraNodelet() : pipeline_(context_) {}

  /// @brief 析构函数，保证安全停止采集线程与相机底层流水线
  ~D455CameraNodelet() override { Stop(); }

 private:
  /// @brief 检查节点当前是否处于有效运行状态（未收到停止信号且 ROS 正常）
  bool Active() const { return !stop_signal_.Requested() && ros::ok(); }

  /**
   * @brief Nodelet 初始化入口（由 Nodelet Manager 在动态加载插件后调用）
   * 
   * 执行配置项校验、建立 ROS 话题发布者、初始化底层相机硬件流，并拉起后台采集线程。
   */
  void onInit() override {
    ros::NodeHandle nh = getNodeHandle();
    image_transport::ImageTransport transport(nh);
    color_pub_ = transport.advertise("color/image_raw", 1);
    depth_pub_ = transport.advertise("aligned_depth_to_color/image_raw", 1);
    camera_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("color/camera_info", 1, true);

    try {
      InitializePipeline();
      capture_thread_ = std::thread(&D455CameraNodelet::CaptureLoop, this);
    } catch (const std::exception &e) {
      StopPipeline();
      NODELET_ERROR("D455 initialization failed: %s", e.what());
      throw;
    } catch (...) {
      StopPipeline();
      throw;
    }
  }

  /**
   * @brief 请求终止节点运行，等待采集线程安全汇合并释放相机 Pipeline
   */
  void Stop() {
    stop_signal_.Request();
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    // The capture thread owns pipeline operations until join has completed.
    StopPipeline();
  }

  /**
   * @brief 安全停止 RealSense 底层数据流并重置对齐器（保证 noexcept）
   */
  void StopPipeline() noexcept {
    if (!pipeline_started_) {
      return;
    }
    pipeline_started_ = false;
    try {
      pipeline_.stop();
    } catch (const std::exception &e) {
      NODELET_WARN("Failed to stop RealSense pipeline: %s", e.what());
    } catch (...) {
      NODELET_WARN("Unknown exception stopping RealSense pipeline.");
    }
    align_.reset();
    pipeline_profile_ = rs2::pipeline_profile();
  }

  /**
   * @brief 构建 Librealsense 启动配置对象
   * @return 配置好的 rs2::config 对象
   */
  rs2::config MakeConfig() const {
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, config_.color_width, config_.color_height,
                         RS2_FORMAT_BGR8, config_.color_fps);
    config.enable_stream(RS2_STREAM_DEPTH, config_.depth_width, config_.depth_height,
                         RS2_FORMAT_Z16, config_.depth_fps);
    return config;
  }

  /**
   * @brief 初始化单台相机的 Pipeline
   *
   * 调用场景：
   * 1. 节点启动首次初始化（由 onInit() 调用）；
   * 2. 硬件断流或异常自愈重连（由 RecoverPipeline() 反复重试调用）。
   *
   * 核心处理步骤：
   * 1. 活跃状态前置防御检查（若节点正被关闭或重连期间收到退出信号，则快速失败退出）；
   * 2. 检查相机是否已连接，并解析数据流配置；
   * 3. 在底层传感器层面开启硬件时间关联宿主机时钟的 GLOBAL_TIME；
   * 4. 启动 pipeline 并获取实际激活的设备；
   * 5. 提取彩色相机内参并校验深度传感器的深度比例因子（depth scale）；
   * 6. 创建空间对齐器（rs2::align 到彩色视点）；
   * 7. 执行 Warmup 预热丢弃不稳定初始帧，获取第一组有效帧并发布初始 CameraInfo。
   */
  void InitializePipeline() {
    // 1. 防御性前置检查：若 ROS 正在关闭（Ctrl+C）或已收到停机请求（如自愈重连期间退出），立即取消并快速失败
    if (!Active()) {
      throw std::runtime_error("Camera initialization cancelled.");
    }

    // 2. 设备存在性预检：相机掉线时直接报错，避免进入 SDK 的设备等待
    if (context_.query_devices().size() == 0) {
      throw std::runtime_error("RealSense camera is not connected.");
    }

    // 3. 解析数据流配置，自动选择当前连接的唯一相机
    rs2::config config = MakeConfig();
    auto device = config.resolve(pipeline_).get_device();

    // 4. 遍历传感器：强制开启彩色与深度相机的 GLOBAL_TIME 硬件时钟对齐
    for (auto sensor : device.query_sensors()) {
      // 过滤：只挑出负责出彩色图和深度图的传感器，略过 IMU
      bool used_by_rgbd = false;
      for (auto profile : sensor.get_stream_profiles()) {
        if (profile.stream_type() == RS2_STREAM_COLOR ||
            profile.stream_type() == RS2_STREAM_DEPTH) {
          used_by_rgbd = true;
          break;
        }
      }
      if (!used_by_rgbd) {
        continue;
      }
      // 将深度传感器和彩色传感器的“全局时间对齐开关”全部强行拨到 1.0f (开启)
      if (!sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
        throw std::runtime_error("An RGBD sensor does not support GLOBAL_TIME.");
      }
      sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);
    }

    // 5. 复核活跃状态后启动流水线，并获取实际激活的设备
    if (!Active()) {
      throw std::runtime_error("Camera initialization cancelled.");
    }
    pipeline_profile_ = pipeline_.start(config);
    pipeline_started_ = true;
    device = pipeline_profile_.get_device();

    // 6. 读取相机几何内参并填充 ROS CameraInfo 消息结构
    sensor_msgs::CameraInfo info;
    auto color_profile =
        pipeline_profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    FillCameraInfo(color_profile.get_intrinsics(), &info);

    // 7. 读取深度传感器的单位比例因子（米/单位，通常为 0.001 即 1mm）
    const float depth_units = device.first<rs2::depth_sensor>().get_depth_scale();
    if (!std::isfinite(depth_units) || depth_units <= 0.0f) {
      throw std::runtime_error("Invalid depth scale reported by depth sensor.");
    }

    // 8. 创建深度图对齐到彩色相机坐标系的图像对齐处理器
    std::unique_ptr<rs2::align> align(new rs2::align(RS2_STREAM_COLOR));

    // 9. 执行预热（Warmup）：丢弃初始未同步帧，等待第一组就绪的全局时间戳 RGB-D 帧
    const rs2::frameset ready_frames = Warmup();
    if (!Active()) {
      throw std::runtime_error("Camera initialization cancelled.");
    }

    // 10. 提交就绪状态：更新 CameraInfo、重置时序监控看门狗并发布初始标定话题
    info.header.frame_id = config_.color_frame_id;
    info.header.stamp = FrameStamp(ready_frames.get_color_frame());
    color_info_ = info;
    depth_units_ = depth_units;
    align_ = std::move(align);
    const auto now = detail::Clock::now();
    color_progress_.Reset(now);
    depth_progress_.Reset(now);
    camera_info_pub_.publish(color_info_);
    NODELET_INFO("D455 pipeline ready (depth scale %.9f m).", depth_units_);
  }

  /**
   * @brief 在指定截止时间前轮询等待新帧组
   * 
   * 使用 poll_for_frames 代替 wait_for_frames，避免 SDK 内部隐式重启流水线，
   * 同时支持随时响应节点的外部停止请求。
   * 
   * @param frames 输出参数，保存获取到的帧组
   * @param deadline 单调时钟截止时间
   * @return true 成功获取到帧且节点依然处于 Active 状态；false 超时或节点退出
   */
  bool WaitForFrames(rs2::frameset *frames, detail::Clock::time_point deadline) {
    while (Active() && detail::Clock::now() < deadline) {
      // Unlike wait_for_frames/try_wait_for_frames, poll_for_frames does not restart
      // the pipeline inside the SDK. All recovery goes through InitializePipeline.
      if (pipeline_.poll_for_frames(frames)) {
        return Active();
      }
      const auto wake = std::min(deadline, detail::Clock::now() + std::chrono::milliseconds(5));
      if (!stop_signal_.WaitUntil(wake)) {
        break;
      }
    }
    return false;
  }

  /**
   * @brief 相机启动与重连后的预热（Preheat）阶段
   * 
   * 丢弃初始不稳定帧，确保彩色帧与深度帧均已成功切换至 GLOBAL_TIME 时间域，
   * 且时间戳在 ROS 1 时间表示范围内。
   * 
   * @return 预热就绪的第一组有效 RGB-D 帧
   */
  rs2::frameset Warmup() {
    const auto deadline =
        detail::Clock::now() + std::chrono::duration_cast<detail::Clock::duration>(
                                   std::chrono::duration<double>(config_.global_time_timeout));
    while (Active() && detail::Clock::now() < deadline) {
      rs2::frameset frames;
      if (!WaitForFrames(&frames, deadline)) {
        break;
      }
      const auto color = frames.get_color_frame();
      const auto depth = frames.get_depth_frame();
      detail::Timestamp timestamp;
      if (color && depth &&
          color.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME &&
          depth.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME &&
          detail::SplitTimestamp(color.get_timestamp(), &timestamp) &&
          detail::SplitTimestamp(depth.get_timestamp(), &timestamp)) {
        return frames;
      }
      // Invalid or duplicated inputs must not make preheating a busy loop.
      stop_signal_.WaitUntil(
          std::min(deadline, detail::Clock::now() + std::chrono::milliseconds(5)));
    }
    if (!Active()) {
      throw std::runtime_error("GLOBAL_TIME warmup cancelled.");
    }
    throw std::runtime_error("GLOBAL_TIME warmup timed out before a valid RGBD pair arrived.");
  }

  /**
   * @brief 相机断流与硬件异常自愈恢复机制
   * 
   * 当检测到图像停滞（Stalled）、时间域异常或帧号回退时触发。
   * 停止当前流水线，并在退避间隔后重新尝试 InitializePipeline()。
   * 
   * @return true 恢复成功；false 达到最大重试次数失败
   */
  bool RecoverPipeline() {
    StopPipeline();
    for (int attempt = 0; attempt < config_.max_reconnect_attempts && Active(); ++attempt) {
      // Only failed attempts require backoff; the first attempt is immediate.
      if (attempt > 0) {
        const auto deadline =
            detail::Clock::now() + std::chrono::duration_cast<detail::Clock::duration>(
                                       std::chrono::duration<double>(config_.reconnect_interval));
        while (Active() && detail::Clock::now() < deadline) {
          stop_signal_.WaitUntil(
              std::min(deadline, detail::Clock::now() + std::chrono::milliseconds(10)));
        }
      }
      if (!Active()) {
        return false;
      }
      try {
        InitializePipeline();
        NODELET_INFO("Pipeline recovered on attempt %d.", attempt + 1);
        return true;
      } catch (const std::exception &e) {
        StopPipeline();
        if (!Active()) {
          return false;
        }
        NODELET_WARN("Recovery attempt %d/%d failed: %s", attempt + 1,
                     config_.max_reconnect_attempts, e.what());
      }
    }
    if (Active()) {
      NODELET_ERROR(
          "Exceeded %d recovery attempts. Capture halted; unload/reload the camera to retry.",
          config_.max_reconnect_attempts);
    }
    return false;
  }

  /**
   * @brief 后台采集工作线程主循环
   * 
   * 线程职责：
   * 1. 看门狗监测：实时检查 RGB 或深度流是否超出 capture_stall_timeout 发生断流停滞，超时则自动触发流水线重连自愈；
   * 2. 轮询取帧：在 frame_wait_timeout_ms 窗口内可中断等待硬件帧就绪；
   * 3. 数据分发：调用 ProcessFrames 对帧组进行空间对齐与 ROS 话题发布；
   * 4. 节流防忙等：通过微秒级可取消休眠防止连续无效输入导致 CPU 满载；
   * 5. 异常安全保障：声明为 noexcept 并顶层拦截一切异常，杜绝子线程崩溃引发进程闪退。
   */
  void CaptureLoop() noexcept {
    try {
      // 1. 核心工作循环：持续运转直到收到主动停机信号或 ROS 关闭
      while (Active()) {
        const auto now = detail::Clock::now();

        // 2. 数据流断流看门狗检测：若彩色流或深度流停滞未更新有效帧超过阈值，主动尝试自愈
        if (color_progress_.Stalled(now, config_.capture_stall_timeout) ||
            depth_progress_.Stalled(now, config_.capture_stall_timeout)) {
          NODELET_WARN("RGB or aligned depth stopped producing valid new frames. Recovering...");
          // 若自愈重试达到最大次数依然失败，则跳出循环结束采集
          if (!RecoverPipeline()) {
            break;
          }
        }

        // 3. 计算本轮单帧获取的单调时钟截止时间
        rs2::frameset frames;
        const auto deadline =
            detail::Clock::now() + std::chrono::milliseconds(config_.frame_wait_timeout_ms);

        // 4. 轮询获取帧组并执行对齐与发布（捕获 RealSense SDK 运行时异常以防单帧错误破坏循环）
        try {
          if (WaitForFrames(&frames, deadline)) {
            ProcessFrames(frames);
          }
        } catch (const rs2::error &e) {
          NODELET_WARN_THROTTLE(1.0, "Frame processing failed: %s", e.what());
        }

        // 5. 节流防忙等：让出 1ms CPU 时间，即使遭遇连续无效帧或瞬时错误也不会吃满单核 CPU，同时可被停机信号即刻打断
        stop_signal_.WaitUntil(detail::Clock::now() + std::chrono::milliseconds(1));
      }
    } catch (const std::exception &e) {
      NODELET_FATAL("Capture halted by exception: %s", e.what());
    } catch (...) {
      NODELET_FATAL("Capture halted by unknown exception.");
    }

    // 6. 异常退出联动清理：无论何种原因退出工作循环，均广播停机信号并关闭底层相机流水线
    stop_signal_.Request();
    StopPipeline();
  }

  /**
   * @brief 将 Librealsense 毫秒级时间戳转换为 ROS 1 ros::Time
   */
  static ros::Time FrameStamp(const rs2::frame &frame) {
    detail::Timestamp timestamp;
    if (!detail::SplitTimestamp(frame.get_timestamp(), &timestamp)) {
      throw std::runtime_error("Frame timestamp cannot be represented by ROS 1 Time.");
    }
    return ros::Time(timestamp.sec, timestamp.nsec);
  }

  /**
   * @brief 帧组解析、空间对齐与 ROS 消息发布核心流程
   * 
   * 处理逻辑：
   * 1. 检查帧有效性与 GLOBAL_TIME 时钟域完整性；
   * 2. 帧去重与时间戳单调性校验（防异常回退）；
   * 3. 发布彩色图像与更新的 CameraInfo；
   * 4. 调用 SDK rs2::align 将深度图对齐到目标彩色坐标系；
   * 5. 校验对齐尺寸并发布 16UC1 深度图。
   */
  void ProcessFrames(const rs2::frameset &frames) {
    const auto color = frames.get_color_frame();
    const auto depth = frames.get_depth_frame();
    if (!color || !depth || !Active()) {
      return;
    }
    if (color.get_frame_timestamp_domain() != RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME ||
        depth.get_frame_timestamp_domain() != RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME) {
      NODELET_WARN_THROTTLE(1.0, "GLOBAL_TIME domain lost. Recovering...");
      if (!RecoverPipeline()) {
        stop_signal_.Request();
      }
      return;
    }
    detail::Timestamp timestamp;
    if (!detail::SplitTimestamp(color.get_timestamp(), &timestamp) ||
        !detail::SplitTimestamp(depth.get_timestamp(), &timestamp)) {
      NODELET_WARN_THROTTLE(1.0, "Invalid/unrepresentable source timestamps; discarding frames.");
      return;
    }
    if (color_progress_.Regressed(color.get_frame_number(), color.get_timestamp()) ||
        depth_progress_.Regressed(depth.get_frame_number(), depth.get_timestamp())) {
      NODELET_WARN("Source frame number or timestamp regressed. Recovering...");
      if (!RecoverPipeline()) {
        stop_signal_.Request();
      }
      return;
    }
    if (color_progress_.IsNew(color.get_frame_number())) {
      color_info_.header.stamp = FrameStamp(color);
      camera_info_pub_.publish(color_info_);
      if (color_pub_.getNumSubscribers() > 0) {
        color_pub_.publish(MakeColorImage(color, color_info_.header.stamp));
      }
      color_progress_.Accept(color.get_frame_number(), color.get_timestamp(), detail::Clock::now());
    }
    if (!depth_progress_.IsNew(depth.get_frame_number())) {
      return;
    }
    const rs2::frameset aligned = align_->process(frames);
    const auto aligned_depth = aligned.get_depth_frame();
    if (!aligned_depth || aligned_depth.get_width() != static_cast<int>(color_info_.width) ||
        aligned_depth.get_height() != static_cast<int>(color_info_.height)) {
      NODELET_WARN_THROTTLE(
          1.0, "Alignment produced empty depth or dimensions inconsistent with CameraInfo.");
      return;
    }
    if (depth_pub_.getNumSubscribers() > 0) {
      // Preserve the original depth sampling time, regardless of align output metadata.
      depth_pub_.publish(MakeAlignedDepthImage(aligned_depth, FrameStamp(depth)));
    }
    depth_progress_.Accept(depth.get_frame_number(), depth.get_timestamp(), detail::Clock::now());
  }

  /**
   * @brief 将 Librealsense 彩色帧打包为 ROS 图像消息智能指针（供同进程零拷贝发布）
   * 
   * @param frame SDK 原生彩色视频帧
   * @param stamp 对应的 ROS 时间戳
   * @return sensor_msgs::ImagePtr 格式消息
   */
  sensor_msgs::ImagePtr MakeColorImage(const rs2::video_frame &frame,
                                       const ros::Time &stamp) const {
    sensor_msgs::ImagePtr message(new sensor_msgs::Image());
    message->header.stamp = stamp;
    message->header.frame_id = config_.color_frame_id;
    message->height = frame.get_height();
    message->width = frame.get_width();
    message->encoding = sensor_msgs::image_encodings::BGR8;
    message->is_bigendian = detail::IsBigEndian();
    message->step = frame.get_stride_in_bytes();
    const size_t size = static_cast<size_t>(message->step) * message->height;
    message->data.resize(size);
    std::memcpy(message->data.data(), frame.get_data(), size);
    return message;
  }

  /**
   * @brief 将已对齐的深度帧打包为 16UC1 毫米格式的 ROS 图像消息智能指针
   * 
   * 处理按行 SDK 字节步长（stride），并将 Z16 单位通过 depth_units 缩放四舍五入为毫米。
   * 
   * @param frame 已对齐到彩色视角的深度帧
   * @param stamp 原始深度采样时间的 ROS 时间戳
   * @return sensor_msgs::ImagePtr 深度图像消息
   */
  sensor_msgs::ImagePtr MakeAlignedDepthImage(const rs2::depth_frame &frame,
                                              const ros::Time &stamp) const {
    sensor_msgs::ImagePtr message(new sensor_msgs::Image());
    message->header.stamp = stamp;
    message->header.frame_id = config_.color_frame_id;
    message->height = frame.get_height();
    message->width = frame.get_width();
    message->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    message->is_bigendian = detail::IsBigEndian();
    message->step = message->width * sizeof(uint16_t);
    const auto stride = frame.get_stride_in_bytes();
    if (stride < static_cast<int>(message->step)) {
      throw std::runtime_error("Depth stride is shorter than one row of pixels.");
    }
    message->data.resize(static_cast<size_t>(message->step) * message->height);
    const auto *source = static_cast<const uint8_t *>(frame.get_data());
    for (size_t y = 0; y < message->height; ++y) {
      detail::CopyDepthRow(source + y * stride, message->data.data() + y * message->step,
                           message->width, static_cast<double>(depth_units_) * 1000.0);
    }
    return message;
  }

  /**
   * @brief 解析 Librealsense 内参并填充 ROS 标准 CameraInfo 结构
   * 
   * 支持模型：
   * - RS2_DISTORTION_NONE / BROWN_CONRADY -> 映射为 plumb_bob (5 系数)
   * - RS2_DISTORTION_KANNALA_BRANDT4      -> 映射为 equidistant (4 系数)
   */
  static void FillCameraInfo(const rs2_intrinsics &intrinsics, sensor_msgs::CameraInfo *info) {
    if (intrinsics.width <= 0 || intrinsics.height <= 0 || !std::isfinite(intrinsics.fx) ||
        intrinsics.fx <= 0.0f || !std::isfinite(intrinsics.fy) || intrinsics.fy <= 0.0f ||
        !std::isfinite(intrinsics.ppx) || !std::isfinite(intrinsics.ppy)) {
      throw std::runtime_error("Invalid color camera intrinsics.");
    }
    bool zero_distortion = true;
    for (float coefficient : intrinsics.coeffs) {
      if (!std::isfinite(coefficient)) {
        throw std::runtime_error("Nonfinite color distortion coefficients.");
      }
      zero_distortion = zero_distortion && coefficient == 0.0f;
    }
    *info = sensor_msgs::CameraInfo();
    info->width = intrinsics.width;
    info->height = intrinsics.height;
    switch (intrinsics.model) {
      case RS2_DISTORTION_NONE:
        info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        info->D.assign(5, 0.0);
        break;
      case RS2_DISTORTION_MODIFIED_BROWN_CONRADY:
      case RS2_DISTORTION_INVERSE_BROWN_CONRADY:
        if (!zero_distortion) {
          throw std::runtime_error(
              "Nonzero Modified/Inverse Brown-Conrady cannot be copied into ROS plumb_bob.");
        }
        info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        info->D.assign(5, 0.0);
        break;
      case RS2_DISTORTION_BROWN_CONRADY:
        info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        info->D.assign(intrinsics.coeffs, intrinsics.coeffs + 5);
        break;
      case RS2_DISTORTION_KANNALA_BRANDT4:
        info->distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
        info->D.assign(intrinsics.coeffs, intrinsics.coeffs + 4);
        break;
      default:
        throw std::runtime_error("Unsupported color distortion model: " +
                                 std::to_string(intrinsics.model));
    }
    info->K[0] = info->P[0] = intrinsics.fx;
    info->K[2] = info->P[2] = intrinsics.ppx;
    info->K[4] = info->P[5] = intrinsics.fy;
    info->K[5] = info->P[6] = intrinsics.ppy;
    info->K[8] = info->P[10] = 1.0;
    info->R[0] = info->R[4] = info->R[8] = 1.0;
  }

  // --- RealSense 核心资源 ---
  rs2::context context_;                                ///< Librealsense 设备上下文
  rs2::pipeline pipeline_;                              ///< 相机采集流水线
  rs2::pipeline_profile pipeline_profile_;              ///< 运行中的流 Profile
  std::unique_ptr<rs2::align> align_;                   ///< 深度对齐到彩色视角的对齐处理器

  // --- 多线程与控制 ---
  std::thread capture_thread_;                          ///< 后台帧采集与分发线程
  detail::StopSignal stop_signal_;                      ///< 可取消等待与安全停机同步信号
  // No pipeline call runs concurrently: init -> capture thread -> destructor after join.
  bool pipeline_started_ = false;                       ///< 流水线是否处于运行中
  detail::StreamProgress color_progress_;               ///< 彩色流时序与停滞检测看门狗
  detail::StreamProgress depth_progress_;               ///< 深度流时序与停滞检测看门狗

  // --- 配置与设备属性 ---
  CameraConfig config_;                                 ///< 相机 C++ 配置参数
  float depth_units_ = 0.001f;                          ///< 深度原始单位（米/单位，默认 1mm）

  // --- ROS 话题发布者 ---
  image_transport::Publisher color_pub_;                ///< 彩色图像发布者
  image_transport::Publisher depth_pub_;                ///< 对齐深度图像发布者
  ros::Publisher camera_info_pub_;                      ///< 相机标定信息发布者 (Latched)
  sensor_msgs::CameraInfo color_info_;                  ///< 缓存的目标彩色视角 CameraInfo
};

}  // namespace d455_camera

PLUGINLIB_EXPORT_CLASS(d455_camera::D455CameraNodelet, nodelet::Nodelet)
