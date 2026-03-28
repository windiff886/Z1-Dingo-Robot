#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <librealsense2/rs.hpp>

namespace d455_camera
{

class D455CameraNode : public rclcpp::Node
{
public:
  D455CameraNode()
  : Node("d455_camera_node"),
    running_(false),
    enable_color_(true),
    enable_depth_(true),
    use_hardware_time_(true)
  {
    declare_parameter<std::string>("serial_no", "");
    declare_parameter<bool>("enable_color", true);
    declare_parameter<bool>("enable_depth", true);
    declare_parameter<int>("color_width", 640);
    declare_parameter<int>("color_height", 480);
    declare_parameter<int>("color_fps", 30);
    declare_parameter<int>("depth_width", 640);
    declare_parameter<int>("depth_height", 480);
    declare_parameter<int>("depth_fps", 30);
    declare_parameter<std::string>("color_frame_id", "d455_color_optical_frame");
    declare_parameter<std::string>("depth_frame_id", "d455_depth_optical_frame");
    declare_parameter<bool>("use_hardware_time", true);

    serial_no_ = get_parameter("serial_no").as_string();
    enable_color_ = get_parameter("enable_color").as_bool();
    enable_depth_ = get_parameter("enable_depth").as_bool();
    color_width_ = get_parameter("color_width").as_int();
    color_height_ = get_parameter("color_height").as_int();
    color_fps_ = get_parameter("color_fps").as_int();
    depth_width_ = get_parameter("depth_width").as_int();
    depth_height_ = get_parameter("depth_height").as_int();
    depth_fps_ = get_parameter("depth_fps").as_int();
    color_frame_id_ = get_parameter("color_frame_id").as_string();
    depth_frame_id_ = get_parameter("depth_frame_id").as_string();
    use_hardware_time_ = get_parameter("use_hardware_time").as_bool();

    if (!enable_color_ && !enable_depth_) {
      throw std::runtime_error("Color and depth streams cannot both be disabled.");
    }

    // QoS depth = 1，与 ROS1 queue_size=1 对齐（相机数据取最新帧即可）
    if (enable_color_) {
      color_image_pub_ = create_publisher<sensor_msgs::msg::Image>("color/image_raw", 1);
      color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("color/camera_info", 1);
    }
    if (enable_depth_) {
      depth_image_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/image_raw", 1);
      depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", 1);
    }

    rs2::config config;
    if (!serial_no_.empty()) {
      config.enable_device(serial_no_);
    }
    if (enable_color_) {
      config.enable_stream(
        RS2_STREAM_COLOR, color_width_, color_height_, RS2_FORMAT_BGR8, color_fps_);
    }
    if (enable_depth_) {
      config.enable_stream(
        RS2_STREAM_DEPTH, depth_width_, depth_height_, RS2_FORMAT_Z16, depth_fps_);
    }

    // Pipeline 启动异常保护（与 ROS1 try-catch 对齐）
    try {
      pipeline_profile_ = pipeline_.start(config);
    } catch (const rs2::error & e) {
      RCLCPP_ERROR(get_logger(), "Failed to start RealSense pipeline: %s", e.what());
      throw std::runtime_error(std::string("RealSense pipeline start failed: ") + e.what());
    }
    configure_device_time();
    configure_camera_infos();

    running_.store(true);
    capture_thread_ = std::thread(&D455CameraNode::capture_loop, this);
  }

  ~D455CameraNode() override
  {
    stop();
  }

private:
  void stop()
  {
    running_.store(false);
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }

    try {
      pipeline_.stop();
    } catch (const rs2::error &) {
    }
  }

  void configure_device_time()
  {
    if (!use_hardware_time_) {
      return;
    }

    try {
      auto device = pipeline_profile_.get_device();
      for (auto & sensor : device.query_sensors()) {
        if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
          sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);
        }
      }
    } catch (const rs2::error & error) {
      RCLCPP_WARN(get_logger(),
        "Failed to enable hardware time sync: %s. Using ROS time instead.", error.what());
      use_hardware_time_ = false;  // 与 ROS1 对齐：失败时回退到 ROS 时间
    }
  }

  void configure_camera_infos()
  {
    for (const auto & stream : pipeline_profile_.get_streams()) {
      auto video_profile = stream.as<rs2::video_stream_profile>();
      if (!video_profile) {
        continue;
      }

      if (stream.stream_type() == RS2_STREAM_COLOR && enable_color_) {
        fill_camera_info(video_profile, color_info_);
      } else if (stream.stream_type() == RS2_STREAM_DEPTH && enable_depth_) {
        fill_camera_info(video_profile, depth_info_);
      }
    }
  }

  void capture_loop()
  {
    while (running_.load() && rclcpp::ok()) {
      rs2::frameset frames;
      try {
        // 使用阻塞式 wait_for_frames()（与 ROS1 对齐），避免高负载时丢帧
        frames = pipeline_.wait_for_frames();
      } catch (const rs2::error & error) {
        if (running_.load()) {
          RCLCPP_ERROR(get_logger(), "RealSense capture error: %s", error.what());
        }
        continue;
      }

      auto stamp = to_ros_time(frames.get_timestamp());

      // 发布前检查订阅者（与 ROS1 getNumSubscribers() 对齐），无订阅者时跳过发布以节省带宽
      if (enable_color_) {
        auto color_frame = frames.get_color_frame();
        if (color_frame && color_image_pub_->get_subscription_count() > 0) {
          auto image_msg = make_image(
            color_frame, sensor_msgs::image_encodings::BGR8, color_frame_id_, stamp);
          auto info_msg = color_info_;
          info_msg.header = image_msg.header;
          color_image_pub_->publish(image_msg);
          color_info_pub_->publish(info_msg);
        }
      }

      if (enable_depth_) {
        auto depth_frame = frames.get_depth_frame();
        if (depth_frame && depth_image_pub_->get_subscription_count() > 0) {
          auto image_msg = make_image(
            depth_frame, sensor_msgs::image_encodings::TYPE_16UC1, depth_frame_id_, stamp);
          auto info_msg = depth_info_;
          info_msg.header = image_msg.header;
          depth_image_pub_->publish(image_msg);
          depth_info_pub_->publish(info_msg);
        }
      }
    }
  }

  builtin_interfaces::msg::Time to_ros_time(double timestamp_ms) const
  {
    if (use_hardware_time_ && timestamp_ms > 0.0) {
      const double seconds = timestamp_ms / 1000.0;
      const auto sec = static_cast<int32_t>(seconds);
      const auto nanosec = static_cast<uint32_t>((seconds - static_cast<double>(sec)) * 1e9);
      builtin_interfaces::msg::Time stamp;
      stamp.sec = sec;
      stamp.nanosec = nanosec;
      return stamp;
    }

    return get_clock()->now().to_msg();
  }

  sensor_msgs::msg::Image make_image(
    const rs2::video_frame & frame,
    const std::string & encoding,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp) const
  {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = frame.get_height();
    msg.width = frame.get_width();
    msg.encoding = encoding;
    msg.is_bigendian = false;
    msg.step = frame.get_stride_in_bytes();

    const auto size = static_cast<size_t>(msg.step) * msg.height;
    msg.data.resize(size);
    std::memcpy(msg.data.data(), frame.get_data(), size);
    return msg;
  }

  void fill_camera_info(
    const rs2::video_stream_profile & profile,
    sensor_msgs::msg::CameraInfo & info) const
  {
    const auto intrinsics = profile.get_intrinsics();
    info.width = intrinsics.width;
    info.height = intrinsics.height;

    switch (intrinsics.model) {
      case RS2_DISTORTION_KANNALA_BRANDT4:
        info.distortion_model = "equidistant";
        break;
      default:
        info.distortion_model = "plumb_bob";
        break;
    }

    info.d.resize(5);
    for (size_t i = 0; i < info.d.size(); ++i) {
      info.d[i] = intrinsics.coeffs[i];
    }

    info.k.fill(0.0);
    info.r.fill(0.0);
    info.p.fill(0.0);

    info.k[0] = intrinsics.fx;
    info.k[2] = intrinsics.ppx;
    info.k[4] = intrinsics.fy;
    info.k[5] = intrinsics.ppy;
    info.k[8] = 1.0;

    info.r[0] = 1.0;
    info.r[4] = 1.0;
    info.r[8] = 1.0;

    info.p[0] = intrinsics.fx;
    info.p[2] = intrinsics.ppx;
    info.p[5] = intrinsics.fy;
    info.p[6] = intrinsics.ppy;
    info.p[10] = 1.0;
  }

  rs2::pipeline pipeline_;
  rs2::pipeline_profile pipeline_profile_;
  std::thread capture_thread_;
  std::atomic<bool> running_;

  bool enable_color_;
  bool enable_depth_;
  bool use_hardware_time_;
  int color_width_;
  int color_height_;
  int color_fps_;
  int depth_width_;
  int depth_height_;
  int depth_fps_;
  std::string serial_no_;
  std::string color_frame_id_;
  std::string depth_frame_id_;

  sensor_msgs::msg::CameraInfo color_info_;
  sensor_msgs::msg::CameraInfo depth_info_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
};

}  // namespace d455_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<d455_camera::D455CameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
