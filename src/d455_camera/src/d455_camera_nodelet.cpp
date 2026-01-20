#include <atomic>
#include <cstring>
#include <string>
#include <thread>

#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <librealsense2/rs.hpp>

namespace d455_camera {

class D455CameraNodelet : public nodelet::Nodelet {
 public:
  D455CameraNodelet()
      : running_(false),
        enable_color_(true),
        enable_depth_(true) {}

  ~D455CameraNodelet() override { Stop(); }

 private:
  void onInit() override {
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle pnh = getPrivateNodeHandle();

    pnh.param("serial_no", serial_no_, std::string(""));
    pnh.param("enable_color", enable_color_, true);
    pnh.param("enable_depth", enable_depth_, true);
    pnh.param("color_width", color_width_, 640);
    pnh.param("color_height", color_height_, 480);
    pnh.param("color_fps", color_fps_, 30);
    pnh.param("depth_width", depth_width_, 640);
    pnh.param("depth_height", depth_height_, 480);
    pnh.param("depth_fps", depth_fps_, 30);
    pnh.param("color_frame_id", color_frame_id_, std::string("d455_color_optical_frame"));
    pnh.param("depth_frame_id", depth_frame_id_, std::string("d455_depth_optical_frame"));

    if (!enable_color_ && !enable_depth_) {
      NODELET_ERROR("Both color and depth streams are disabled.");
      return;
    }

    image_transport::ImageTransport it(nh);
    if (enable_color_) {
      color_pub_ = it.advertiseCamera("color/image_raw", 1);
    }
    if (enable_depth_) {
      depth_pub_ = it.advertiseCamera("depth/image_raw", 1);
    }

    rs2::config cfg;
    if (!serial_no_.empty()) {
      cfg.enable_device(serial_no_);
    }
    if (enable_color_) {
      cfg.enable_stream(RS2_STREAM_COLOR, color_width_, color_height_, RS2_FORMAT_BGR8,
                        color_fps_);
    }
    if (enable_depth_) {
      cfg.enable_stream(RS2_STREAM_DEPTH, depth_width_, depth_height_, RS2_FORMAT_Z16,
                        depth_fps_);
    }

    try {
      pipeline_profile_ = pipeline_.start(cfg);
    } catch (const rs2::error &e) {
      NODELET_ERROR("Failed to start RealSense pipeline: %s", e.what());
      return;
    }

    try {
      if (enable_color_) {
        auto color_profile =
            pipeline_profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        FillCameraInfo(color_profile, &color_info_);
      }
      if (enable_depth_) {
        auto depth_profile =
            pipeline_profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        FillCameraInfo(depth_profile, &depth_info_);
      }
    } catch (const rs2::error &e) {
      NODELET_ERROR("Failed to query camera intrinsics: %s", e.what());
      return;
    }

    running_.store(true);
    capture_thread_ = std::thread(&D455CameraNodelet::CaptureLoop, this);
  }

  void Stop() {
    if (!running_.exchange(false)) {
      return;
    }
    try {
      pipeline_.stop();
    } catch (const rs2::error &e) {
      NODELET_WARN("Failed to stop RealSense pipeline: %s", e.what());
    }
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
  }

  void CaptureLoop() {
    while (running_.load() && ros::ok()) {
      rs2::frameset frames;
      try {
        frames = pipeline_.wait_for_frames();
      } catch (const rs2::error &e) {
        if (running_.load()) {
          NODELET_ERROR("RealSense capture error: %s", e.what());
        }
        continue;
      }

      const ros::Time stamp = ros::Time::now();

      if (enable_color_) {
        rs2::video_frame color_frame = frames.get_color_frame();
        if (color_frame && color_pub_.getNumSubscribers() > 0) {
          sensor_msgs::ImagePtr image_msg =
              MakeImage(color_frame, sensor_msgs::image_encodings::BGR8, color_frame_id_, stamp);
          sensor_msgs::CameraInfoPtr info_msg(new sensor_msgs::CameraInfo(color_info_));
          info_msg->header = image_msg->header;
          color_pub_.publish(image_msg, info_msg);
        }
      }

      if (enable_depth_) {
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        if (depth_frame && depth_pub_.getNumSubscribers() > 0) {
          sensor_msgs::ImagePtr image_msg =
              MakeImage(depth_frame, sensor_msgs::image_encodings::TYPE_16UC1, depth_frame_id_,
                        stamp);
          sensor_msgs::CameraInfoPtr info_msg(new sensor_msgs::CameraInfo(depth_info_));
          info_msg->header = image_msg->header;
          depth_pub_.publish(image_msg, info_msg);
        }
      }
    }
  }

  sensor_msgs::ImagePtr MakeImage(const rs2::video_frame &frame, const std::string &encoding,
                                  const std::string &frame_id, const ros::Time &stamp) {
    sensor_msgs::ImagePtr msg(new sensor_msgs::Image());
    msg->header.stamp = stamp;
    msg->header.frame_id = frame_id;
    msg->height = frame.get_height();
    msg->width = frame.get_width();
    msg->encoding = encoding;
    msg->is_bigendian = false;
    msg->step = frame.get_stride_in_bytes();

    const size_t size = static_cast<size_t>(msg->step) * msg->height;
    msg->data.resize(size);
    std::memcpy(msg->data.data(), frame.get_data(), size);
    return msg;
  }

  void FillCameraInfo(const rs2::video_stream_profile &profile,
                      sensor_msgs::CameraInfo *info) {
    rs2_intrinsics intr = profile.get_intrinsics();
    info->width = intr.width;
    info->height = intr.height;

    switch (intr.model) {
      case RS2_DISTORTION_KANNALA_BRANDT4:
        info->distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
        break;
      default:
        info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        break;
    }

    info->D.resize(5);
    for (size_t i = 0; i < info->D.size(); ++i) {
      info->D[i] = intr.coeffs[i];
    }

    for (size_t i = 0; i < info->K.size(); ++i) {
      info->K[i] = 0.0;
    }
    for (size_t i = 0; i < info->R.size(); ++i) {
      info->R[i] = 0.0;
    }
    for (size_t i = 0; i < info->P.size(); ++i) {
      info->P[i] = 0.0;
    }

    info->K[0] = intr.fx;
    info->K[2] = intr.ppx;
    info->K[4] = intr.fy;
    info->K[5] = intr.ppy;
    info->K[8] = 1.0;

    info->R[0] = 1.0;
    info->R[4] = 1.0;
    info->R[8] = 1.0;

    info->P[0] = intr.fx;
    info->P[2] = intr.ppx;
    info->P[5] = intr.fy;
    info->P[6] = intr.ppy;
    info->P[10] = 1.0;
  }

  rs2::pipeline pipeline_;
  rs2::pipeline_profile pipeline_profile_;
  std::thread capture_thread_;
  std::atomic<bool> running_;

  bool enable_color_;
  bool enable_depth_;
  int color_width_;
  int color_height_;
  int color_fps_;
  int depth_width_;
  int depth_height_;
  int depth_fps_;
  std::string serial_no_;
  std::string color_frame_id_;
  std::string depth_frame_id_;

  image_transport::CameraPublisher color_pub_;
  image_transport::CameraPublisher depth_pub_;
  sensor_msgs::CameraInfo color_info_;
  sensor_msgs::CameraInfo depth_info_;
};

}  // namespace d455_camera

PLUGINLIB_EXPORT_CLASS(d455_camera::D455CameraNodelet, nodelet::Nodelet)
