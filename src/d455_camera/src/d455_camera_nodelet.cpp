#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <librealsense2/rs.hpp>
#include <stdexcept>
#include <string>

#include "camera_config.h"

namespace d455_camera {

class D455CameraNodelet : public nodelet::Nodelet {
 public:
  // Stop callbacks before the pipeline and publishers are destroyed.
  ~D455CameraNodelet() override { capture_timer_.stop(); }

 private:
  void onInit() override {
    ros::NodeHandle nh = getNodeHandle();
    image_transport::ImageTransport transport(nh);
    color_pub_ = transport.advertise("color/image_raw", 1);
    depth_pub_ = transport.advertise("aligned_depth_to_color/image_raw", 1);
    camera_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("color/camera_info", 1, true);

    rs2::config streams;
    streams.enable_stream(RS2_STREAM_COLOR, config_.width, config_.height,
                          RS2_FORMAT_BGR8, config_.fps);
    streams.enable_stream(RS2_STREAM_DEPTH, config_.width, config_.height,
                          RS2_FORMAT_Z16, config_.fps);
    const auto device = streams.resolve(pipeline_).get_device();
    device.first<rs2::color_sensor>().set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);
    device.first<rs2::depth_sensor>().set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);

    const auto profile = pipeline_.start(streams);
    const auto color_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    FillCameraInfo(color_profile.get_intrinsics());
    millimeters_per_unit_ =
        static_cast<double>(profile.get_device().first<rs2::depth_sensor>().get_depth_scale()) * 1000.0;

    // getNodeHandle() uses the nodelet's single-threaded callback queue.
    capture_timer_ = nh.createWallTimer(ros::WallDuration(0.005),
                                       &D455CameraNodelet::Capture, this);
    NODELET_INFO("D455 started: %dx%d at %d FPS.", config_.width, config_.height, config_.fps);
  }

  static ros::Time FrameStamp(const rs2::frame &frame) {
    ros::Time stamp;
    return stamp.fromSec(frame.get_timestamp() / 1000.0);
  }

  void Capture(const ros::WallTimerEvent &) {
    rs2::frameset frames;
    if (!pipeline_.poll_for_frames(&frames)) {
      return;
    }
    const auto color = frames.get_color_frame();
    const auto depth = frames.get_depth_frame();
    // GLOBAL_TIME becomes available after the SDK establishes the clock mapping.
    if (color.get_frame_timestamp_domain() != RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME ||
        depth.get_frame_timestamp_domain() != RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME) {
      return;
    }

    const ros::Time color_stamp = FrameStamp(color);
    color_info_.header.stamp = color_stamp;
    camera_info_pub_.publish(color_info_);
    if (color_pub_.getNumSubscribers() > 0) {
      auto message = MakeImage(color, color_stamp, sensor_msgs::image_encodings::BGR8,
                               color.get_stride_in_bytes());
      std::memcpy(message->data.data(), color.get_data(), message->data.size());
      color_pub_.publish(message);
    }
    if (depth_pub_.getNumSubscribers() > 0) {
      const rs2::frameset aligned = align_.process(frames);
      depth_pub_.publish(MakeDepthImage(aligned.get_depth_frame(), FrameStamp(depth)));
    }
  }

  sensor_msgs::ImagePtr MakeImage(const rs2::video_frame &frame, const ros::Time &stamp,
                                  const std::string &encoding, uint32_t step) const {
    sensor_msgs::ImagePtr message(new sensor_msgs::Image());
    message->header.stamp = stamp;
    message->header.frame_id = config_.color_frame_id;
    message->height = frame.get_height();
    message->width = frame.get_width();
    message->encoding = encoding;
    const uint16_t endian = 1;
    message->is_bigendian = *reinterpret_cast<const uint8_t *>(&endian) == 0;
    message->step = step;
    message->data.resize(static_cast<size_t>(step) * message->height);
    return message;
  }

  sensor_msgs::ImagePtr MakeDepthImage(const rs2::depth_frame &frame,
                                       const ros::Time &stamp) const {
    auto message = MakeImage(frame, stamp, sensor_msgs::image_encodings::TYPE_16UC1,
                             frame.get_width() * sizeof(uint16_t));
    const auto *source = static_cast<const uint8_t *>(frame.get_data());
    for (size_t y = 0; y < message->height; ++y) {
      const auto *row = source + y * frame.get_stride_in_bytes();
      for (size_t x = 0; x < message->width; ++x) {
        uint16_t raw;
        std::memcpy(&raw, row + x * sizeof(raw), sizeof(raw));
        const double millimeters = std::round(raw * millimeters_per_unit_);
        // Zero denotes invalid depth; values beyond 16UC1's range also map to zero.
        const uint16_t value = raw == 0 || millimeters > 65535.0
                                   ? 0
                                   : static_cast<uint16_t>(std::max(1.0, millimeters));
        std::memcpy(message->data.data() + y * message->step + x * sizeof(value),
                    &value, sizeof(value));
      }
    }
    return message;
  }

  void FillCameraInfo(const rs2_intrinsics &intrinsics) {
    color_info_.header.frame_id = config_.color_frame_id;
    color_info_.width = intrinsics.width;
    color_info_.height = intrinsics.height;
    color_info_.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    color_info_.D.assign(5, 0.0);
    switch (intrinsics.model) {
      case RS2_DISTORTION_NONE:
        break;
      case RS2_DISTORTION_BROWN_CONRADY:
        color_info_.D.assign(intrinsics.coeffs, intrinsics.coeffs + 5);
        break;
      case RS2_DISTORTION_MODIFIED_BROWN_CONRADY:
      case RS2_DISTORTION_INVERSE_BROWN_CONRADY:
        // With zero distortion these models reduce to the same pinhole projection.
        if (std::any_of(std::begin(intrinsics.coeffs), std::end(intrinsics.coeffs),
                        [](float coefficient) { return coefficient != 0.0f; })) {
          throw std::runtime_error("Color distortion cannot be represented as ROS plumb_bob.");
        }
        break;
      default:
        throw std::runtime_error("Unsupported D455 color distortion model.");
    }
    color_info_.K[0] = color_info_.P[0] = intrinsics.fx;
    color_info_.K[2] = color_info_.P[2] = intrinsics.ppx;
    color_info_.K[4] = color_info_.P[5] = intrinsics.fy;
    color_info_.K[5] = color_info_.P[6] = intrinsics.ppy;
    color_info_.K[8] = color_info_.P[10] = 1.0;
    color_info_.R[0] = color_info_.R[4] = color_info_.R[8] = 1.0;
  }

  const CameraConfig config_;
  rs2::pipeline pipeline_;
  rs2::align align_{RS2_STREAM_COLOR};
  double millimeters_per_unit_ = 0.0;
  image_transport::Publisher color_pub_;
  image_transport::Publisher depth_pub_;
  ros::Publisher camera_info_pub_;
  sensor_msgs::CameraInfo color_info_;
  ros::WallTimer capture_timer_;
};

}  // namespace d455_camera

PLUGINLIB_EXPORT_CLASS(d455_camera::D455CameraNodelet, nodelet::Nodelet)
