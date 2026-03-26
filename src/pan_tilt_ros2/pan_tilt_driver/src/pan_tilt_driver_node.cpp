/**********************************************************************************
** MIT License
**
** Copyright (c) 2019 I-Quotient-Robotics https://github.com/I-Quotient-Robotics
** ROS2 port (c) 2026
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
** SOFTWARE.
*********************************************************************************/
#include <memory>
#include <string>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "pan_tilt_msgs/msg/pan_tilt_cmd_deg.hpp"
#include "pan_tilt_msgs/msg/pan_tilt_cmd_rad.hpp"
#include "pan_tilt_msgs/msg/pan_tilt_status.hpp"
#include "PanTiltDriver.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PanTiltDriverNode : public rclcpp::Node
{
public:
  PanTiltDriverNode() : Node("pan_tilt_driver_node")
  {
    // Declare and get parameters
    this->declare_parameter("ID", 0x01);
    this->declare_parameter("port_name", std::string("/dev/ttyUSB0"));
    this->declare_parameter("yaw_joint_name", std::string("iqr_pan_tilt_yaw_joint"));
    this->declare_parameter("pitch_joint_name", std::string("iqr_pan_tilt_pitch_joint"));

    id_ = this->get_parameter("ID").as_int();
    port_name_ = this->get_parameter("port_name").as_string();
    yaw_joint_name_ = this->get_parameter("yaw_joint_name").as_string();
    pitch_joint_name_ = this->get_parameter("pitch_joint_name").as_string();

    // Initialize JointState message
    js_.name.resize(2);
    js_.position.resize(2);
    js_.velocity.resize(2);
    js_.effort.resize(2);
    js_.name[0] = yaw_joint_name_;
    js_.name[1] = pitch_joint_name_;

    // Publishers
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    status_pub_ = this->create_publisher<pan_tilt_msgs::msg::PanTiltStatus>("/pan_tilt_status", 10);

    // Subscribers
    cmd_deg_sub_ = this->create_subscription<pan_tilt_msgs::msg::PanTiltCmdDeg>(
        "pan_tilt_cmd_deg", 10, std::bind(&PanTiltDriverNode::callBackDeg, this, _1));
    cmd_rad_sub_ = this->create_subscription<pan_tilt_msgs::msg::PanTiltCmdRad>(
        "pan_tilt_cmd_rad", 10, std::bind(&PanTiltDriverNode::callBackRad, this, _1));

    // Create driver
    RCLCPP_INFO(this->get_logger(), "pan-tilt ID:%d, port name:%s", id_, port_name_.c_str());
    pt_ = std::make_unique<IQR::PanTiltDriver>(static_cast<uint8_t>(id_), port_name_);

    // Timer at 50Hz
    timer_ = this->create_wall_timer(20ms, std::bind(&PanTiltDriverNode::publishState, this));
  }

private:
  void publishState()
  {
    IQR::PanTiltStatus pt_st;
    if (pt_->getStatus(pt_st))
    {
      double yaw_rad = 0.01745329 * pt_st.yaw_now;
      double pitch_rad = 0.01745329 * pt_st.pitch_now;

      js_.header.stamp = this->now();
      js_.position[0] = yaw_rad;
      js_.position[1] = pitch_rad;
      joint_pub_->publish(js_);

      pan_tilt_msgs::msg::PanTiltStatus st;
      st.header.stamp = this->now();
      st.id = pt_st.id;
      st.serial_num = pt_st.serial_num;
      st.hw_version = pt_st.hw_version;
      st.bd_version = pt_st.bd_version;
      st.sw_version = pt_st.sw_version;
      st.set_zero = pt_st.set_zero;
      st.speed = pt_st.speed;
      st.yaw_goal = pt_st.yaw_goal;
      st.pitch_goal = pt_st.pitch_goal;
      st.reserved = pt_st.reserved;
      st.driver_ec = pt_st.driver_ec;
      st.encoder_ec = pt_st.encoder_ec;
      st.yaw_now = pt_st.yaw_now;
      st.pitch_now = pt_st.pitch_now;
      st.yaw_temp = pt_st.yaw_temp;
      st.pitch_temp = pt_st.pitch_temp;
      st.yaw_raw = pt_st.yaw_raw;
      st.pitch_raw = pt_st.pitch_raw;
      st.loop_ec = pt_st.loop_ec;
      st.loop_time = pt_st.loop_time;
      status_pub_->publish(st);
    }
  }

  void callBackRad(const pan_tilt_msgs::msg::PanTiltCmdRad::SharedPtr msg)
  {
    if (msg->yaw > 1.0471974 || msg->yaw < -1.0471974 ||
        msg->pitch > 1.0471974 || msg->pitch < -1.0471974 ||
        msg->speed > 0.5235987 || msg->speed < 0.01745329)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Input param error, yaw:[-1.0471974, 1.0471974] pitch:[-1.0471974, 1.0471974] speed[0.01745329, 0.5235987]");
      return;
    }
    pt_->setPose(float(msg->yaw / 0.01745329), float(msg->pitch / 0.01745329), uint16_t(msg->speed / 0.01745329));
  }

  void callBackDeg(const pan_tilt_msgs::msg::PanTiltCmdDeg::SharedPtr msg)
  {
    if (msg->yaw > 60.0 || msg->yaw < -60.0 ||
        msg->pitch > 60.0 || msg->pitch < -60.0 ||
        msg->speed > 30 || msg->speed <= 0)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Input param error, yaw:[-60, 60] pitch:[-60, 60] speed[1, 30]");
      return;
    }
    pt_->setPose(msg->yaw, msg->pitch, msg->speed);
  }

  // Members
  std::unique_ptr<IQR::PanTiltDriver> pt_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<pan_tilt_msgs::msg::PanTiltStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<pan_tilt_msgs::msg::PanTiltCmdDeg>::SharedPtr cmd_deg_sub_;
  rclcpp::Subscription<pan_tilt_msgs::msg::PanTiltCmdRad>::SharedPtr cmd_rad_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int id_;
  std::string port_name_;
  std::string yaw_joint_name_;
  std::string pitch_joint_name_;
  sensor_msgs::msg::JointState js_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PanTiltDriverNode>());
  rclcpp::shutdown();
  return 0;
}
