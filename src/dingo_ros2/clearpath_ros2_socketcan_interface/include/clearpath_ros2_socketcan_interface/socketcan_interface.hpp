/**
Software License Agreement (BSD)

\authors   Roni Kreinin <rkreinin@clearpathrobotics.com>
\authors   Luis Camero <lcamero@clearpathrobotics.com>
\copyright Copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CLEARPATH_ROS2_SOCKETCAN_INTERFACE__SOCKETCAN_INTERFACE_HPP_
#define CLEARPATH_ROS2_SOCKETCAN_INTERFACE__SOCKETCAN_INTERFACE_HPP_


#include <queue>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "clearpath_ros2_socketcan_interface/visibility_control.h"

namespace clearpath_ros2_socketcan_interface
{

class SocketCANInterface
{
public:
  SocketCANInterface(const std::string & canbus, std::shared_ptr<rclcpp::Node> & nh);
  SocketCANInterface(
    const std::string& canbus,
    std::shared_ptr<rclcpp::Node> & nh, 
    std::function<void(const can_msgs::msg::Frame::SharedPtr msg)> cb);

  bool recv(can_msgs::msg::Frame::SharedPtr msg);
  void send(can_msgs::msg::Frame msg);
  void queue(can_msgs::msg::Frame msg);

  void startSendTimer(uint16_t period_ms);
  void stopSendTimer();

private:
  std::string canbus_;  // CANBUS interface
  std::shared_ptr<rclcpp::Node> nh_;
  rclcpp::TimerBase::SharedPtr send_timer_;

  std::queue<can_msgs::msg::Frame> can_rx_message_queue_;
  std::queue<can_msgs::msg::Frame> can_tx_message_queue_;
  std::mutex rx_queue_mutex_;
  std::mutex tx_queue_mutex_;

  can_msgs::msg::Frame frame_msg_;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_sub_;

  void rxCallback(const can_msgs::msg::Frame::SharedPtr msg);
  void sendFromQueue();
};

}  // namespace clearpath_ros2_socketcan_interface

#endif  // CLEARPATH_ROS2_SOCKETCAN_INTERFACE__SOCKETCAN_INTERFACE_HPP_
