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

#include "clearpath_ros2_socketcan_interface/socketcan_interface.hpp"

namespace clearpath_ros2_socketcan_interface
{

/**
 * @brief Construct a new SocketCANInterface object
 * Uses default Rx callback to queue up received messages
 * 
 * @param canbus Name of CAN bus
 * @param nh Pointer to node handle
 */
SocketCANInterface::SocketCANInterface(
  const std::string & canbus,
  std::shared_ptr<rclcpp::Node> & nh
)
: canbus_(canbus), nh_(nh)
{
  // Rx Subscriber
  can_rx_sub_ = nh->create_subscription<can_msgs::msg::Frame>(
    canbus_ + "/rx",
    rclcpp::SensorDataQoS(),
    std::bind(&SocketCANInterface::rxCallback, this, std::placeholders::_1));

  // Tx Publisher
  can_tx_pub_ = nh->create_publisher<can_msgs::msg::Frame>(
    canbus_ + "/tx",
    rclcpp::SystemDefaultsQoS());
}

/**
 * @brief Construct a new SocketCANInterface object
 * Uses custom callback function to handle received messages
 * 
 * @param canbus Name of CAN bus
 * @param nh Pointer to node handle
 * @param cb Rx subscriber callback
 */
SocketCANInterface::SocketCANInterface(
    const std::string& canbus,
    std::shared_ptr<rclcpp::Node> & nh, 
    std::function<void(const can_msgs::msg::Frame::SharedPtr msg)> cb
)
: canbus_(canbus), nh_(nh)
{
  // Rx Subscriber
  can_rx_sub_ = nh->create_subscription<can_msgs::msg::Frame>(
    canbus_ + "/rx",
    rclcpp::SensorDataQoS(),
    cb);

  // Tx Publisher
  can_tx_pub_ = nh->create_publisher<can_msgs::msg::Frame>(
    canbus_ + "/tx",
    rclcpp::SystemDefaultsQoS());
}

/**
 * @brief Default Rx subscription callback
 * Queues messages
 * @param msg Pointer to received message
 */
void SocketCANInterface::rxCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (msg) {
    std::lock_guard<std::mutex> lock(rx_queue_mutex_);
    can_rx_message_queue_.push(*msg);
  }
}

/**
 * @brief Receive a CAN message frame from the Rx queue
 * Only works when interface is created with default constructor
 * @param msg Message pointer to fill with new message
 * @return true on success
 * @return false if queue is empty
 */
bool SocketCANInterface::recv(can_msgs::msg::Frame::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(rx_queue_mutex_);
  if (can_rx_message_queue_.empty()) {
    return false;
  }
  *msg = can_rx_message_queue_.front();
  can_rx_message_queue_.pop();
  return true;
}

/**
 * @brief Queue up a message to be sent at a later time by the wall timer.
 *
 * @param msg CAN message frame to send
 */
void SocketCANInterface::queue(can_msgs::msg::Frame msg)
{
  std::lock_guard<std::mutex> lock(tx_queue_mutex_);
  can_tx_message_queue_.push(msg);
}

/**
 * @brief Stamp and publish a CAN frame message
 * 
 * @param msg Message to publish
 */
void SocketCANInterface::send(can_msgs::msg::Frame msg)
{
  msg.header.stamp = nh_->get_clock()->now();
  can_tx_pub_->publish(msg);
}

/**
 * @brief Start a wall timer to periodically check for queued Tx messages and send them.
 * 
 * @param period_ms Wall timer period
 */
void SocketCANInterface::startSendTimer(uint16_t period_ms)
{
  if (period_ms == 0)
  {
    RCLCPP_WARN(nh_->get_logger(), "Period must be a non-zero value");
    return;
  }

  send_timer_ = nh_->create_wall_timer(
    std::chrono::milliseconds(period_ms), std::bind(&SocketCANInterface::sendFromQueue, this));
}

/**
 * @brief Stop the Tx wall timer
 * 
 */
void SocketCANInterface::stopSendTimer()
{
  send_timer_->cancel();
}

/**
 * @brief Callback for Tx wall timer
 * 
 */
void SocketCANInterface::sendFromQueue()
{
  std::lock_guard<std::mutex> lock(tx_queue_mutex_);
  if (!can_tx_message_queue_.empty()) {
    send(can_tx_message_queue_.front());
    can_tx_message_queue_.pop();
  }
}

}  // namespace clearpath_ros2_socketcan_interface
