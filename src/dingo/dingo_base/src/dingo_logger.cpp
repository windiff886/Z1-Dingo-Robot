/**
 *  \file       dingo_logger.cpp
 *  \brief      Network logger class for Dingo
 *  \copyright  Copyright (c) 2020, Clearpath Robotics, Inc.
 *
 * Software License Agreement (BSD)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 */

#include <map>

#include "dingo_base/dingo_logger.h"

namespace dingo_base
{
namespace
{
const std::map<std::string, uint8_t> LOG_LEVELS =
{
  {"DEBUG", rosgraph_msgs::Log::DEBUG},
  {"INFO", rosgraph_msgs::Log::INFO},
  {"WARN", rosgraph_msgs::Log::WARN},
  {"ERROR", rosgraph_msgs::Log::ERROR},
  {"FATAL", rosgraph_msgs::Log::FATAL}
};
}  // namespace

DingoLogger::DingoLogger(const std::string& name, ros::NodeHandle& nh, boost::asio::io_service& io_service,
                         const boost::asio::ip::udp::endpoint& local_endpoint) :
  socket_(io_service, local_endpoint),
  log_pub_(nh.advertise<rosgraph_msgs::Log>("/rosout", 100)),
  name_(name),
  regexp_("^\\[(.+)\\] (.+):(\\d+) (.+)$")
{
  receive();
}

void DingoLogger::receive()
{
  socket_.async_receive_from(boost::asio::buffer(receive_buffer_), remote_endpoint_,
      [this](const boost::system::error_code&, std::size_t bytes_transferred)
      {
        onUdpReceive(bytes_transferred);
      });
}

void DingoLogger::onUdpReceive(std::size_t bytes_transferred)
{
  const std::string text(receive_buffer_.begin(), receive_buffer_.begin() + bytes_transferred);
  std::smatch fields;
  if (std::regex_match(text, fields, regexp_))
  {
    rosgraph_msgs::Log message;
    message.header.stamp = ros::Time::now();
    message.level = LOG_LEVELS.at(fields.str(1));
    message.file = fields.str(2);
    message.line = std::stoul(fields.str(3));
    message.name = name_;
    message.msg = fields.str(4);
    log_pub_.publish(message);
  }
  receive();
}

}  // namespace dingo_base
