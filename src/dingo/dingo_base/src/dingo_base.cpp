/**
 *  \file       dingo_base.cpp
 *  \brief      Main entry point for dingo base
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

#include <chrono>
#include <string>
#include <thread>

#include <boost/asio.hpp>

#include "controller_manager/controller_manager.h"
#include "dingo_base/dingo_cooling.h"
#include "dingo_base/dingo_diagnostic_updater.h"
#include "dingo_base/dingo_hardware.h"
#include "dingo_base/dingo_lighting.h"
#include "dingo_base/dingo_logger.h"
#include "puma_motor_driver/diagnostic_updater.h"
#include "puma_motor_driver/socketcan_gateway.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/spinner.h"
#include "rosserial_server/udp_socket_session.h"

int main(int argc, char* argv[])
{
  const std::string node_name = "dingo_node";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh, pnh("~");

  puma_motor_driver::SocketCANGateway gateway("vcan0");
  dingo_base::DingoHardware dingo(nh, pnh, gateway);
  gateway.connect();

  // Controller switching services wait for update(), so use their own callback queue.
  ros::CallbackQueue controller_queue;
  ros::NodeHandle controller_nh(nh);
  controller_nh.setCallbackQueue(&controller_queue);
  controller_manager::ControllerManager cm(&dingo, controller_nh);
  ros::AsyncSpinner controller_spinner(1, &controller_queue);
  controller_spinner.start();
  dingo_base::DingoLighting lighting(&nh);
  dingo_base::DingoCooling cooling(&nh);
  dingo_base::DingoDiagnosticUpdater diagnostics;
  puma_motor_driver::PumaMotorDriverDiagnosticUpdater motor_diagnostics;

  using boost::asio::ip::address;
  using boost::asio::ip::udp;
  const auto host = address::from_string("192.168.131.1");
  const auto mcu = address::from_string("192.168.131.2");
  boost::asio::io_service io_service;
  dingo_base::DingoLogger logger(node_name + "_mcu", nh, io_service, udp::endpoint(host, 11413));
  rosserial_server::UdpSocketSession socket(io_service, udp::endpoint(host, 11411), udp::endpoint(mcu, 11411));
  std::thread io_thread([&io_service] { io_service.run(); });

  ros::Rate rate(25);
  auto last_time = std::chrono::steady_clock::now();
  while (ros::ok())
  {
    // CAN caches and their feedback/status publishers share this thread.
    dingo.canRead();
    ros::spinOnce();

    const auto now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> elapsed = now - last_time;
    last_time = now;

    if (dingo.isActive())
    {
      dingo.checkPowerReset();
      dingo.updateJointsFromHardware();
    }
    else
    {
      dingo.configure();
    }

    cm.update(ros::Time::now(), ros::Duration(elapsed.count()), dingo.inReset());

    if (dingo.isActive())
    {
      dingo.command();
      dingo.requestData();
    }
    else
    {
      dingo.verify();
    }
    rate.sleep();
  }

  controller_spinner.stop();
  io_service.stop();
  io_thread.join();
  return 0;
}
