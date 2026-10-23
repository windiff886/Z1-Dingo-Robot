/**
 *  \file       dingo_hardware.h
 *  \brief      Class representing Dingo hardware
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

#ifndef DINGO_BASE_DINGO_HARDWARE_H
#define DINGO_BASE_DINGO_HARDWARE_H

#include <memory>
#include <vector>

#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"
#include "puma_motor_driver/driver.h"
#include "puma_motor_driver/gateway.h"
#include "puma_motor_driver/multi_driver_node.h"
#include "ros/ros.h"

namespace dingo_base
{

class DingoHardware : public hardware_interface::RobotHW
{
public:
  DingoHardware(ros::NodeHandle& nh, ros::NodeHandle& pnh, puma_motor_driver::Gateway& gateway);

  void configure();
  void verify();
  bool isActive();
  void checkPowerReset();
  bool inReset() const;
  void requestData();
  void updateJointsFromHardware();
  void command();
  void canRead();

private:
  bool areAllDriversActive() const;

  puma_motor_driver::Gateway& gateway_;
  std::vector<puma_motor_driver::Driver> drivers_;
  std::unique_ptr<puma_motor_driver::MultiDriverNode> multi_driver_node_;
  bool active_;
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  struct Joint
  {
    double position = 0;
    double velocity = 0;
    double effort = 0;
    double velocity_command = 0;
  }
  joints_[4];
};

}  // namespace dingo_base

#endif  // DINGO_BASE_DINGO_HARDWARE_H
