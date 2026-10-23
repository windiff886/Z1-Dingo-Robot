/**
 *  \file       dingo_hardware.cpp
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

#include <array>
#include <cstdint>
#include <string>

#include "dingo_base/dingo_hardware.h"
#include "puma_motor_msgs/Status.h"

namespace dingo_base
{

DingoHardware::DingoHardware(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                             puma_motor_driver::Gateway& gateway) :
  gateway_(gateway),
  active_(false)
{
  // These six values are supplied by dingo_control/config/hardware/base.yaml.
  double gear_ratio, gain_p, gain_i, gain_d;
  int encoder_cpr;
  bool flip_motor_direction;
  pnh.getParam("gear_ratio", gear_ratio);
  pnh.getParam("encoder_cpr", encoder_cpr);
  pnh.getParam("flip_motor_direction", flip_motor_direction);
  pnh.getParam("gains/p", gain_p);
  pnh.getParam("gains/i", gain_i);
  pnh.getParam("gains/d", gain_d);

  const std::array<std::string, 4> joint_names =
      {{"front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"}};
  const std::array<uint8_t, 4> joint_can_ids = {{2, 3, 4, 5}};
  const std::array<double, 4> joint_directions = {{1, -1, 1, -1}};

  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    hardware_interface::JointStateHandle state_handle(joint_names[i],
        &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
    joint_state_interface_.registerHandle(state_handle);
    velocity_joint_interface_.registerHandle(hardware_interface::JointHandle(state_handle, &joints_[i].velocity_command));

    puma_motor_driver::Driver driver(gateway_, joint_can_ids[i], joint_names[i]);
    driver.clearMsgCache();
    driver.setEncoderCPR(encoder_cpr);
    driver.setGearRatio(gear_ratio * joint_directions[i] * (flip_motor_direction ? -1 : 1));
    driver.setMode(puma_motor_msgs::Status::MODE_SPEED, gain_p, gain_i, gain_d);
    drivers_.push_back(driver);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);
  multi_driver_node_.reset(new puma_motor_driver::MultiDriverNode(nh, drivers_));
}

void DingoHardware::configure()
{
  for (auto& driver : drivers_)
  {
    driver.configureParams();
  }
}

void DingoHardware::verify()
{
  for (auto& driver : drivers_)
  {
    driver.verifyParams();
  }
}

bool DingoHardware::areAllDriversActive() const
{
  for (const auto& driver : drivers_)
  {
    if (!driver.isConfigured())
    {
      return false;
    }
  }
  return true;
}

bool DingoHardware::isActive()
{
  const bool configured = areAllDriversActive();
  if (active_ != configured)
  {
    active_ = configured;
    multi_driver_node_->activePublishers(active_);
    ROS_INFO_STREAM("Dingo hardware " << (active_ ? "active" : "configuring"));
  }
  return active_;
}

void DingoHardware::checkPowerReset()
{
  for (auto& driver : drivers_)
  {
    if (driver.lastPower() != 0)
    {
      active_ = false;
      multi_driver_node_->activePublishers(false);
      for (auto& motor : drivers_)
      {
        motor.resetConfiguration();
      }
      return;
    }
  }
}

bool DingoHardware::inReset() const
{
  return !active_;
}

void DingoHardware::requestData()
{
  for (auto& driver : drivers_)
  {
    driver.requestFeedbackPowerState();
  }
}

void DingoHardware::updateJointsFromHardware()
{
  for (std::size_t i = 0; i < drivers_.size(); ++i)
  {
    joints_[i].effort = drivers_[i].lastCurrent();
    joints_[i].position = drivers_[i].lastPosition();
    joints_[i].velocity = drivers_[i].lastSpeed();
  }
}

void DingoHardware::command()
{
  for (std::size_t i = 0; i < drivers_.size(); ++i)
  {
    drivers_[i].commandSpeed(joints_[i].velocity_command);
  }
}

void DingoHardware::canRead()
{
  puma_motor_driver::Message message;
  while (gateway_.recv(&message))
  {
    for (auto& driver : drivers_)
    {
      driver.processMessage(message);
    }
  }
}

}  // namespace dingo_base
