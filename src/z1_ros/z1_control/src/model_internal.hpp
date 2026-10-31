#pragma once
#include "z1_control/model.hpp"
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
namespace z1 {
struct Model::Impl {
  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  pinocchio::FrameIndex tool;
  std::string revision;
  Pose toolKinematics(const Vec &q, Jacobian &jacobian);
};
} // namespace z1
