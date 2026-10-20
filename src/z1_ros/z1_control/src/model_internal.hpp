#pragma once
#include "z1_control/model.hpp"
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
namespace z1 {
struct Model::Impl {
  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  pinocchio::GeometryModel geometry;
  std::unique_ptr<pinocchio::GeometryData> geometry_data;
  pinocchio::FrameIndex tool;
  std::vector<FixedBox> boxes;
  double radius_bound{};
  std::string revision;
};
} // namespace z1
