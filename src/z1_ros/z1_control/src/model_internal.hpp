#pragma once
#include "z1_control/model.hpp"
#include <coal/BV/AABB.h>
#include <coal/distance.h>
#include <coal/shape/geometric_shapes.h>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
namespace z1 {
struct Model::Impl {
  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  pinocchio::GeometryModel geometry;
  pinocchio::FrameIndex tool;
  std::vector<FixedBox> boxes;
  struct CollisionBody {
    const coal::CollisionGeometry *shape{};
    pinocchio::JointIndex joint{};
    pinocchio::SE3 placement;
    Eigen::Vector3d center, half_size;
    Vec reach{Vec::Zero()};
    std::array<bool, 6> affected{};
    coal::AABB world_aabb;
    pinocchio::SE3 world;
  };
  struct DistancePair {
    size_t first{}, second{};
    Vec reach;
    std::string name;
    std::unique_ptr<coal::ComputeDistance> compute;
    double distance{};
    bool cached{};
  };
  std::vector<std::unique_ptr<coal::Box>> box_shapes;
  std::vector<CollisionBody> collision_bodies;
  std::vector<DistancePair> distance_pairs;
  Vec geometry_q{Vec::Zero()};
  bool geometry_cached{};
  size_t nearest_pair{};
  double radius_bound{};
  std::string revision;
  Pose toolKinematics(const Vec &q, Jacobian &jacobian);
};
} // namespace z1
