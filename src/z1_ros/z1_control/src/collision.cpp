#include "model_internal.hpp"
#include <coal/distance.h>
#include <limits>
#include <pinocchio/algorithm/kinematics.hpp>

namespace z1 {
Clearance Model::clearance(const Vec &q) { return clearance(q, Vec::Zero()); }

Clearance Model::clearance(const Vec &q, const Vec &joint_displacement) {
  const auto sample = clearanceSample(q, joint_displacement);
  return {sample.distance, std::string(sample.pair)};
}

ClearanceSample Model::clearanceSample(const Vec &q,
                                       const Vec &joint_displacement) {
  auto &p = *impl_;
  if (!p.geometry_cached || (p.geometry_q.array() != q.array()).any()) {
    pinocchio::forwardKinematics(p.model, *p.data, q);
    for (auto &body : p.collision_bodies) {
      body.world = p.data->oMi[body.joint] * body.placement;
      const Eigen::Vector3d center = body.world.act(body.center);
      const Eigen::Vector3d half =
          body.world.rotation().cwiseAbs() * body.half_size;
      body.world_aabb = coal::AABB(center - half, center + half);
    }
    for (auto &pair : p.distance_pairs)
      pair.cached = false;
    p.geometry_q = q;
    p.geometry_cached = true;
  }
  const Vec displacement = joint_displacement.cwiseAbs();
  double minimum = std::numeric_limits<double>::infinity();
  size_t nearest = p.nearest_pair;
  auto query = [&](size_t index) {
    auto &pair = p.distance_pairs[index];
    const auto &a = p.collision_bodies[pair.first];
    const auto &b = p.collision_bodies[pair.second];
    const double allowance = pair.reach.dot(displacement);
    if (!pair.cached) {
      const double bound = a.world_aabb.distance(b.world_aabb);
      // Overlapping AABBs give no lower bound on signed penetration depth.
      if (bound > 0 && bound - allowance >= minimum)
        return;
      coal::DistanceRequest request;
      request.enable_signed_distance = true;
      coal::DistanceResult result;
      pair.distance = (*pair.compute)(
          coal::Transform3s(a.world.rotation(), a.world.translation()),
          coal::Transform3s(b.world.rotation(), b.world.translation()), request,
          result);
      if (!std::isfinite(pair.distance))
        throw std::runtime_error("nonfinite collision distance");
      pair.cached = true;
    }
    if (pair.distance - allowance < minimum) {
      minimum = pair.distance - allowance;
      nearest = index;
    }
  };
  // Seed pruning with the previous closest pair, without changing the result.
  query(p.nearest_pair);
  for (size_t i = 0; i < p.distance_pairs.size(); ++i)
    if (i != p.nearest_pair)
      query(i);
  p.nearest_pair = nearest;
  return {minimum, p.distance_pairs[nearest].name};
}

double Model::distanceMotionBound(const Vec &joint_motion) const {
  const Vec motion = joint_motion.cwiseAbs();
  double bound = 0;
  for (const auto &pair : impl_->distance_pairs)
    bound = std::max(bound, pair.reach.dot(motion));
  return bound;
}
} // namespace z1
