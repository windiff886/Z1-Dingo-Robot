#pragma once

#include "z1_control/model.hpp"
#include "z1_control/trajectory.hpp"

namespace z1 {

struct LinearPathOptions {
  double speed{1};
  // Maximum distance trimmed from either side of an intermediate waypoint.
  // Distance is max(translation metres, 0.24 * rotation radians), so pure
  // rotations can also blend. Translation stays inside the trimmed corner's
  // convex hull; zero visits every waypoint at rest.
  double blend_radius{0};
  double translation_tolerance{0.0001};
  double rotation_tolerance{0.001};
};

// Offline MoveL preparation: Cartesian interpolation, continuous local IK,
// checked joint polynomial fitting, then global joint-limit retiming. Sampling
// the returned trajectory needs no IK. No collision detours are introduced.
JointTrajectory planLinearPath(Model &model, const Vec &start,
                               const std::vector<Pose> &targets,
                               const Config &config,
                               const LinearPathOptions &options = {});

} // namespace z1
