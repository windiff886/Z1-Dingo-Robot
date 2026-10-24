#pragma once

#include "z1_control/types.hpp"
#include <array>
#include <vector>

namespace z1 {

struct TrajectoryPoint {
  double time_from_start{};
  Vec position{Vec::Zero()}, velocity{Vec::Zero()}, acceleration{Vec::Zero()};
  bool has_velocity{}, has_acceleration{};
};

struct TrajectorySample {
  Vec position{Vec::Zero()}, velocity{Vec::Zero()}, acceleration{Vec::Zero()},
      jerk{Vec::Zero()};
};

// Prepared, C2-continuous quintic Hermite segments. Construction verifies the
// polynomial extrema; sampling only evaluates the selected segment.
class JointTrajectory {
public:
  // Missing endpoint derivatives are zero. Missing interior velocity is a
  // limited secant estimate; missing acceleration is zero on both sides.
  // retime=false preserves all times and explicit derivatives, or rejects.
  static JointTrajectory timed(const std::vector<TrajectoryPoint> &points,
                               const Config &config, const Vec &lower,
                               const Vec &upper, bool retime = false);
  // speed is a common execution speed scale in (0,1], applied to v/a/jerk.
  static JointTrajectory waypoints(const std::vector<Vec> &points,
                                   const Config &config, const Vec &lower,
                                   const Vec &upper, double speed = 1);
  // Outside the closed trajectory interval, hold the endpoint with zero
  // derivatives. At exact knots the specified derivatives are retained.
  TrajectorySample sample(double time) const;
  // Additional slowdown, preserving the normalized polynomial geometry.
  JointTrajectory scaled(double factor) const;
  double duration() const;
  double timeScale() const { return time_scale_; }
  const std::vector<double> &times() const { return times_; }
  bool empty() const { return segments_.empty(); }

private:
  struct Segment {
    // Position coefficients in normalized segment time s in [0, 1].
    std::array<Vec, 6> coefficients;
    double duration{};
  };
  std::vector<Segment> segments_;
  std::vector<double> times_;
  double time_scale_{1};
};

} // namespace z1
