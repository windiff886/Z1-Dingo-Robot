#pragma once
#include "types.hpp"
#include <array>

namespace z1 {
// All times are steady-clock seconds. Cartesian derivatives are base-frame
// linear/angular velocity and acceleration at the TCP, ordered XYZ/rotation
// XYZ.
struct JointReference {
  double time{};
  Vec position{Vec::Zero()}, velocity{Vec::Zero()}, acceleration{Vec::Zero()};
  bool has_velocity{}, has_acceleration{};
};
struct CartesianReference {
  double time{};
  Pose pose{Pose::Identity()};
  Vec velocity{Vec::Zero()}, acceleration{Vec::Zero()};
  bool has_velocity{}, has_acceleration{};
};

// A small time horizon, not an unbounded command queue. New points replace
// previously scheduled points at or after their timestamp.
template <class Reference> class ReferenceTimeline {
public:
  void reset(const Reference &initial);
  bool push(Reference, double velocity_filter_time);
  Reference sample(double time, double max_extrapolation) const;
  void prune(double time);
  double endTime() const { return count_ ? points_[count_ - 1].time : 0; }

private:
  std::array<Reference, 32> points_;
  size_t count_{};
};
extern template class ReferenceTimeline<JointReference>;
extern template class ReferenceTimeline<CartesianReference>;
} // namespace z1
