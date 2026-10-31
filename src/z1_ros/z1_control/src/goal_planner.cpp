#include "z1_control/goal_planner.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace z1 {
namespace {
std::array<double, 6> array(const Vec &v) {
  std::array<double, 6> out;
  std::copy(v.data(), v.data() + 6, out.begin());
  return out;
}
Vec vector(const std::array<double, 6> &v) {
  return Eigen::Map<const Vec>(v.data());
}
} // namespace
ruckig::InputParameter<6> jointMotionInput(const Config &c, const Vec &q,
                                          const Vec &v, const Vec &a) {
  ruckig::InputParameter<6> in;
  in.current_position = array(q);
  in.current_velocity = array(v);
  in.current_acceleration = array(a);
  in.max_velocity = array(c.velocity);
  in.max_acceleration = array(c.acceleration);
  in.max_jerk = array(c.jerk);
  return in;
}
bool jointPathWithinLimits(const GoalTrajectory &t, Model &model,
                                       std::string *failure) {
  auto fail = [failure](const std::string &reason) {
    if (failure)
      *failure = reason;
    return false;
  };
  double end = t.get_duration();
  if (!std::isfinite(end) || end < 0)
    return fail("invalid trajectory duration");
  // On Ruckig 0.9.2 zero-duration profiles contain unspecified intermediate
  // positions. Only evaluate initialized, positive-duration polynomial
  // segments.
  std::array<double, 6> endpoint, endpoint_v, endpoint_a;
  t.at_time(0, endpoint, endpoint_v, endpoint_a);
  if (!model.withinLimits(vector(endpoint)))
    return fail("trajectory start outside joint limits");
  t.at_time(end, endpoint, endpoint_v, endpoint_a);
  if (!model.withinLimits(vector(endpoint)))
    return fail("trajectory end outside joint limits");
  if (end > 0)
    for (const auto &section : t.get_profiles())
      for (int axis = 0; axis < 6; ++axis) {
        const auto &p = section[axis];
        auto segment = [&](double h, double pos, double vel, double accel,
                           double jerk) {
          auto point = [&](double x) {
            double q =
                pos + vel * x + 0.5 * accel * x * x + jerk * x * x * x / 6;
            return q >= model.lower()[axis] - 1e-10 &&
                   q <= model.upper()[axis] + 1e-10;
          };
          if (!point(0) || !point(h))
            return false;
          if (std::abs(jerk) < 1e-14) {
            if (std::abs(accel) > 1e-14) {
              double root = -vel / accel;
              if (root > 0 && root < h && !point(root))
                return false;
            }
          } else {
            double discriminant = accel * accel - 2 * jerk * vel;
            if (discriminant >= 0) {
              for (double root : {(-accel + std::sqrt(discriminant)) / jerk,
                                  (-accel - std::sqrt(discriminant)) / jerk})
                if (root > 0 && root < h && !point(root))
                  return false;
            }
          }
          return true;
        };
        if (p.brake.duration > 0)
          for (int i = 0; i < 2; ++i)
            if (p.brake.t[i] > 0 &&
                !segment(p.brake.t[i], p.brake.p[i], p.brake.v[i], p.brake.a[i],
                         p.brake.j[i]))
              return fail("braking polynomial outside joint limits");
        for (int i = 0; i < 7; ++i)
          if (p.t[i] > 0 && !segment(p.t[i], p.p[i], p.v[i], p.a[i], p.j[i]))
            return fail("trajectory polynomial outside joint limits");
      }
  return true;
}
GoalTrajectory planJointGoal(Model &model, const Config &c, const Vec &start,
                             const Vec &target, double speed) {
  if (!std::isfinite(speed) || speed <= 0 || speed > 1)
    throw std::invalid_argument("speed must be in (0,1]");
  if (!model.withinLimits(start) || !model.withinLimits(target))
    throw std::invalid_argument("joint target or start outside limits");
  if (!std::isfinite(c.period) || c.period <= 0)
    throw std::invalid_argument("control period must be positive");
  for (const auto *limit : {&c.velocity, &c.acceleration, &c.jerk})
    if (!limit->allFinite() || (limit->array() <= 0).any())
      throw std::invalid_argument("motion limits must be finite and positive");
  if ((c.velocity.array() > model.velocityLimit().array()).any())
    throw std::invalid_argument("velocity exceeds model limit");
  auto in = jointMotionInput(c, start, Vec::Zero(), Vec::Zero());
  in.target_position = array(target);
  for (auto &v : in.max_velocity)
    v *= speed;
  ruckig::Ruckig<6> planner(c.period);
  GoalTrajectory trajectory;
  if (planner.calculate(in, trajectory) != ruckig::Result::Working)
    throw std::runtime_error("finite trajectory generator error");
  std::string reason;
  if (!jointPathWithinLimits(trajectory, model, &reason))
    throw std::runtime_error(reason);
  return trajectory;
}
GoalTrajectory planPoseGoal(Model &model, const Config &c, const Vec &start,
                            const Pose &target, double speed) {
  Vec joint_target;
  std::string reason;
  if (!model.ik(target, start, joint_target, c.joint_limit_tolerance, &reason))
    throw std::invalid_argument(reason);
  return planJointGoal(model, c, start, joint_target, speed);
}
} // namespace z1
