#pragma once
#include "model.hpp"
#include <ruckig/ruckig.hpp>
namespace z1 {
using GoalTrajectory = ruckig::Trajectory<6>;
// Pure trajectory preparation: no controller state, transport or ROS access.
ruckig::InputParameter<6> jointMotionInput(const Config &, const Vec &position,
                                          const Vec &velocity,
                                          const Vec &acceleration);
bool jointPathWithinLimits(const GoalTrajectory &, Model &,
                           std::string *failure = nullptr);
GoalTrajectory planJointGoal(Model &, const Config &, const Vec &start,
                             const Vec &target, double speed = 1);
GoalTrajectory planPoseGoal(Model &, const Config &, const Vec &start,
                            const Pose &target, double speed = 1);
} // namespace z1
