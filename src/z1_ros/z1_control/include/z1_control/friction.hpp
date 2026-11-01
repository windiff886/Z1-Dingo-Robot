#pragma once
#include "types.hpp"
namespace z1 {
// Loads one arm from the shared YAML; no hardware or ROS access.
FrictionConfig loadFrictionConfig(const std::string &path,
                                  const std::string &arm);
void validateFrictionConfig(const FrictionConfig &);
Vec frictionFeedforward(const FrictionConfig &, const Vec &joint_velocity);
} // namespace z1
