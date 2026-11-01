#include "z1_control/friction.hpp"
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
namespace z1 {
void validateFrictionConfig(const FrictionConfig &c) {
  if (!c.coulomb.allFinite() || !c.viscous.allFinite() ||
      (c.coulomb.array() < 0).any() || (c.viscous.array() < 0).any() ||
      !std::isfinite(c.velocity_scale) || c.velocity_scale <= 0 ||
      !std::isfinite(c.impedance_velocity_filter) ||
      c.impedance_velocity_filter <= 0 ||
      !std::isfinite(c.impedance_torque_rate) || c.impedance_torque_rate <= 0 ||
      c.has_gripper < -1 || c.has_gripper > 1)
    throw std::invalid_argument("invalid joint-side friction parameters");
}
FrictionConfig loadFrictionConfig(const std::string &path,
                                  const std::string &arm) {
  if (arm != "left" && arm != "right")
    throw std::invalid_argument("friction arm must be left or right");
  const auto root = YAML::LoadFile(path);
  if (root["schema_version"].as<int>() != 1)
    throw std::invalid_argument("unsupported friction schema_version");
  FrictionConfig c;
  c.enabled = root["enabled"].as<bool>();
  c.velocity_scale = root["velocity_scale_rad_s"].as<double>();
  c.impedance_velocity_filter =
      root["impedance_velocity_filter_s"].as<double>();
  c.impedance_torque_rate = root["impedance_torque_rate_nm_s"].as<double>();
  const auto values = root["arms"][arm];
  for (auto item : {std::make_pair("coulomb_nm", &c.coulomb),
                    std::make_pair("viscous_nm_s_per_rad", &c.viscous)}) {
    const auto a = values[item.first];
    if (!a.IsSequence() || a.size() != 6)
      throw std::invalid_argument(
          "friction coefficient must contain six numbers");
    for (int i = 0; i < 6; ++i)
      (*item.second)[i] = a[i].as<double>();
  }
  if (values["has_gripper"] && !values["has_gripper"].IsNull())
    c.has_gripper = values["has_gripper"].as<bool>() ? 1 : 0;
  c.model_revision = values["model_revision"].as<std::string>("");
  validateFrictionConfig(c);
  return c;
}
Vec frictionFeedforward(const FrictionConfig &c, const Vec &velocity) {
  if (!c.enabled)
    return Vec::Zero();
  return (c.coulomb.array() * (velocity.array() / c.velocity_scale).tanh() +
          c.viscous.array() * velocity.array())
      .matrix();
}
} // namespace z1
