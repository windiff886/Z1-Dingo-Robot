#pragma once
#include "unitree_arm_sdk/control/unitreeArm.h"
#include <memory>
#include <cmath>
#include <stdexcept>

namespace z1_official {
// Initialization from upstream's multi-robot example / hasGripper constructor.
// unitreeArm owns these components. All algorithms remain in the official library.
inline UNITREE_ARM::CtrlComponents* components(unsigned controller_port, unsigned sdk_port,
                                              bool has_gripper, double load_kg = 0.0) {
  using namespace UNITREE_ARM;
  if (!controller_port || controller_port > 65535 || !sdk_port || sdk_port > 65535 ||
      controller_port == sdk_port || !std::isfinite(load_kg) || load_kg < 0)
    throw std::invalid_argument("invalid official SDK ports or load");
  auto c = std::unique_ptr<CtrlComponents>(new CtrlComponents());
  c->udp = nullptr;
  c->armModel = nullptr;
  c->recvState = {};
  c->dt = 0.002;
  c->udp = new UDPPort("127.0.0.1", controller_port, sdk_port,
                       RECVSTATE_LENGTH, BlockYN::NO, 500000);
  if (has_gripper)
    c->armModel = new Z1Model(Vec3(0.0382, 0.0, 0.0), 0.80225,
        Vec3(0.0037, 0.0014, -0.0003),
        Vec3(0.00057593, 0.00099960, 0.00106337).asDiagonal());
  else
    c->armModel = new Z1Model();
  c->armModel->addLoad(load_kg);
  return c.release();
}
}  // namespace z1_official
