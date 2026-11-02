// Test-only hardware boundary. The real upstream FSM/model and SDK UDP remain.
// No MCU socket is created. Not installed or linked into production binaries.
#include "interface/IOUDP.h"
#include <algorithm>
#include <cstdlib>
#include <string>

namespace {
class MemoryPort : public IOPort {
 public:
  MemoryPort() : IOPort("test-only", BlockYN::NO, 0, 1, false) {}
  size_t send(uint8_t*, size_t) override { return 0; }
  size_t recv(uint8_t*, size_t) override { return 0; }
  size_t recv(uint8_t*) override { return 0; }
};
}

IOUDP::IOUDP(const char* ip, uint, size_t, bool) {
  const std::string address(ip);
  if (address != "127.0.0.2" && address != "127.0.0.3") std::abort();
  hasGripper = address == "127.0.0.2";
  hasErrorState = false;
  _jointNum = hasGripper ? 7 : 6;
  _motorNum = hasGripper ? 8 : 7;
  _ioPort = new MemoryPort();
  lowCmd = new LowlevelCmd();
  lowState = new LowlevelState(.004);
  if (hasGripper) {
    lowCmd->resizeGripper();
    lowState->resizeGripper(.004);
  }
}
IOUDP::~IOUDP() { delete _ioPort; }
bool IOUDP::calibration() { return false; }
bool IOUDP::sendRecv(const LowlevelCmd*, LowlevelState* state) {
  const double q[] = {hasGripper ? .125 : -.25, .8, -1.1, .3, 0, 0, 0};
  for (size_t i = 0; i < state->q.size(); ++i) {
    state->q[i] = q[i];
    state->dq[i] = state->ddq[i] = state->tau[i] = 0;
  }
  std::fill(state->temperature.begin(), state->temperature.end(), 25);
  std::fill(state->errorstate.begin(), state->errorstate.end(), 0);
  std::fill(state->isMotorConnected.begin(), state->isMotorConnected.end(), 0);
  state->runFilter();
  return true;
}
