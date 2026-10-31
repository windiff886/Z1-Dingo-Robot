#include "IOInterface/IOUDP/IOUDP.h"
#include "udp_observer.hpp"
#include "z1_control/transport.hpp"
#include <sstream>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>
namespace z1 {
struct McuTransport::Impl {
  udp::Observer observer;
  std::unique_ptr<IOUDP> io;
  std::unique_ptr<LowLevelState> state;
  uint64_t sequence{};
  bool passive_feedback{}, calibration_attempted{};
  ~Impl() {
    io.reset();
    for (auto fd : observer.descriptors)
      close(fd);
  }
};
McuTransport::McuTransport(const McuConfig &c) : impl_(new Impl) {
  if (c.receive_budget_us == 0 || !std::isfinite(c.period) || c.period <= 0 ||
      c.receive_budget_us >= c.period * 1e6)
    throw std::runtime_error("invalid MCU receive budget/period");
  auto &p = *impl_;
  p.observer.budget_us = 30000;
  udp::Scope scope(p.observer);
  p.state.reset(new LowLevelState(c.period));
  p.io.reset(new IOUDP(c.ip.c_str(), c.local_port));
  if (!p.io->init() ||
      (p.observer.motor_count != 7 && p.observer.motor_count != 8) ||
      p.observer.protocol == 0)
    throw std::runtime_error(
        "MCU handshake failed or unsupported protocol/motor count");
  p.observer.budget_us = c.receive_budget_us;
}
McuTransport::~McuTransport() = default;
Vec McuTransport::filteredVelocity() const {
  return impl_->state->getDqFiltered();
}
Exchange McuTransport::exchange(const Output &output, double) {
  auto &p = *impl_;
  p.passive_feedback = false;
  udp::Scope scope(p.observer);
  p.observer.fresh = false;
  p.observer.sent = true;
  // Drop packets queued before this request rather than retimestamping an old
  // backlog as fresh.
  for (int fd : p.observer.descriptors) {
    uint8_t old[1500];
    unsigned drained = 0;
    while (drained < 32 && recv(fd, old, sizeof(old), MSG_DONTWAIT) >= 0)
      ++drained;
    if (drained == 32) {
      Exchange error;
      error.error = "receive backlog limit exceeded";
      error.feedback.transport_fault = true;
      return error;
    }
  }
  LowLevelCmd cmd;
  // Firmware position gains and feedforward torque are dispatched to MCU.
  if (output.enabled) {
    if (!output.q.allFinite() || !output.dq.allFinite() ||
        !output.tau.allFinite() || !output.kp.allFinite() ||
        !output.kd.allFinite())
      throw std::runtime_error("nonfinite low-level arm command");
    for (int i = 0; i < 6; ++i) {
      cmd.q[i] = output.q[i];
      cmd.dq[i] = output.dq[i];
      cmd.tau[i] = output.tau[i];
      cmd.Kp[i] = output.kp[i];
      cmd.Kd[i] = output.kd[i];
    }
    if (p.observer.motor_count == 8) {
      if (!std::isfinite(output.gripper_q) || output.gripper_q < -1 ||
          output.gripper_q > 0) {
        std::ostringstream detail;
        detail << "invalid gripper command: position=" << output.gripper_q
               << " rad, allowed range=[-1, 0]";
        throw std::runtime_error(detail.str());
      }
      cmd.q[6] = output.gripper_q;
      cmd.Kp[6] = output.gripper_kp;
      cmd.Kd[6] = output.gripper_kd;
    }
  }
  p.io->sendRecv(&cmd, p.state.get());
  Exchange e;
  e.sent = p.observer.sent;
  if (p.observer.fresh) {
    Feedback &f = e.feedback;
    f.fresh = true;
    f.sequence = ++p.sequence;
    f.received_at =
        std::chrono::duration<double>(p.observer.received.time_since_epoch())
            .count();
    for (int i = 0; i < 6; ++i) {
      f.q[i] = p.state->q[i];
      f.dq[i] = p.state->dq[i];
      f.tau[i] = p.state->tau[i];
    }
    f.has_gripper = p.observer.motor_count == 8;
    f.gripper_q = p.state->q[6];
    f.gripper_dq = p.state->dq[6];
    f.gripper_tau = p.state->tau[6];
    // Public arrays expose device errors; V1 has no motor status information.
    if (p.observer.protocol == 2)
      for (unsigned joint = 0; joint < (f.has_gripper ? 7u : 6u); ++joint) {
        for (unsigned motor = 0; motor < (joint == 1 ? 2u : 1u); ++motor) {
          auto offset = joint * 22 + 16 + motor * 3;
          const int temperature =
              static_cast<int8_t>(p.observer.state_bytes[offset]);
          const unsigned error = p.observer.state_bytes[offset + 1];
          const unsigned connection = p.observer.state_bytes[offset + 2] >> 6;
          // Match this vendor version's LowLevelState::checkError() and
          // msgUdpToSerial.h. In particular, its observed 0x40 status is not
          // classified as a motor error by the official library.
          constexpr unsigned motor_error_mask = 0x01 | 0x02 | 0x04 | 0x20;
          if (temperature >= 80 || (error & motor_error_mask) != 0 ||
              connection != 0) {
            f.motor_fault = true;
            if (e.error.empty()) {
              std::ostringstream detail;
              detail << "MCU motor fault: joint=" << joint + 1
                     << " motor=" << motor + 1
                     << " temperature=" << temperature << " error=0x"
                     << std::hex << error << std::dec
                     << " connection=" << connection;
              e.error = detail.str();
            }
          }
        }
      }
  } else {
    e.error = "no valid new MCU packet";
  }
  if (!e.sent) {
    e.error = "MCU packet send failed";
    e.feedback.transport_fault = true;
  }
  p.passive_feedback = !output.enabled && e.sent && e.feedback.fresh &&
                       !e.feedback.motor_fault && !e.feedback.transport_fault &&
                       e.feedback.q.allFinite() && e.feedback.dq.allFinite();
  return e;
}
CalibrationReply McuTransport::calibrate(unsigned reply_timeout_us) {
  auto &p = *impl_;
  const double age = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - p.observer.received)
                         .count();
  if (reply_timeout_us == 0)
    return {CalibrationCode::NotAttempted, "reply timeout must be positive"};
  if (p.calibration_attempted)
    return {CalibrationCode::NotAttempted,
            "calibration already attempted on this connection"};
  if (!p.passive_feedback || age > 0.25)
    return {CalibrationCode::NotAttempted,
            "recent valid passive feedback is required"};

  // Discard queued feedback/old replies before the single maintenance request.
  for (int fd : p.observer.descriptors) {
    uint8_t bytes[1500];
    unsigned drained = 0;
    while (drained < 32 && recv(fd, bytes, sizeof(bytes), MSG_DONTWAIT) >= 0)
      ++drained;
    if (drained == 32)
      return {CalibrationCode::NotAttempted, "receive backlog limit exceeded"};
  }
  udp::Scope scope(p.observer);
  struct RestoreBudget {
    udp::Observer &observer;
    unsigned previous;
    ~RestoreBudget() { observer.budget_us = previous; }
  } restore{p.observer, p.observer.budget_us};
  p.observer.budget_us = reply_timeout_us;
  p.observer.fresh = false;
  p.observer.sent = true;
  p.passive_feedback = false;
  p.calibration_attempted = true;
  const bool accepted = p.io->calibration();
  // The vendor ignores the receive return code. Require a new, exact-length
  // reply from the MCU endpoint as well as its boolean result.
  if (!p.observer.sent || !p.observer.fresh || p.observer.last_length != 1)
    return {CalibrationCode::Unknown,
            "no valid calibration reply; do not automatically retry"};
  if (accepted && p.observer.state_bytes[0] == 0x10)
    return {CalibrationCode::Succeeded, "MCU acknowledged calibration"};
  if (p.observer.state_bytes[0] == 0xff)
    return {CalibrationCode::DeviceRejected,
            "MCU reported a calibration error"};
  return {CalibrationCode::Unknown,
          "unexpected calibration reply; do not automatically retry"};
}
} // namespace z1
