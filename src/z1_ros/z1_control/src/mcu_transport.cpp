#include "IOInterface/IOUDP/IOUDP.h"
#include "udp_observer.hpp"
#include "z1_control/transport.hpp"
#include <dlfcn.h>
#include <fstream>
#include <iomanip>
#include <openssl/evp.h>
#include <sstream>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>
namespace z1 {
namespace {
void verifyLibrary() {
  Dl_info info{};
  void *symbol = dlsym(RTLD_DEFAULT, "_ZN5IOUDPC1EPKcj");
  if (!symbol || !dladdr(symbol, &info))
    throw std::runtime_error("cannot identify vendor IOUDP library");
  std::ifstream f(info.dli_fname, std::ios::binary);
  if (!f)
    throw std::runtime_error("cannot verify vendor binary");
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(),
                                                              EVP_MD_CTX_free);
  if (!ctx || EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("cannot initialize binary fingerprint");
  char bytes[32768];
  while (f) {
    f.read(bytes, sizeof(bytes));
    if (EVP_DigestUpdate(ctx.get(), bytes, f.gcount()) != 1)
      throw std::runtime_error("cannot hash vendor binary");
  }
  if (!f.eof())
    throw std::runtime_error("vendor binary read failed");
  unsigned char hash[EVP_MAX_MD_SIZE];
  unsigned length = 0;
  if (EVP_DigestFinal_ex(ctx.get(), hash, &length) != 1 || length != 32)
    throw std::runtime_error("cannot finalize binary fingerprint");
  std::ostringstream hex;
  for (unsigned i = 0; i < length; ++i)
    hex << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(hash[i]);
  if (hex.str() != Z1_VENDOR_HASH)
    throw std::runtime_error("vendor binary fingerprint mismatch");
}
} // namespace
struct McuTransport::Impl {
  udp::Observer observer;
  std::unique_ptr<IOUDP> io;
  std::unique_ptr<LowLevelState> state;
  uint64_t sequence{};
  unsigned missed{};
  ~Impl() {
    io.reset();
    state.reset();
    for (auto fd : observer.descriptors)
      close(fd);
  }
};
McuTransport::McuTransport(const McuConfig &c) : impl_(new Impl) {
  if (!c.hardware_validated)
    throw std::runtime_error("MCU control requires hardware_validated=true "
                             "after hardware commissioning");
  if (c.receive_budget_us == 0 || c.receive_budget_us > 10000 ||
      !std::isfinite(c.period) || c.period <= 0 ||
      c.receive_budget_us >= c.period * 1e6)
    throw std::runtime_error("invalid MCU receive budget/period");
  verifyLibrary();
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
Exchange McuTransport::exchange(const Output &output, double) {
  auto &p = *impl_;
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
  if (!output.q.allFinite() || !output.dq.allFinite() ||
      !output.tau.allFinite() || !std::isfinite(output.gripper_q) ||
      output.gripper_q < -1 || output.gripper_q > 0)
    throw std::runtime_error("invalid low-level command");
  // Feedback PD is owned by the host core: firmware joint Kp/Kd remain zero.
  if (output.enabled)
    for (int i = 0; i < 6; ++i) {
      cmd.q[i] = output.q[i];
      cmd.dq[i] = output.dq[i];
      cmd.tau[i] = output.tau[i];
    }
  if (output.enabled && p.observer.motor_count == 8) {
    cmd.q[6] = output.gripper_q;
    cmd.Kp[6] = output.gripper_kp;
    cmd.Kd[6] = output.gripper_kd;
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
          f.motor_fault |=
              static_cast<int8_t>(p.observer.state_bytes[offset]) >= 80 ||
              p.observer.state_bytes[offset + 1] != 0 ||
              (p.observer.state_bytes[offset + 2] >> 6) != 0;
        }
      }
    p.missed = 0;
  } else {
    ++p.missed;
    e.error = "no valid new MCU packet";
  }
  if (!e.sent) {
    e.error = "MCU packet send failed";
    e.feedback.transport_fault = true;
  }
  return e;
}
} // namespace z1
