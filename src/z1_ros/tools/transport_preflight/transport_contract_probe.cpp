// Offline contract probe: real vendor IOUDP, fake UdpPort, no robot or socket.
// This is a validation tool, not a transport adapter for the control runtime.
#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "IOInterface/IOUDP/IOUDP.h"

namespace {
struct Scenario {
  int protocol = 1;
  bool handshake = true;
  IOPortStatus receive_status = IOPortStatus::ok;
  IOPortStatus send_status = IOPortStatus::ok;
  std::size_t port_constructions = 0;
  std::size_t injected_samples = 0;
  double position = 0.125;
};

Scenario scenario;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

std::size_t inject_sample(uint8_t* output, std::size_t capacity) {
  // Six joints, seven motors; no gripper in this fixture.
  if (scenario.protocol == 1) {
    std::array<JointStateV1, 6> sample{};
    for (auto& joint : sample) joint.Pos = static_cast<float>(scenario.position);
    require(capacity >= sizeof(sample), "V1 receive buffer too small");
    std::memcpy(output, sample.data(), sizeof(sample));
    ++scenario.injected_samples;
    return sizeof(sample);
  }
  std::array<JointStateV2, 6> sample{};
  for (auto& joint : sample) {
    joint.Pos = static_cast<float>(scenario.position);
    for (auto& motor : joint.state) {
      motor.temperature = 25;
      motor.isConnected.state = 0;
    }
  }
  require(capacity >= sizeof(sample), "V2 receive buffer too small");
  std::memcpy(output, sample.data(), sizeof(sample));
  ++scenario.injected_samples;
  return sizeof(sample);
}

void probe_protocol(int protocol) {
  scenario = Scenario{};
  scenario.protocol = protocol;
  IOUDP io("127.0.0.1", 0);
  require(scenario.port_constructions == 1, "fake UdpPort constructor not used");
  require(io.init() && io.initialized(), "fixture handshake failed");
  require(!io.hasGripper, "fixture should report no gripper");

  LowLevelCmd command{};
  command.setPassive();
  LowLevelState state(0.004);

  // Exercise a successful receive first, so later failures test a cached sample.
  scenario.position = 0.375;
  const auto sample_count = scenario.injected_samples;
  require(io.sendRecv(&command, &state), "initial exchange returned false");
  require(scenario.injected_samples == sample_count + 1, "no fresh sample injected");
  require(state.q.front() != 0.0, "real IOUDP did not decode fixture feedback");
  const auto cached_q = state.q;
  const auto cached_dq = state.dq;
  const auto cached_tau = state.tau;

  for (const auto status : {IOPortStatus::timeout, IOPortStatus::crcbad,
                            IOPortStatus::msgbad, IOPortStatus::err}) {
    scenario.receive_status = status;
    const auto before = scenario.injected_samples;
    const bool reported = io.sendRecv(&command, &state);
    require(scenario.injected_samples == before, "failed receive injected data");
    require(reported, "vendor contract changed: failed receive no longer returns true");
    require(state.q == cached_q && state.dq == cached_dq && state.tau == cached_tau,
            "failed receive changed raw cached feedback");
    std::cout << "V" << protocol << " receive=" << status
              << ": sendRecv=true, raw feedback unchanged\n";
  }

  scenario.send_status = IOPortStatus::err;
  require(io.sendRecv(&command, &state), "vendor send failure contract changed");
  std::cout << "V" << protocol
            << " send=err/receive=err: sendRecv=true\n";

  // A new packet can contain identical measurements on a stationary robot.
  scenario.receive_status = IOPortStatus::ok;
  scenario.send_status = IOPortStatus::ok;
  const auto before = scenario.injected_samples;
  require(io.sendRecv(&command, &state), "recovered receive returned false");
  require(scenario.injected_samples == before + 1, "new identical sample not injected");
  require(state.q == cached_q && state.dq == cached_dq && state.tau == cached_tau,
          "identical packet changed raw feedback");
  std::cout << "V" << protocol
            << " new identical packet: raw feedback unchanged\n";
}
}  // namespace

// Symbol interposition is deliberately limited to this offline executable.
// The real IOUDP constructor calls this UdpPort constructor through its PLT.
UdpPort::UdpPort(std::string name, uint, std::string, uint, std::size_t timeout_ms)
    : IOPort(std::move(name), timeout_ms) {
  ++scenario.port_constructions;
  isConnect_ = true;
}

UdpPort::UdpPort(std::string name, uint, std::size_t timeout_ms)
    : IOPort(std::move(name), timeout_ms) {
  ++scenario.port_constructions;
  isConnect_ = true;
}

IOPortStatus UdpPort::send(uint8_t*, std::size_t) {
  return scenario.send_status;
}

std::size_t UdpPort::recvLen(uint8_t* output, std::size_t capacity) {
  if (!scenario.handshake) return 0;
  if (capacity == 2) {
    output[0] = 0x20;
    output[1] = 7;
    return 2;
  }
  return inject_sample(output, capacity);
}

IOPortStatus UdpPort::recv(uint8_t* output, std::size_t capacity) {
  if (scenario.receive_status != IOPortStatus::ok) return scenario.receive_status;
  if (capacity == 9) {
    // The real IOUDP init checks the seven motor connection bytes at offset 2.
    std::fill(output, output + capacity, uint8_t{1});
    return IOPortStatus::ok;
  }
  require(inject_sample(output, capacity) == capacity, "unexpected receive length");
  return IOPortStatus::ok;
}

// Abort if any vendor code bypasses the fake port and tries to open a socket.
// This guard is exported only by the offline probe, never by a runtime adapter.
extern "C" int socket(int, int, int) noexcept {
  std::cerr << "FAIL: offline probe attempted to open a socket\n";
  std::abort();
}

int main() {
  try {
    static_assert(sizeof(JointCmd) == 20, "unexpected command packing");
    static_assert(sizeof(JointStateV1) == 16, "unexpected V1 packing");
    static_assert(sizeof(JointStateV2) == 22, "unexpected V2 packing");
    probe_protocol(1);
    probe_protocol(2);

    scenario = Scenario{};
    scenario.handshake = false;
    IOUDP disconnected("127.0.0.1", 0);
    require(!disconnected.init() && !disconnected.initialized(),
            "absent fixture handshake should fail init");
    std::cout << "Absent handshake: init=false\n"
              << "PASS: offline vendor contract reproduced; no sockets opened.\n"
              << "Hardware transport and feedback freshness remain unverified.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
