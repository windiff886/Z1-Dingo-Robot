#include "z1_control/transport.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <atomic>
#include <cstring>
#include <iostream>
#include <limits>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
using namespace z1;
void require(bool b, const char *why) {
  if (!b)
    throw std::runtime_error(why);
}
double now() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
class Firmware {
public:
  int fd;
  std::thread thread;
  std::atomic<bool> running{true}, drop{false}, malformed{false};
  std::atomic<bool> oversized{false};
  std::atomic<bool> foreign{false}, motor_error{false}, hot{false},
      disconnected{false};
  std::atomic<unsigned> error_bits{1};
  int protocol, joints;
  bool combined_handshake;
  std::atomic<unsigned> commands{0};
  std::atomic<bool> active_command{false};
  std::atomic<unsigned> calibrations{0};
  Firmware(int v, int n, bool combined = true)
      : protocol(v), joints(n), combined_handshake(combined) {
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    require(fd >= 0, "loopback socket unavailable");
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(8880);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) < 0) {
      close(fd);
      throw std::runtime_error("loopback port 8880 unavailable");
    }
    thread = std::thread([this] { run(); });
  }
  ~Firmware() {
    running = false;
    if (thread.joinable())
      thread.join();
    close(fd);
  }
  void run() {
    int stage = 0;
    while (running) {
      pollfd p{fd, POLLIN, 0};
      if (poll(&p, 1, 10) <= 0)
        continue;
      uint8_t bytes[1500];
      sockaddr_in source{};
      socklen_t len = sizeof(source);
      auto n = recvfrom(fd, bytes, sizeof(bytes), 0,
                        reinterpret_cast<sockaddr *>(&source), &len);
      if (drop)
        continue;
      std::array<uint8_t, 160> reply{};
      size_t size{};
      if (n == 1 && bytes[0] == 1) {
        ++calibrations;
        reply[0] = 0x10;
        size = 1;
      } else if (n == 1) {
        if (combined_handshake) {
          reply[0] = 0x20;
          reply[1] = joints == 7 ? 8 : 7;
          size = reply[1] + 2;
          std::fill(reply.begin() + 2, reply.begin() + size, 1);
        } else if (stage % 2 == 0) {
          reply[0] = 0x20;
          reply[1] = joints == 7 ? 8 : 7;
          size = 2;
        } else {
          size = (joints == 7 ? 8 : 7) + 2;
          std::fill(reply.begin(), reply.begin() + size, 1);
        }
        ++stage;
      } else {
        ++commands;
        if (std::any_of(bytes, bytes + n, [](uint8_t byte) { return byte != 0; }))
          active_command = true;
        size = joints * (protocol == 1 ? 16 : 22);
        for (int j = 0; j < joints; ++j) {
          const size_t stride = protocol == 1 ? 16 : 22;
          float torque = static_cast<float>(-0.75 - j * 0.125);
          float velocity = static_cast<float>(0.5 + j * 0.25);
          float acceleration = static_cast<float>(5.0 + j);
          float pos = static_cast<float>(0.125 + j * 0.01);
          memcpy(reply.data() + j * stride, &torque, 4);
          memcpy(reply.data() + j * stride + 4, &velocity, 4);
          memcpy(reply.data() + j * stride + 8, &acceleration, 4);
          memcpy(reply.data() + j * stride + 12, &pos, 4);
          if (protocol == 2) {
            reply[j * 22 + 16] = 25;
            reply[j * 22 + 19] = 25;
            if (j == 0) {
              if (motor_error)
                reply[j * 22 + 17] = error_bits;
              if (hot)
                reply[j * 22 + 16] = 85;
              if (disconnected)
                reply[j * 22 + 18] = 64;
            }
          }
        }
        if (malformed)
          size -= 1;
        if (oversized)
          size += 1;
      }
      int sender = foreign ? socket(AF_INET, SOCK_DGRAM, 0) : fd;
      sendto(sender, reply.data(), size, 0,
             reinterpret_cast<sockaddr *>(&source), len);
      if (sender != fd)
        close(sender);
    }
  }
};
int main() {
  try {
    McuConfig cfg;
    cfg.ip = "127.0.0.1";
    cfg.local_port = 0;
    // This codec/loopback test uses a generous receive window, independently
    // of the production 2 ms control period.
    cfg.period = 0.01;
    cfg.receive_budget_us = 5000;
    for (bool combined : {true, false})
      for (int version : {1, 2})
        for (int joints : {6, 7}) {
          Firmware server(version, joints, combined);
          McuTransport driver(cfg);
          Output command;
          auto a = driver.exchange(command, now());
          require(a.sent && a.feedback.fresh, "fresh feedback missing");
          for (int joint = 0; joint < 6; ++joint) {
            require(std::abs(a.feedback.q[joint] - (0.125 + joint * 0.01)) < 1e-6,
                    "position field or joint index decoded incorrectly");
            require(std::abs(a.feedback.dq[joint] - (0.5 + joint * 0.25)) < 1e-6,
                    "velocity field or joint index decoded incorrectly");
            require(std::abs(a.feedback.tau[joint] - (-0.75 - joint * 0.125)) < 1e-6,
                    "torque field or joint index decoded incorrectly");
          }
          require(a.feedback.has_gripper == (joints == 7),
                  "gripper detection failed");
          if (joints == 7) {
            require(std::abs(a.feedback.gripper_q - 0.185) < 1e-6 &&
                        std::abs(a.feedback.gripper_dq - 2.0) < 1e-6 &&
                        std::abs(a.feedback.gripper_tau + 1.5) < 1e-6,
                    "gripper feedback mixed with main arm joints");
          }
          Output passive;
          passive.q[0] = std::numeric_limits<double>::quiet_NaN();
          passive.gripper_q = 1;
          auto ignored = driver.exchange(passive, now());
          require(ignored.sent && ignored.feedback.fresh && !server.active_command,
                  "disabled output fields changed the passive wire command");
          Output active = passive;
          active.enabled = true;
          bool invalid_arm_rejected = false;
          try {
            driver.exchange(active, now());
          } catch (const std::runtime_error &) {
            invalid_arm_rejected = true;
          }
          require(invalid_arm_rejected, "nonfinite enabled arm command accepted");
          active.q.setZero();
          bool invalid_gripper_rejected = false;
          try {
            driver.exchange(active, now());
          } catch (const std::runtime_error &) {
            invalid_gripper_rejected = true;
          }
          require(invalid_gripper_rejected == (joints == 7),
                  "gripper validation did not match the connected hardware");
          auto b = driver.exchange(command, now());
          require(b.feedback.fresh && b.feedback.sequence > a.feedback.sequence,
                  "identical values treated as stale");
          require(b.feedback.received_at >= a.feedback.received_at,
                  "receive timestamp not monotonic");
          server.drop = true;
          auto c = driver.exchange(command, now());
          require(c.sent && !c.feedback.fresh, "drop falsely refreshed sample");
          server.drop = false;
          server.malformed = true;
          auto d = driver.exchange(command, now());
          require(!d.feedback.fresh, "wrong length accepted");
          server.malformed = false;
          server.oversized = true;
          auto oversized = driver.exchange(command, now());
          require(!oversized.feedback.fresh, "oversized feedback accepted");
          server.oversized = false;
          auto e = driver.exchange(command, now());
          require(e.feedback.fresh, "feedback did not recover");
          server.foreign = true;
          auto invalid_peer = driver.exchange(command, now());
          require(!invalid_peer.feedback.fresh, "wrong source port accepted");
          server.foreign = false;
          if (version == 2) {
            server.motor_error = true;
            server.error_bits = 0x40;
            require(!driver.exchange(command, now()).feedback.motor_fault,
                    "vendor-unclassified status falsely reported as fault");
            for (unsigned known : {0x01u, 0x02u, 0x04u, 0x20u}) {
              server.error_bits = known | 0x40;
              require(driver.exchange(command, now()).feedback.motor_fault,
                      "known motor error with extra status bits ignored");
            }
            server.error_bits = 1;
            server.motor_error = false;
            for (auto *flag :
                 {&server.motor_error, &server.hot, &server.disconnected}) {
              *flag = true;
              auto device_fault = driver.exchange(command, now());
              require(device_fault.feedback.fresh &&
                          device_fault.feedback.motor_fault,
                      "V2 device fault ignored");
              *flag = false;
            }
            auto recovered = driver.exchange(command, now());
            require(!recovered.feedback.motor_fault,
                    "V2 device fault did not clear in feedback");
          }
          std::cout << "V" << version << ", joints=" << joints
                    << ", combined handshake=" << combined
                    << ": real UDP handshake, decode, stationary freshness, drop "
                       "and length errors passed\n";
        }
    {
      Firmware server(2, 6);
      auto slow = cfg;
      slow.period = 0.03;
      slow.receive_budget_us = 20000;
      McuTransport driver(slow);
      require(driver.exchange(Output{}, now()).feedback.fresh,
              "receive budget below its configured period was rejected");
    }
    cfg.local_port = 19081;
    {
      Firmware server(1, 6);
      McuTransport first(cfg);
      bool threw = false;
      try {
        McuTransport second(cfg);
      } catch (const std::exception &) {
        threw = true;
      }
      require(threw, "duplicate local port accepted");
    }
    {
      Firmware server(2, 6);
      McuTransport driver(cfg);
      require(driver.calibrate(200000).code == CalibrationCode::NotAttempted,
              "calibration without passive feedback was allowed");
      Output enabled;
      enabled.enabled = true;
      driver.exchange(enabled, now());
      require(driver.calibrate(200000).code == CalibrationCode::NotAttempted,
              "calibration after an enabled command was allowed");
      driver.exchange(Output{}, now());
      require(driver.calibrate(0).code == CalibrationCode::NotAttempted,
              "invalid calibration timeout was allowed");
      require(server.calibrations == 0, "preflight failure sent calibration");
      const double started = now();
      require(driver.calibrate(1000001).code == CalibrationCode::Succeeded,
              "official calibration reply was not accepted");
      require(now() - started >= 1.9, "official calibration wait was bypassed");
      driver.exchange(Output{}, now());
      require(driver.calibrate(200000).code == CalibrationCode::NotAttempted,
              "calibration repeated on the same connection");
      require(server.calibrations == 1,
              "more than one calibration request sent");
      server.drop = true;
      const double waiting = now();
      require(!driver.exchange(Output{}, now()).feedback.fresh,
              "missing feedback accepted after calibration");
      require(now() - waiting < 0.1,
              "calibration did not restore receive budget");
    }
    std::cout << "PASS: observed official MCU adapter over loopback\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
