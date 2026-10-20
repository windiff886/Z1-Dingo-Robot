#include "z1_control/transport.hpp"
#include <arpa/inet.h>
#include <array>
#include <atomic>
#include <cstring>
#include <iostream>
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
  std::atomic<bool> foreign{false}, motor_error{false}, hot{false},
      disconnected{false};
  int protocol, joints;
  std::atomic<unsigned> commands{0};
  Firmware(int v, int n) : protocol(v), joints(n) {
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
      if (n == 1) {
        if (stage % 2 == 0) {
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
        size = joints * (protocol == 1 ? 16 : 22);
        for (int j = 0; j < joints; ++j) {
          float pos = static_cast<float>(0.125 + j * 0.01);
          memcpy(reply.data() + j * (protocol == 1 ? 16 : 22) + 12, &pos, 4);
          if (protocol == 2) {
            reply[j * 22 + 16] = 25;
            reply[j * 22 + 19] = 25;
            if (j == 0) {
              if (motor_error)
                reply[j * 22 + 17] = 1;
              if (hot)
                reply[j * 22 + 16] = 85;
              if (disconnected)
                reply[j * 22 + 18] = 64;
            }
          }
        }
        if (malformed)
          size -= 1;
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
    cfg.hardware_validated = true;
    cfg.receive_budget_us = 5000;
    for (int version : {1, 2})
      for (int joints : {6, 7}) {
        Firmware server(version, joints);
        McuTransport driver(cfg);
        Output command;
        auto a = driver.exchange(command, now());
        require(a.sent && a.feedback.fresh, "fresh feedback missing");
        require(std::abs(a.feedback.q[0] - 0.125) < 1e-6,
                "vendor packet decoded incorrectly");
        require(a.feedback.has_gripper == (joints == 7),
                "gripper detection failed");
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
        auto e = driver.exchange(command, now());
        require(e.feedback.fresh, "feedback did not recover");
        server.foreign = true;
        auto invalid_peer = driver.exchange(command, now());
        require(!invalid_peer.feedback.fresh, "wrong source port accepted");
        server.foreign = false;
        if (version == 2) {
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
                  << ": real UDP handshake, decode, stationary freshness, drop "
                     "and length errors passed\n";
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
    cfg.hardware_validated = false;
    bool gated = false;
    try {
      McuTransport driver(cfg);
    } catch (const std::exception &) {
      gated = true;
    }
    require(gated, "uncommissioned hardware allowed");
    std::cout << "PASS: observed official MCU adapter over loopback\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
