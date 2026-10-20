#pragma once
#include <array>
#include <chrono>
#include <cstdint>
#include <vector>
namespace z1::udp {
struct Observer {
  std::vector<int> descriptors;
  unsigned budget_us{1500};
  bool sent{true}, fresh{};
  size_t last_length{};
  unsigned motor_count{}, protocol{};
  std::chrono::steady_clock::time_point received;
  std::array<uint8_t, 154> state_bytes{};
};
extern thread_local Observer *active;
struct Scope {
  Observer *old;
  explicit Scope(Observer &o) : old(active) { active = &o; }
  ~Scope() { active = old; }
};
} // namespace z1::udp
