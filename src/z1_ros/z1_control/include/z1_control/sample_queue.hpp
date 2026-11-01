#pragma once
#include "runtime.hpp"
#include <array>
#include <atomic>
namespace z1 {
// One control-thread producer, one ROS publisher consumer. Never overwrite data.
template <size_t Capacity = 2048> class SampleQueue {
public:
  bool push(const RuntimeSample &sample) {
    const auto write = write_.load(std::memory_order_relaxed);
    const auto next = (write + 1) % Capacity;
    if (next == read_.load(std::memory_order_acquire)) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    samples_[write] = sample;
    write_.store(next, std::memory_order_release);
    return true;
  }
  bool pop(RuntimeSample &sample) {
    const auto read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire))
      return false;
    sample = samples_[read];
    read_.store((read + 1) % Capacity, std::memory_order_release);
    return true;
  }
  uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }
private:
  static_assert(Capacity > 1, "sample queue needs at least two slots");
  std::array<RuntimeSample, Capacity> samples_;
  std::atomic<size_t> write_{0}, read_{0};
  std::atomic<uint64_t> dropped_{0};
};
} // namespace z1
