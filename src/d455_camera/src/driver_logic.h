#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>

namespace d455_camera {
namespace detail {

using Clock = std::chrono::steady_clock;

// 即使在采集线程启动前，该信号也能安全生效。
/**
 * @brief 可取消等待与安全停机同步信号类
 * 
 * 结合 std::atomic、std::mutex 与 std::condition_variable，
 * 既支持低开销无锁查询停机状态，又支持可被停机请求随时打断的高精度定时等待，
 * 避免无脑 sleep 导致析构卡顿与线程假死。
 */
class StopSignal {
 public:
  /**
   * @brief 查询当前是否已收到停机请求
   * @return true 已请求停机；false 正常运行中
   */
  bool Requested() const { return requested_.load(); }

  /**
   * @brief 发出停机请求并广播唤醒所有等待中的线程
   * 
   * 在互斥锁保护下将 requested_ 置为 true（防止丢失唤醒），
   * 并在锁外调用 notify_all 唤醒所有挂起的等待线程以减少锁竞争。
   */
  void Request() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      requested_.store(true);
    }
    condition_.notify_all();
  }

  /**
   * @brief 在指定截止时间点前挂起等待，可被停机信号随时打断
   * 
   * @param deadline 单调时钟截止时间点
   * @return true 超时唤醒且未收到停机请求（可继续运行）；
   *         false 收到停机请求提前中断（应退出工作循环）
   */
  bool WaitUntil(Clock::time_point deadline) {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait_until(lock, deadline, [this] { return Requested(); });
    return !Requested();
  }

 private:
  std::atomic<bool> requested_{false};  ///< 是否已发出停机请求的原子标志位
  std::mutex mutex_;                    ///< 保护条件变量与状态变更的互斥锁
  std::condition_variable condition_;   ///< 用于可中断休眠与唤醒的条件变量
};

struct Timestamp {
  uint32_t sec;
  uint32_t nsec;
};

inline bool SplitTimestamp(double milliseconds, Timestamp *result) {
  if (!std::isfinite(milliseconds) || milliseconds < 0.0) {
    return false;
  }
  const double seconds = milliseconds / 1000.0;
  double whole = std::floor(seconds);
  double nanoseconds = std::round((seconds - whole) * 1e9);
  if (nanoseconds >= 1e9) {
    ++whole;
    nanoseconds = 0.0;
  }
  if (whole > std::numeric_limits<uint32_t>::max()) {
    return false;
  }
  *result = {static_cast<uint32_t>(whole), static_cast<uint32_t>(nanoseconds)};
  return true;
}

struct StreamProgress {
  uint64_t frame_number = 0;
  double timestamp_ms = 0.0;
  bool has_frame = false;
  Clock::time_point last_valid;

  void Reset(Clock::time_point now) {
    frame_number = 0;
    timestamp_ms = 0.0;
    has_frame = false;
    last_valid = now;
  }

  bool IsNew(uint64_t number) const { return !has_frame || number != frame_number; }

  bool Regressed(uint64_t number, double stamp_ms) const {
    return has_frame && (number < frame_number || stamp_ms < timestamp_ms - 1000.0);
  }

  void Accept(uint64_t number, double stamp_ms, Clock::time_point now) {
    frame_number = number;
    timestamp_ms = stamp_ms;
    has_frame = true;
    last_valid = now;
  }

  bool Stalled(Clock::time_point now, double timeout_sec) const {
    return std::chrono::duration<double>(now - last_valid).count() >= timeout_sec;
  }
};

inline uint16_t DepthMillimeters(uint16_t raw, double millimeters_per_unit) {
  if (raw == 0 || !std::isfinite(millimeters_per_unit) || millimeters_per_unit <= 0.0) {
    return 0;
  }
  const double millimeters = raw * millimeters_per_unit;
  const double rounded = std::round(millimeters);
  if (!std::isfinite(rounded) || rounded > 65535.0) {
    return 0;
  }
  return static_cast<uint16_t>(std::max(1.0, rounded));
}

inline bool ValidDuration(double seconds) {
  // Leave room for adding a steady-clock epoch without overflowing its representation.
  return std::isfinite(seconds) &&
         seconds >= std::chrono::duration<double>(Clock::duration(1)).count() &&
         seconds < std::chrono::duration<double>(Clock::duration::max()).count() / 2.0;
}

inline void CopyDepthRow(const uint8_t *source, uint8_t *destination, size_t width,
                         double millimeters_per_unit) {
  // The fast path must preserve rounding for every possible uint16_t value.
  if (std::abs(millimeters_per_unit - 1.0) * 65535.0 < 0.5) {
    std::memcpy(destination, source, width * sizeof(uint16_t));
    return;
  }
  for (size_t x = 0; x < width; ++x) {
    uint16_t raw;
    std::memcpy(&raw, source + x * sizeof(raw), sizeof(raw));
    const uint16_t converted = DepthMillimeters(raw, millimeters_per_unit);
    std::memcpy(destination + x * sizeof(converted), &converted, sizeof(converted));
  }
}

inline bool IsBigEndian() {
  const uint16_t value = 0x0102;
  return *reinterpret_cast<const uint8_t *>(&value) == 0x01;
}

}  // namespace detail
}  // namespace d455_camera
