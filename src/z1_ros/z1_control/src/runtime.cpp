#include "z1_control/runtime.hpp"
#include <cerrno>
#include <chrono>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
namespace z1 {
double monotonicTime() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
Runtime::Runtime(ModelConfig m, Config c, std::unique_ptr<Transport> t,
                 RuntimeOptions options)
    : controller_(m, c), config_(c), transport_(std::move(t)),
      options_(options) {
  if (!transport_)
    throw std::invalid_argument("missing transport");
}
Runtime::~Runtime() { stop(); }
Runtime::Runtime(ModelConfig m, Config c,
                 std::function<std::unique_ptr<Transport>()> factory,
                 RuntimeOptions options)
    : controller_(m, c), config_(c), transport_(factory()), options_(options) {
  if (!transport_)
    throw std::invalid_argument("missing transport");
}
RuntimeTiming Runtime::timing() const {
  RuntimeTiming result;
  result.cycles = timing_.cycles.load(std::memory_order_relaxed);
  result.missed_slots = timing_.missed_slots.load(std::memory_order_relaxed);
  result.exchange_us =
      timing_.exchange_ns.load(std::memory_order_relaxed) / 1e3;
  result.max_exchange_us =
      timing_.max_exchange_ns.load(std::memory_order_relaxed) / 1e3;
  result.wakeup_lateness_us =
      timing_.wakeup_lateness_ns.load(std::memory_order_relaxed) / 1e3;
  result.max_wakeup_lateness_us =
      timing_.max_wakeup_lateness_ns.load(std::memory_order_relaxed) / 1e3;
  result.realtime_requested = options_.realtime_priority != 0;
  result.affinity_requested = options_.cpu_affinity != -1;
  result.realtime_applied =
      timing_.realtime_applied.load(std::memory_order_relaxed);
  result.affinity_applied =
      timing_.affinity_applied.load(std::memory_order_relaxed);
  result.realtime_error =
      timing_.realtime_error.load(std::memory_order_relaxed);
  result.affinity_error =
      timing_.affinity_error.load(std::memory_order_relaxed);
  return result;
}
void Runtime::start() {
  if (running_.exchange(true))
    return;
  timing_.cycles = 0;
  timing_.missed_slots = 0;
  timing_.exchange_ns = 0;
  timing_.max_exchange_ns = 0;
  timing_.wakeup_lateness_ns = 0;
  timing_.max_wakeup_lateness_ns = 0;
  timing_.realtime_applied = false;
  timing_.affinity_applied = false;
  timing_.realtime_error = 0;
  timing_.affinity_error = 0;
  thread_ = std::thread([this] {
    using Clock = std::chrono::steady_clock;
    if (options_.cpu_affinity != -1) {
      int error = EINVAL;
      if (options_.cpu_affinity >= 0 && options_.cpu_affinity < CPU_SETSIZE) {
        cpu_set_t cpus;
        CPU_ZERO(&cpus);
        CPU_SET(options_.cpu_affinity, &cpus);
        error = pthread_setaffinity_np(pthread_self(), sizeof(cpus), &cpus);
      }
      timing_.affinity_error.store(error, std::memory_order_relaxed);
      timing_.affinity_applied.store(error == 0, std::memory_order_relaxed);
    }
    if (options_.realtime_priority != 0) {
      sched_param parameters{};
      parameters.sched_priority = options_.realtime_priority;
      int error =
          pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameters);
      timing_.realtime_error.store(error, std::memory_order_relaxed);
      timing_.realtime_applied.store(error == 0, std::memory_order_relaxed);
    }
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(config_.period));
    auto next = Clock::now();
    Output out;
    auto record = [](std::atomic<uint64_t> &last,
                     std::atomic<uint64_t> &maximum, uint64_t value) {
      last.store(value, std::memory_order_relaxed);
      if (value > maximum.load(std::memory_order_relaxed))
        maximum.store(value, std::memory_order_relaxed);
    };
    auto cycle = [&] {
      const auto began = Clock::now();
      record(timing_.wakeup_lateness_ns, timing_.max_wakeup_lateness_ns,
             began > next
                 ? std::chrono::duration_cast<std::chrono::nanoseconds>(began -
                                                                        next)
                       .count()
                 : 0);
      Exchange e;
      bool exchanged = true;
      const auto exchange_began = Clock::now();
      try {
        e = transport_->exchange(out, std::chrono::duration<double>(
                                          exchange_began.time_since_epoch())
                                          .count());
      } catch (const std::exception &error) {
        exchanged = false;
        e.feedback.fresh = false;
        e.feedback.transport_fault = true;
        e.error = std::string("transport exception: ") + error.what();
      }
      record(timing_.exchange_ns, timing_.max_exchange_ns,
             std::chrono::duration_cast<std::chrono::nanoseconds>(
                 Clock::now() - exchange_began)
                 .count());
      // Receive time belongs to the transport; evaluate validity after the
      // bounded exchange.
      if (!e.sent) {
        e.feedback.transport_fault = true;
        if (e.error.empty())
          e.error = "MCU packet send failed";
      }
      out = controller_.step(e.feedback, monotonicTime(), e.error);
      if (!e.sent)
        out = Output{};
      timing_.cycles.fetch_add(1, std::memory_order_relaxed);
      return exchanged;
    };
    auto waitNext = [&] {
      next += period;
      const auto finished = Clock::now();
      if (next <= finished) {
        // Preserve the absolute grid while skipping expired slots. Never run
        // old cycles back-to-back to make up a delayed exchange or wakeup.
        const auto missed = (finished - next) / period + 1;
        next += period * missed;
        timing_.missed_slots.fetch_add(missed, std::memory_order_relaxed);
      }
      std::this_thread::sleep_until(next);
    };
    while (running_) {
      cycle();
      waitNext();
    }
    // Normal process shutdown attempts a bounded controlled stop while feedback
    // is valid.
    auto mode = controller_.snapshot().mode;
    if (mode != Mode::Disabled && mode != Mode::Fault &&
        mode != Mode::AwaitResume) {
      controller_.operate(Operation::Stop);
      auto deadline = Clock::now() + std::chrono::seconds(2);
      while (Clock::now() < deadline) {
        if (!cycle()) {
          out = Output{};
          break;
        }
        auto stopped = controller_.snapshot().mode;
        if (stopped == Mode::AwaitResume || stopped == Mode::Disabled ||
            stopped == Mode::Fault)
          break;
        waitNext();
      }
    }
    // Attempt a final passive packet; this is not a firmware stop
    // acknowledgment.
    try {
      transport_->exchange(Output{}, monotonicTime());
    } catch (const std::exception &) {
    }
  });
}
void Runtime::stop() {
  running_ = false;
  if (thread_.joinable())
    thread_.join();
}
} // namespace z1
