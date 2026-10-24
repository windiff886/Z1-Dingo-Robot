#pragma once
#include "controller.hpp"
#include "transport.hpp"
#include <atomic>
#include <functional>
#include <thread>
namespace z1 {
double monotonicTime();
struct RuntimeOptions {
  // Both settings apply only to the control thread. Zero / -1 leave the
  // operating system's defaults unchanged.
  int realtime_priority{0};
  int cpu_affinity{-1};
};
struct RuntimeTiming {
  // Reset on start; cycles include controlled stopping, excluding the final
  // passive exchange. missed_slots counts expired absolute-grid deadlines.
  uint64_t cycles{}, missed_slots{};
  double exchange_us{}, max_exchange_us{}, wakeup_lateness_us{},
      max_wakeup_lateness_us{};
  bool realtime_requested{}, realtime_applied{}, affinity_requested{},
      affinity_applied{};
  // pthread error numbers; a failed optional setting does not stop the runtime.
  int realtime_error{}, affinity_error{};
};
class Runtime {
public:
  Runtime(ModelConfig, Config, std::unique_ptr<Transport>, RuntimeOptions = {});
  Runtime(ModelConfig, Config, std::function<std::unique_ptr<Transport>()>,
          RuntimeOptions = {});
  ~Runtime();
  Controller &controller() { return controller_; }
  // Lock-free observational snapshot; fields may span adjacent cycles.
  RuntimeTiming timing() const;
  void start();
  void stop();

private:
  Controller controller_;
  Config config_;
  std::unique_ptr<Transport> transport_;
  RuntimeOptions options_;
  struct TimingCounters {
    std::atomic<uint64_t> cycles{0}, missed_slots{0}, exchange_ns{0},
        max_exchange_ns{0}, wakeup_lateness_ns{0}, max_wakeup_lateness_ns{0};
    std::atomic<bool> realtime_applied{false}, affinity_applied{false};
    std::atomic<int> realtime_error{0}, affinity_error{0};
  } timing_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};
} // namespace z1
