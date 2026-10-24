// Wall-clock scheduling probe. Simulation only: never creates an MCU backend.
#include "z1_control/runtime.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <sys/utsname.h>
#include <thread>
#include <vector>
using namespace z1;
namespace {
struct MeasuredSimulation final : Transport {
  Simulation simulation;
  std::atomic<bool> record{false};
  std::vector<double> times;
  MeasuredSimulation(ModelConfig model, double dt, double duration)
      : simulation(model, dt) {
    times.reserve(static_cast<size_t>((duration + 1) / dt));
  }
  Exchange exchange(const Output &out, double now) override {
    if (record.load(std::memory_order_relaxed))
      times.push_back(monotonicTime());
    return simulation.exchange(out, now);
  }
};
double quantile(std::vector<double> values, double q) {
  std::sort(values.begin(), values.end());
  return values[std::min(values.size() - 1,
                         static_cast<size_t>(q * values.size()))];
}
} // namespace
int main(int argc, char **argv) {
  try {
    if (argc < 2 || argc > 5) {
      std::cerr << "Usage: runtime_rate_probe MODEL [seconds_per_rate=4] "
                   "[cpu=-1] [fifo_priority=0]\nSimulation only.\n";
      return 2;
    }
    const double duration = argc > 2 ? std::stod(argv[2]) : 4;
    if (!std::isfinite(duration) || duration < 1 || duration > 120)
      throw std::invalid_argument("duration must be 1..120 seconds per rate");
    RuntimeOptions options;
    if (argc > 3)
      options.cpu_affinity = std::stoi(argv[3]);
    if (argc > 4)
      options.realtime_priority = std::stoi(argv[4]);
    utsname system{};
    if (uname(&system) == 0)
      std::cout << "host_kernel=" << system.release
                << ",architecture=" << system.machine << '\n';
    std::ifstream cpuinfo("/proc/cpuinfo");
    for (std::string line; std::getline(cpuinfo, line);)
      if (line.find("model name") == 0) {
        std::cout << "host_cpu=" << line.substr(line.find(':') + 1) << '\n';
        break;
      }
    std::cout << "backend=simulation,seconds_per_rate=" << duration
              << ",requested_cpu=" << options.cpu_affinity
              << ",requested_fifo_priority=" << options.realtime_priority
              << '\n';
    bool failed = false;
    for (double rate : {250., 500., 1000.}) {
      Config config;
      config.period = 1 / rate;
      const auto model = defaultModelConfig(argv[1]);
      auto backend =
          std::make_unique<MeasuredSimulation>(model, config.period, duration);
      auto *measured = backend.get();
      Runtime runtime(model, config, std::move(backend), options);
      runtime.start();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      runtime.controller().operate(Operation::Enable);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      runtime.controller().operate(Operation::JointStream);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      auto state = runtime.controller().snapshot();
      const auto session = state.session;
      const Vec initial = state.feedback.q;
      const auto start = std::chrono::steady_clock::now();
      const double began = monotonicTime();
      const auto before = runtime.timing();
      measured->record = true;
      unsigned accepted = 0, rejected = 0;
      const unsigned updates = static_cast<unsigned>(duration * 50);
      for (unsigned i = 0; i < updates; ++i) {
        const double now = monotonicTime(), t = now + 0.04 - began;
        constexpr double omega = 3.141592653589793;
        JointReference point;
        point.time = now + 0.04;
        point.position = initial;
        point.position[0] += 0.01 * std::sin(omega * t);
        point.velocity[0] = 0.01 * omega * std::cos(omega * t);
        point.acceleration[0] = -0.01 * omega * omega * std::sin(omega * t);
        point.has_velocity = point.has_acceleration = true;
        if (runtime.controller().stream(session, i + 1, point, now))
          ++accepted;
        else
          ++rejected;
        std::this_thread::sleep_until(start +
                                      std::chrono::milliseconds(20 * (i + 1)));
      }
      measured->record = false;
      state = runtime.controller().snapshot();
      const auto after = runtime.timing();
      runtime.stop();
      if (measured->times.size() < 2)
        throw std::runtime_error("too few control cycles recorded");
      std::vector<double> periods, errors;
      unsigned late = 0;
      for (size_t i = 1; i < measured->times.size(); ++i) {
        const double dt = measured->times[i] - measured->times[i - 1];
        periods.push_back(dt * 1e6);
        errors.push_back(std::abs(dt - config.period) * 1e6);
        late += dt > 1.5 * config.period;
      }
      const double actual = (measured->times.size() - 1) /
                            (measured->times.back() - measured->times.front());
      std::cout << std::fixed << std::setprecision(3) << "target_hz=" << rate
                << ",actual_hz=" << actual
                << ",period_p50_us=" << quantile(periods, 0.5)
                << ",period_p99_us=" << quantile(periods, 0.99)
                << ",abs_jitter_p99_us=" << quantile(errors, 0.99)
                << ",max_period_us="
                << *std::max_element(periods.begin(), periods.end())
                << ",cycles_over_1.5T=" << late
                << ",missed_slots=" << after.missed_slots - before.missed_slots
                << ",max_exchange_us_since_start=" << after.max_exchange_us
                << ",realtime_applied=" << after.realtime_applied
                << ",realtime_error=" << after.realtime_error
                << ",affinity_applied=" << after.affinity_applied
                << ",affinity_error=" << after.affinity_error
                << ",accepted=" << accepted << ",rejected=" << rejected
                << ",mode=" << static_cast<int>(state.mode)
                << ",reason=" << state.reason << '\n';
      failed |= state.mode != Mode::JointStream || rejected != 0;
    }
    return failed ? 1 : 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
