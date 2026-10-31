#include "z1_control/runtime.hpp"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
#include <vector>

using namespace z1;
using Clock = std::chrono::steady_clock;
namespace {
void require(bool value, const char *message) {
  if (!value)
    throw std::runtime_error(message);
}
template <typename Predicate>
void waitFor(Predicate ready, const char *message) {
  const auto deadline = Clock::now() + std::chrono::seconds(3);
  while (!ready()) {
    require(Clock::now() < deadline, message);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
struct RecordedTransport final : Transport {
  struct Sample {
    double time;
    bool enabled;
    int cpu;
  };
  std::vector<Sample> samples;
  unsigned delay_cycle{};
  std::chrono::milliseconds delay{};
  Vec q{(Vec() << 0, 0.8, -1.1, 0.3, 0, 0).finished()};
  uint64_t sequence{};
  RecordedTransport() { samples.reserve(1024); }
  Exchange exchange(const Output &out, double now) override {
    samples.push_back({now, out.enabled, sched_getcpu()});
    if (++sequence == delay_cycle)
      std::this_thread::sleep_for(delay);
    if (out.enabled)
      q = out.q;
    Feedback feedback;
    feedback.q = q;
    feedback.dq = out.enabled ? out.dq : Vec::Zero();
    feedback.tau = out.enabled ? out.tau : Vec::Zero();
    feedback.received_at = monotonicTime();
    feedback.sequence = sequence;
    feedback.fresh = true;
    return {feedback, true, {}};
  }
};
void overrunSkipsSlots(const ModelConfig &model) {
  Config config;
  config.period = 0.01;
  auto transport = std::make_unique<RecordedTransport>();
  auto *recorded = transport.get();
  recorded->delay_cycle = 4;
  recorded->delay = std::chrono::milliseconds(85);
  Runtime runtime(model, config, std::move(transport));
  runtime.start();
  runtime.start(); // Starting an active runtime must not create another loop.
  waitFor([&] { return runtime.timing().cycles >= 15; },
          "runtime did not advance");
  runtime.stop();
  const auto timing = runtime.timing();
  require(timing.missed_slots >= 8, "overdue slots were not counted");
  require(timing.max_exchange_us >= 80000, "exchange delay was not measured");
  require(timing.cycles + 1 == recorded->samples.size(),
          "shutdown did not send exactly one final passive packet");
  // A delayed exchange must return to a future slot. It must not issue a burst
  // of expired cycles immediately after the delayed one. Ignore the final
  // passive packet, which intentionally follows the shutdown decision.
  for (size_t i = 4; i + 1 < recorded->samples.size(); ++i)
    require(recorded->samples[i].time - recorded->samples[i - 1].time > 0.001,
            "runtime issued a burst of overdue cycles");
  require(!timing.realtime_requested && !timing.affinity_requested,
          "default runtime unexpectedly requested thread scheduling settings");
  require(!recorded->samples.back().enabled, "final packet was not passive");
  runtime.stop();
  std::cout << "overrun cycles=" << timing.cycles
            << " missed_slots=" << timing.missed_slots
            << " max_exchange_us=" << timing.max_exchange_us << '\n';
}
void schedulerSettings(const ModelConfig &model) {
  Config config;
  RuntimeOptions invalid;
  invalid.realtime_priority = -1;
  invalid.cpu_affinity = CPU_SETSIZE;
  Runtime rejected(model, config, std::make_unique<RecordedTransport>(),
                   invalid);
  rejected.start();
  waitFor([&] { return rejected.timing().cycles >= 3; },
          "rejected scheduler options stopped the runtime");
  rejected.stop();
  const auto status = rejected.timing();
  require(status.realtime_requested && !status.realtime_applied &&
              status.realtime_error == EINVAL,
          "real-time setting failure was not exposed");
  require(status.affinity_requested && !status.affinity_applied &&
              status.affinity_error == EINVAL,
          "affinity setting failure was not exposed");

  cpu_set_t permitted;
  CPU_ZERO(&permitted);
  require(pthread_getaffinity_np(pthread_self(), sizeof(permitted),
                                 &permitted) == 0,
          "could not inspect test thread affinity");
  int selected = 0;
  while (selected < CPU_SETSIZE && !CPU_ISSET(selected, &permitted))
    ++selected;
  require(selected < CPU_SETSIZE, "no permitted CPU");
  RuntimeOptions pinned;
  pinned.cpu_affinity = selected;
  auto transport = std::make_unique<RecordedTransport>();
  auto *recorded = transport.get();
  Runtime runtime(model, config, std::move(transport), pinned);
  runtime.start();
  waitFor([&] { return runtime.timing().cycles >= 3; },
          "pinned runtime did not advance");
  runtime.stop();
  require(runtime.timing().affinity_applied &&
              runtime.timing().affinity_error == 0,
          "permitted thread affinity was not applied");
  require(
      std::all_of(recorded->samples.begin(), recorded->samples.end(),
                  [&](const auto &sample) { return sample.cpu == selected; }),
      "control thread ran outside requested affinity");
}
void controlledShutdown(const ModelConfig &model) {
  Config config;
  auto transport = std::make_unique<RecordedTransport>();
  auto *recorded = transport.get();
  Runtime runtime(model, config, std::move(transport));
  runtime.start();
  waitFor([&] { return runtime.timing().cycles >= 3; }, "no initial feedback");
  runtime.controller().operate(Operation::Enable);
  waitFor([&] { return runtime.controller().snapshot().mode == Mode::Holding; },
          "runtime did not enable");
  auto q = runtime.controller().snapshot().feedback.q;
  q[0] += 0.08;
  require(runtime.controller().goal(1, q, 0.3), "runtime goal rejected");
  waitFor(
      [&] { return runtime.controller().snapshot().mode == Mode::JointGoal; },
      "runtime motion did not start");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const auto before = runtime.timing().cycles;
  runtime.stop();
  require(runtime.controller().snapshot().mode == Mode::AwaitResume,
          "runtime did not finish its controlled stop");
  require(runtime.timing().cycles > before + 2,
          "controlled stop did not execute across control slots");
  require(!recorded->samples.back().enabled, "shutdown omitted passive packet");
  // Every exchange during braking uses the same absolute period scheduler.
  // The exception is the intentional final passive packet.
  for (size_t i = before + 1; i + 1 < recorded->samples.size(); ++i)
    require(recorded->samples[i].time - recorded->samples[i - 1].time >
                config.period * 0.1,
            "controlled stop used a burst of unpaced updates");
}

void transportFailureDetails(const ModelConfig &model) {
  struct FailingTransport final : Transport {
    RecordedTransport delegate;
    std::atomic<bool> fail{false};
    int kind;
    explicit FailingTransport(int value) : kind(value) {}
    Exchange exchange(const Output &output, double now) override {
      if (fail.exchange(false)) {
        if (kind == 0)
          throw std::runtime_error("invalid gripper command: position=1.18e-05");
        Exchange error;
        error.feedback.transport_fault = true;
        error.error = kind == 1 ? "receive backlog limit exceeded"
                               : "MCU packet send failed";
        return error;
      }
      return delegate.exchange(output, now);
    }
  };
  for (int kind = 0; kind < 3; ++kind) {
    auto transport = std::make_unique<FailingTransport>(kind);
    auto *injected = transport.get();
    Runtime runtime(model, Config{}, std::move(transport));
    runtime.start();
    waitFor([&] { return runtime.controller().snapshot().feedback_valid; },
            "missing feedback before transport fault test");
    auto target = runtime.controller().snapshot().feedback.q;
    target[0] += .04;
    require(runtime.controller().goal(100, target, .05),
            "goal rejected before injected transport fault");
    injected->fail = true;
    waitFor([&] { return runtime.controller().snapshot().mode == Mode::Fault; },
            "transport failure did not fault");
    runtime.stop();
    const std::string expected = kind == 0 ? "invalid gripper command"
                                : kind == 1 ? "receive backlog limit exceeded"
                                            : "MCU packet send failed";
    require(runtime.controller().snapshot().reason.find(expected) !=
                std::string::npos,
            "transport fault detail was discarded from status");
    bool failed = false;
    for (const auto &event : runtime.controller().events())
      failed |= event.id == 100 && event.outcome == Outcome::Failed &&
                event.reason.find(expected) != std::string::npos;
    require(failed, "transport fault detail missing from motion result");
    require(!injected->delegate.samples.back().enabled,
            "transport failure did not restore passive output");
  }
}

void shutdownTransportFailure(const ModelConfig &model) {
  struct StopFailure final : Transport {
    RecordedTransport delegate;
    Controller *controller{};
    bool failed{};
    Exchange exchange(const Output &out, double now) override {
      if (!failed && controller->snapshot().mode == Mode::Stopping) {
        failed = true;
        throw std::runtime_error("transport failed during shutdown");
      }
      return delegate.exchange(out, now);
    }
  };
  auto transport = std::make_unique<StopFailure>();
  auto *recorded = transport.get();
  Runtime runtime(model, Config{}, std::move(transport));
  recorded->controller = &runtime.controller();
  runtime.start();
  waitFor([&] { return runtime.controller().snapshot().feedback_valid; },
          "missing feedback before shutdown fault test");
  auto target = runtime.controller().snapshot().feedback.q;
  target[0] += .08;
  require(runtime.controller().goal(1, target, .3),
          "goal rejected before shutdown fault test");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  runtime.stop();
  require(recorded->failed && runtime.controller().snapshot().mode == Mode::Fault,
          "shutdown transport exception did not end in Fault");
  require(!recorded->delegate.samples.back().enabled,
          "shutdown failure omitted the final passive exchange");
}
} // namespace
int main(int argc, char **argv) {
  try {
    require(argc == 2, "model argument missing");
    const auto model = defaultModelConfig(argv[1]);
    overrunSkipsSlots(model);
    schedulerSettings(model);
    controlledShutdown(model);
    transportFailureDetails(model);
    shutdownTransportFailure(model);
    std::cout << "runtime timing and shutdown tests passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
