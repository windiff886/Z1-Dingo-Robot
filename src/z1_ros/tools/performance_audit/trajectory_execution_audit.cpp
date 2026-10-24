// Offline comparison of complete timed plans and online target chasing.
// Simulation advances at fixed dt; this is not a wall-clock or MCU benchmark.
#include "z1_control/controller.hpp"
#include "z1_control/transport.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <new>
#include <stdexcept>
#include <vector>

namespace {
thread_local bool count_new = false;
thread_local uint64_t new_calls = 0;
volatile double sample_sink = 0;
using Clock = std::chrono::steady_clock;
using namespace z1;
constexpr double amplitude = 0.02, duration = 4.0, producer_period = 0.02,
                 future = 0.04;
const double omega = std::acos(-1.0) / 2;

double micros(Clock::time_point start) {
  return std::chrono::duration<double, std::micro>(Clock::now() - start)
      .count();
}

TrajectoryPoint point(const Vec &base, double time) {
  const double s = std::sin(omega * time), c = std::cos(omega * time);
  TrajectoryPoint p;
  p.time_from_start = time;
  p.position = base;
  p.position[0] += amplitude * s * s * s;
  p.velocity[0] = 3 * amplitude * omega * s * s * c;
  p.acceleration[0] =
      3 * amplitude * omega * omega * (2 * s * c * c - s * s * s);
  p.has_velocity = p.has_acceleration = true;
  return p;
}

std::vector<TrajectoryPoint> points(const Vec &base) {
  std::vector<TrajectoryPoint> result;
  const int count = std::lround(duration / producer_period);
  result.reserve(count + 1);
  for (int i = 0; i <= count; ++i)
    result.push_back(point(base, i * producer_period));
  return result;
}

void require(bool condition, const std::string &reason) {
  if (!condition)
    throw std::runtime_error(reason);
}

struct Metrics {
  double reference_squared{}, reference_max{}, actual_squared{},
      actual_all_squared{}, planning_us{};
  std::vector<double> steps;
  uint64_t sent{};
};

void run(const ModelConfig &model, const Config &config, bool timed) {
  Controller controller(model, config);
  Simulation simulation(model, config.period);
  Output output;
  Feedback feedback;
  double now = 1;
  auto tick = [&]() {
    now += config.period;
    feedback = simulation.exchange(output, now).feedback;
    output = controller.step(feedback, now);
  };
  tick();
  controller.operate(Operation::Enable);
  for (int i = 0; i < std::lround(2 / config.period); ++i)
    tick();
  require(controller.snapshot().mode == Mode::Holding, "warmup failed");
  // Match the additional cycle needed to enter JointStream in both runs.
  if (!timed)
    controller.operate(Operation::JointStream);
  tick();
  const Vec base = output.q;
  const double began = now;
  const auto session = controller.snapshot().session;
  uint64_t sequence = 0;
  Metrics metrics;
  metrics.steps.reserve(std::lround(duration / config.period));
  auto send = [&](double phase) {
    const auto sample = point(base, phase + future);
    JointReference reference;
    reference.time = now + future;
    reference.position = sample.position;
    reference.velocity = sample.velocity;
    reference.acceleration = sample.acceleration;
    reference.has_velocity = reference.has_acceleration = true;
    require(controller.stream(session, ++sequence, reference, now),
            "scheduled stream target rejected");
    ++metrics.sent;
  };
  if (timed) {
    const auto plan = points(base);
    const auto start = Clock::now();
    const bool accepted = controller.trajectory(1, plan, false);
    metrics.planning_us = micros(start);
    require(accepted, "timed plan rejected");
    metrics.sent = plan.size();
  } else {
    require(controller.snapshot().mode == Mode::JointStream,
            "stream mode not entered");
    send(0);
  }
  const auto accepted = controller.snapshot();
  const int cycles = std::lround(duration / config.period);
  const int producer_cycles = std::lround(producer_period / config.period);
  for (int i = 1; i <= cycles; ++i) {
    now += config.period;
    const double elapsed = now - began;
    feedback = simulation.exchange(output, now).feedback;
    if (!timed && i % producer_cycles == 0)
      send(elapsed);
    const auto start = Clock::now();
    output = controller.step(feedback, now);
    metrics.steps.push_back(micros(start));
    const auto state = controller.snapshot();
    require(state.mode != Mode::Fault && state.mode != Mode::AwaitResume &&
                state.mode != Mode::Stopping,
            "unexpected stop: " + state.reason);
    require(state.mode == (timed ? Mode::JointTrajectory : Mode::JointStream) ||
                (timed && state.mode == Mode::Holding &&
                 elapsed >= duration - config.period),
            "motion ended before the input period");
    const auto ideal = point(base, elapsed);
    const double reference_error = output.q[0] - ideal.position[0];
    const double actual_error = feedback.q[0] - ideal.position[0];
    metrics.reference_squared += reference_error * reference_error;
    metrics.reference_max =
        std::max(metrics.reference_max, std::abs(reference_error));
    metrics.actual_squared += actual_error * actual_error;
    metrics.actual_all_squared += (feedback.q - ideal.position).squaredNorm();
  }
  const auto measured = controller.snapshot();
  if (!timed) {
    // End the stream explicitly after measurement. Hold acknowledges a fully
    // stopped AwaitResume state; it is not an interrupt for an active stream.
    controller.operate(Operation::Stop);
    for (int i = 0; i < std::lround(3 / config.period) &&
                    controller.snapshot().mode != Mode::AwaitResume;
         ++i) {
      tick();
      const auto state = controller.snapshot();
      require(state.mode != Mode::Fault,
              "stream cleanup stop failed: " + state.reason);
    }
    require(controller.snapshot().mode == Mode::AwaitResume,
            "stream cleanup did not finish stopping");
    controller.operate(Operation::Hold);
  }
  for (int i = 0; i < std::lround(3 / config.period) &&
                  controller.snapshot().mode != Mode::Holding;
       ++i) {
    tick();
    const auto state = controller.snapshot();
    require(state.mode != Mode::Fault &&
                (!timed || state.mode != Mode::AwaitResume),
            "settling failed: " + state.reason);
  }
  const auto final = controller.snapshot();
  require(final.mode == Mode::Holding, "run did not finish in Holding");
  bool succeeded = false;
  for (const auto &event : controller.events())
    if (event.id == 1 && event.outcome == Outcome::Succeeded)
      succeeded = true;
  require(!timed || succeeded, "timed plan did not report success");
  std::sort(metrics.steps.begin(), metrics.steps.end());
  std::cout << "execution,kind=" << (timed ? "timed_plan" : "scheduled_stream")
            << ",samples=" << cycles << ",input_points=" << metrics.sent
            << ",q1_reference_to_ideal_rms_rad="
            << std::sqrt(metrics.reference_squared / cycles)
            << ",q1_reference_to_ideal_max_rad=" << metrics.reference_max
            << ",q1_actual_to_ideal_rms_rad="
            << std::sqrt(metrics.actual_squared / cycles)
            << ",all_joint_actual_to_ideal_l2_rms_rad="
            << std::sqrt(metrics.actual_all_squared / cycles)
            << ",step_median_us=" << metrics.steps[metrics.steps.size() / 2]
            << ",step_p99_us=" << metrics.steps[metrics.steps.size() * 99 / 100]
            << ",plan_wall_us=" << metrics.planning_us
            << ",trajectory_duration_s=" << accepted.trajectory_duration
            << ",trajectory_time_scale=" << accepted.trajectory_time_scale
            << ",measurement_final_mode="
            << static_cast<unsigned>(measured.mode)
            << ",terminal_mode=" << static_cast<unsigned>(final.mode)
            << ",unexpected_stop=0,timed_succeeded=" << (timed && succeeded)
            << '\n';
}
} // namespace

void *operator new(std::size_t size) {
  if (count_new)
    ++new_calls;
  if (auto memory = std::malloc(size ? size : 1))
    return memory;
  throw std::bad_alloc();
}
void *operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void *memory) noexcept { std::free(memory); }
void operator delete[](void *memory) noexcept { std::free(memory); }
void operator delete(void *memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void *memory, std::size_t) noexcept {
  std::free(memory);
}

int main(int argc, char **argv) {
  try {
    require(argc == 2, "usage: trajectory_execution_audit MODEL.urdf");
    std::cout << std::setprecision(12);
    Config config;
    config.period = 0.002;
    const auto model_config = defaultModelConfig(argv[1]);
    std::cout << "method,offline=1,fixed_simulation_dt_s=" << config.period
              << ",warmup_s=2,measurement_s=" << duration
              << ",input_period_s=" << producer_period
              << ",stream_future_s=" << future
              << ",signal=q1_base+0.02*sin(pi*t/2)^3,retime=0\n";
    std::cout
        << "scope,timed_plan_has_complete_future=1,stream_has_40ms_future=1,"
           "same_model_gains_and_limits=1,not_a_wall_clock_or_MCU_test=1,"
           "no_claim_for_arbitrary_online_targets=1\n";
    std::cout << "config,velocity=" << config.velocity.transpose()
              << ",acceleration=" << config.acceleration.transpose()
              << ",jerk=" << config.jerk.transpose()
              << ",kp=" << config.kp.transpose()
              << ",kd=" << config.kd.transpose()
              << ",reference_preview_s=" << config.reference_preview << '\n';
    run(model_config, config, true);
    run(model_config, config, false);
    Model model(model_config);
    Vec base;
    base << 0, 0.8, -1.1, 0.3, 0, 0;
    const auto plan = JointTrajectory::timed(
        points(base), config, model.lower(), model.upper(), false);
    new_calls = 0;
    count_new = true;
    for (int i = 0; i < 2000; ++i)
      sample_sink += plan.sample(i * config.period).position[0];
    count_new = false;
    std::cout << "allocation,timed_sample_calls=2000,ordinary_cpp_new_calls="
              << new_calls << ",direct_malloc_and_aligned_new_not_counted=1\n";
    return 0;
  } catch (const std::exception &error) {
    count_new = false;
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
