#include "z1_control/trajectory.hpp"

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#include <stdexcept>

namespace {
std::atomic<bool> count_allocations{false};
std::atomic<unsigned> allocations{0};
void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}
template <typename F> void reject(F &&action, const char *message) {
  try {
    action();
  } catch (const std::invalid_argument &) {
    return;
  }
  throw std::runtime_error(message);
}
} // namespace

void *operator new(std::size_t size) {
  if (count_allocations.load(std::memory_order_relaxed))
    allocations.fetch_add(1, std::memory_order_relaxed);
  if (void *memory = std::malloc(size ? size : 1))
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

int main() {
  try {
    using namespace z1;
    Config config;
    config.velocity.setConstant(20);
    config.acceleration.setConstant(100);
    config.jerk.setConstant(1000);
    const Vec lower = Vec::Constant(-3), upper = Vec::Constant(3);
    std::vector<TrajectoryPoint> points(3);
    points[0].time_from_start = 0;
    points[1].time_from_start = 0.7;
    points[2].time_from_start = 1.9;
    for (size_t i = 0; i < points.size(); ++i) {
      points[i].position.setConstant(0.2 * i);
      points[i].velocity.setConstant(0.1 - 0.03 * i);
      points[i].acceleration.setConstant(0.2 - 0.07 * i);
      points[i].has_velocity = points[i].has_acceleration = true;
    }
    const auto trajectory =
        JointTrajectory::timed(points, config, lower, upper);
    require(trajectory.duration() == 1.9 && trajectory.timeScale() == 1 &&
                trajectory.times()[1] == 0.7,
            "timed trajectory changed knot times");
    for (const auto &point : points) {
      const auto sample = trajectory.sample(point.time_from_start);
      require((sample.position - point.position).norm() < 1e-12 &&
                  (sample.velocity - point.velocity).norm() < 1e-12 &&
                  (sample.acceleration - point.acceleration).norm() < 1e-11,
              "explicit endpoint derivatives were changed");
    }
    const auto before = trajectory.sample(-1);
    const auto after = trajectory.sample(10);
    require((before.position - points.front().position).norm() < 1e-12 &&
                (after.position - points.back().position).norm() < 1e-12 &&
                before.velocity.isZero() && after.acceleration.isZero() &&
                after.jerk.isZero(),
            "out of range sampling did not hold endpoint");

    auto limits = config;
    limits.velocity.setConstant(0.05);
    limits.acceleration.setConstant(0.1);
    limits.jerk.setConstant(0.4);
    reject([&] { JointTrajectory::timed(points, limits, lower, upper); },
           "infeasible exact timing was silently altered");
    const auto retimed =
        JointTrajectory::timed(points, limits, lower, upper, true);
    const double scale = retimed.timeScale();
    require(scale > 1 && retimed.duration() == trajectory.duration() * scale,
            "retiming did not report global scale");
    for (int i = 0; i <= 1000; ++i) {
      const double time = trajectory.duration() * i / 1000;
      const auto original = trajectory.sample(time);
      const auto stretched = retimed.sample(time * scale);
      require(
          (original.position - stretched.position).norm() < 1e-12 &&
              (original.velocity / scale - stretched.velocity).norm() < 1e-12 &&
              (original.acceleration / (scale * scale) - stretched.acceleration)
                      .norm() < 1e-12,
          "retiming changed path geometry or derivative scaling");
      require((stretched.velocity.cwiseAbs().array() <=
               limits.velocity.array() * (1 + 1e-10))
                      .all() &&
                  (stretched.acceleration.cwiseAbs().array() <=
                   limits.acceleration.array() * (1 + 1e-10))
                      .all() &&
                  (stretched.jerk.cwiseAbs().array() <=
                   limits.jerk.array() * (1 + 1e-10))
                      .all(),
              "retimed segment still exceeds dynamic limits");
    }
    const auto further_scaled = retimed.scaled(2);
    require(further_scaled.timeScale() == scale * 2 &&
                further_scaled.duration() == retimed.duration() * 2 &&
                (further_scaled.sample(0.9 * scale * 2).position -
                 trajectory.sample(0.9).position)
                        .norm() < 1e-12,
            "additional global scaling changed path");

    std::vector<Vec> passing{Vec::Zero(), Vec::Constant(0.1),
                             Vec::Constant(0.2)};
    const auto through =
        JointTrajectory::waypoints(passing, Config{}, lower, upper);
    const double knot = through.times()[1];
    const auto left = through.sample(knot - 1e-7);
    const auto at = through.sample(knot);
    const auto right = through.sample(knot + 1e-7);
    require(at.velocity.minCoeff() > 0.01 &&
                (left.position - right.position).norm() < 1e-6 &&
                (left.velocity - right.velocity).norm() < 1e-6 &&
                (left.acceleration - right.acceleration).norm() < 1e-4,
            "internal waypoint is not a C2 continuous passing point");
    require(through.sample(0).velocity.isZero() &&
                through.sample(through.duration()).velocity.norm() < 1e-12,
            "automatic endpoints do not start/end at rest");
    const auto slow =
        JointTrajectory::waypoints(passing, Config{}, lower, upper, 0.5);
    require(std::abs(slow.duration() - 2 * through.duration()) < 1e-10,
            "waypoint speed does not scale the whole execution");

    std::vector<TrajectoryPoint> interval(2);
    interval[1].time_from_start = 1;
    interval[1].position.setConstant(1);
    auto interval_limits = config;
    interval_limits.velocity.setConstant(1.8); // Peak 1.875 at s=0.5.
    reject(
        [&] {
          JointTrajectory::timed(interval, interval_limits, lower, upper);
        },
        "interior velocity extremum was missed");
    interval_limits = config;
    interval_limits.acceleration.setConstant(5.7); // Peak 5.7735.
    reject(
        [&] {
          JointTrajectory::timed(interval, interval_limits, lower, upper);
        },
        "interior acceleration extremum was missed");
    interval_limits = config;
    interval_limits.jerk.setConstant(59.9); // Peak 60.
    reject(
        [&] {
          JointTrajectory::timed(interval, interval_limits, lower, upper);
        },
        "jerk extremum was missed");
    // q(s)=16*s^2*(1-s)^2: endpoints are both zero, interior maximum is 1.
    interval[1].position.setZero();
    for (auto &point : interval) {
      point.acceleration.setConstant(32);
      point.has_velocity = point.has_acceleration = true;
    }
    reject(
        [&] {
          JointTrajectory::timed(interval, config, lower, Vec::Constant(0.5),
                                 true);
        },
        "position overshoot between valid knots was missed");

    // Integral of (x-.5)^2*(x-.2)*(x-.8), over x in [0,.6].
    // The velocity polynomial has a repeated interior root as well as the
    // turning point at x=.2; the endpoints remain below the chosen upper bound.
    interval[0].position.setZero();
    interval[0].velocity.setConstant(0.024);
    interval[0].acceleration.setConstant(-0.1476);
    interval[1].position.setConstant(0.002472);
    interval[1].velocity.setConstant(-0.00048);
    interval[1].acceleration.setConstant(-0.00504);
    reject(
        [&] {
          JointTrajectory::timed(interval, config, lower,
                                 Vec::Constant(0.0027));
        },
        "repeated-root polynomial hid an interior position extremum");
    // A degenerate, linear polynomial should not acquire artificial extrema.
    interval[0].position.setZero();
    interval[1].position.setConstant(0.1);
    for (auto &point : interval) {
      point.velocity.setConstant(0.1);
      point.acceleration.setZero();
    }
    const auto linear = JointTrajectory::timed(interval, config, lower, upper);
    require((linear.sample(0.37).position - Vec::Constant(0.037)).norm() <
                    1e-12 &&
                linear.sample(0.37).acceleration.norm() < 1e-12,
            "low-order polynomial sampling changed linear motion");

    auto invalid = points;
    invalid[1].time_from_start = 0;
    reject([&] { JointTrajectory::timed(invalid, config, lower, upper); },
           "nonincreasing time accepted");
    invalid = points;
    invalid[0].time_from_start = 1e-6;
    reject([&] { JointTrajectory::timed(invalid, config, lower, upper); },
           "nonzero starting time accepted");
    invalid = points;
    invalid[1].has_velocity = false;
    reject([&] { JointTrajectory::timed(invalid, config, lower, upper); },
           "acceleration without velocity accepted");
    invalid = points;
    invalid[1].position[0] = std::numeric_limits<double>::quiet_NaN();
    reject([&] { JointTrajectory::timed(invalid, config, lower, upper); },
           "nonfinite trajectory accepted");
    reject([&] { trajectory.scaled(0.5); }, "unsafe speedup accepted");

    allocations = 0;
    count_allocations = true;
    double checksum = 0;
    for (int i = 0; i < 10000; ++i)
      checksum += through.sample(through.duration() * i / 9999).position.sum();
    count_allocations = false;
    require(checksum > 0 && allocations == 0,
            "prepared trajectory sampling allocated memory");
    std::cout << "trajectory tests passed: exact knots, C2 passing, polynomial "
                 "extrema, retiming, zero-allocation sampling\n";
    return 0;
  } catch (const std::exception &error) {
    count_allocations = false;
    std::cerr << error.what() << '\n';
    return 1;
  }
}
