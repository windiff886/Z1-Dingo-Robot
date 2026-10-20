#include "z1_control/runtime.hpp"
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
namespace {
volatile std::sig_atomic_t quit = 0;
void signalHandler(int) { quit = 1; }
} // namespace
int main(int argc, char **argv) {
  try {
    if (argc < 2) {
      std::cerr << "Usage: z1_standalone MODEL.urdf [duration_seconds] "
                   "[--demo]\nSimulation only; hardware is launched through "
                   "explicit ROS configuration.\n";
      return 2;
    }
    double duration = argc > 2 ? std::stod(argv[2]) : 10;
    if (!std::isfinite(duration) || duration <= 0)
      throw std::invalid_argument("duration must be positive");
    auto model = z1::defaultModelConfig(argv[1]);
    z1::Config config;
    z1::Runtime runtime(model, config,
                        std::make_unique<z1::Simulation>(model, config.period));
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    runtime.start();
    auto began = z1::monotonicTime();
    bool enabled = false, goal = false;
    while (!quit && z1::monotonicTime() - began < duration) {
      if (argc > 3 && std::string(argv[3]) == "--demo") {
        if (!enabled && runtime.controller().snapshot().feedback.sequence > 0) {
          runtime.controller().operate(z1::Operation::Enable);
          enabled = true;
        }
        if (!goal && z1::monotonicTime() - began > 1.5 &&
            runtime.controller().snapshot().mode == z1::Mode::Holding) {
          auto q = runtime.controller().snapshot().feedback.q;
          q[0] += 0.1;
          runtime.controller().goal(1, q, 0.2);
          goal = true;
        }
      }
      for (auto &e : runtime.controller().events())
        std::cout << "command " << e.id << " outcome "
                  << static_cast<int>(e.outcome) << ": " << e.reason << '\n';
      if (runtime.controller().snapshot().mode == z1::Mode::Fault) {
        std::cerr << runtime.controller().snapshot().reason << '\n';
        runtime.stop();
        return 1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    runtime.stop();
    auto s = runtime.controller().snapshot();
    std::cout << "feedback=" << s.feedback.sequence
              << " mode=" << static_cast<int>(s.mode)
              << " distance=" << s.collision_distance
              << " overruns=" << s.overruns << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
