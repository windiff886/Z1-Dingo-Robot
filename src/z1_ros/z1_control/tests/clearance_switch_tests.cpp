#include "z1_control/runtime.hpp"
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

using namespace z1;
namespace {
void require(bool value, const char *message) {
  if (!value)
    throw std::runtime_error(message);
}
Vec home() {
  Vec q = Vec::Zero();
  q[3] = -0.07435;
  return q;
}
Config commissioning() {
  Config c;
  c.period = 0.004;
  c.collision_check_enabled = false;
  return c;
}
struct Fixture {
  Config config;
  Controller controller;
  Feedback feedback;
  Output output;
  double now{1};
  Fixture(const ModelConfig &model, Config c, const Vec &q = home())
      : config(c), controller(model, c) {
    feedback.q = q;
    tick(false);
    require(!output.enabled, "startup implicitly enabled output");
  }
  void tick(bool follow = true) {
    if (follow && output.enabled) {
      feedback.q = output.q;
      feedback.dq = output.dq;
      feedback.tau = output.tau;
    }
    now += config.period;
    ++feedback.sequence;
    feedback.received_at = now;
    feedback.fresh = true;
    output = controller.step(feedback, now);
    require((output.tau.cwiseAbs().array() <= config.torque.array()).all(),
            "torque limit lost with clearance checks off");
    require((output.dq.cwiseAbs().array() <= config.velocity.array() + 1e-8).all(),
            "velocity limit lost with clearance checks off");
  }
  void enable() {
    controller.operate(Operation::Enable);
    tick();
    require(controller.snapshot().mode == Mode::Holding && output.enabled,
            "commissioning enable failed");
  }
  void finish(Mode expected = Mode::Holding) {
    for (unsigned i = 0; i < 5000; ++i) {
      tick();
      const auto mode = controller.snapshot().mode;
      require(mode != Mode::Fault, "motion faulted with clearance checks off");
      if (mode == expected)
        return;
      require(expected == Mode::AwaitResume || mode != Mode::AwaitResume,
              "geometry unexpectedly stopped commissioning motion");
    }
    throw std::runtime_error("offline motion did not finish");
  }
};

void enableAndMotion(const ModelConfig &model) {
  require(Config{}.collision_check_enabled, "native default must retain geometry protection");
  Fixture protected_home(model, Config{});
  protected_home.controller.operate(Operation::Enable);
  protected_home.tick();
  require(protected_home.controller.snapshot().mode == Mode::Disabled &&
              !protected_home.output.enabled,
          "default geometry gate no longer rejects close home pose");

  Fixture f(model, commissioning());
  f.enable();
  require(f.controller.snapshot().collision_distance < .008,
          "test no longer covers hard geometric margin");
  for (unsigned i = 0; i < 50; ++i)
    f.tick();
  require(f.controller.snapshot().mode == Mode::Holding,
          "holding still applies clearance warning or fault");

  Vec target = home();
  target[0] = .02;
  require(f.controller.goal(1, target, .05), "finite joint goal blocked by geometry");
  f.finish();
  require((f.output.q - target).norm() < 1e-8, "finite goal failed to track");
  target[0] += .01;
  Model kinematics(model);
  require(f.controller.poseGoal(2, kinematics.fk(target), .05),
          "RViz pose goal blocked with checks off");
  f.finish();
  target = f.output.q;
  target[0] += .01;
  require(f.controller.jointWaypoints(3, {target}, .05),
          "prepared waypoint blocked by geometry");
  f.finish();

  TrajectoryPoint point;
  point.position = f.output.q;
  point.position[0] += .01;
  point.time_from_start = 1;
  require(f.controller.trajectory(4, {point}), "timed path blocked by geometry");
  f.finish();
  f.controller.operate(Operation::JointStream);
  f.tick();
  const auto session = f.controller.snapshot().session;
  target = f.output.q;
  target[0] += .03;
  for (unsigned i = 1; i < 100; ++i) {
    require(f.controller.stream(session, i, target, f.now), "stream rejected");
    f.tick();
    require(f.controller.snapshot().mode == Mode::JointStream,
            "stream braking lookahead still enforces geometry");
  }
  f.controller.operate(Operation::Stop);
  f.tick();
  f.finish(Mode::AwaitResume);
  f.controller.operate(Operation::Hold);
  f.tick();
  require(f.controller.snapshot().mode == Mode::Holding,
          "hold after stop still enforces geometry");
  f.controller.operate(Operation::Disable);
  f.tick();
  require(!f.output.enabled && f.controller.snapshot().mode == Mode::Disabled,
          "disable lost with geometry off");
}

void remainingProtections(const ModelConfig &model) {
  Fixture f(model, commissioning());
  f.enable();
  Model bounds(model);
  Vec bad = home();
  bad[0] = bounds.upper()[0] + .1;
  require(!f.controller.goal(1, bad), "joint goal limit disabled");
  require(!f.controller.jointWaypoints(2, {bad}), "prepared joint limit disabled");
  bad = home();
  bad[0] = std::numeric_limits<double>::quiet_NaN();
  require(!f.controller.goal(3, bad), "nonfinite target accepted");

  // Braking must still reject a polynomial that exits a joint limit, even
  // though all geometric checks are disabled.
  f.feedback.q = home();
  f.feedback.q[0] = bounds.upper()[0] - 1e-8;
  f.feedback.dq[0] = .02;
  f.controller.operate(Operation::Stop);
  f.tick(false);
  require(f.controller.snapshot().mode == Mode::Fault && !f.output.enabled,
          "joint-bounded braking lost with checks off");
  require(f.controller.snapshot().reason.find("joint limits") != std::string::npos,
          "braking test failed for unrelated reason");

  Fixture motor(model, commissioning());
  motor.enable();
  motor.feedback.motor_fault = true;
  motor.tick(false);
  require(motor.controller.snapshot().mode == Mode::Fault && !motor.output.enabled,
          "motor fault protection disabled");

  Fixture stale(model, commissioning());
  stale.enable();
  stale.now += .09;
  stale.feedback.fresh = false;
  stale.output = stale.controller.step(stale.feedback, stale.now);
  require(stale.controller.snapshot().mode == Mode::Fault && !stale.output.enabled,
          "feedback/deadline protection disabled");
}

void configuredObstacle(const ModelConfig &model) {
  auto blocked = model;
  Model kinematics(model);
  FixedBox box;
  box.name = "offline_test_obstacle";
  box.size = Eigen::Vector3d::Constant(.2);
  box.placement = kinematics.fk(home());
  blocked.fixed_boxes.push_back(box);
  Fixture f(blocked, commissioning());
  f.enable();
  require(f.controller.snapshot().collision_distance < 0,
          "test no longer covers configured obstacle enforcement");
  Vec target = home();
  target[0] = .01;
  require(f.controller.goal(1, target, .05), "configured-obstacle geometry still enforced");
  f.finish();
}

void runtimeCommissioning(const ModelConfig &model) {
  struct Follow final : Transport {
    Vec q{home()};
    uint64_t sequence{};
    bool last_enabled{};
    Exchange exchange(const Output &out, double) override {
      if (out.enabled)
        q = out.q;
      last_enabled = out.enabled;
      Feedback f;
      f.q = q;
      f.dq = out.enabled ? out.dq : Vec::Zero();
      f.tau = out.enabled ? out.tau : Vec::Zero();
      f.sequence = ++sequence;
      f.fresh = true;
      f.received_at = monotonicTime();
      return {f, true, {}};
    }
  };
  auto transport = std::make_unique<Follow>();
  auto *recorded = transport.get();
  Runtime runtime(model, commissioning(), std::move(transport));
  auto wait = [&](auto ready) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!ready()) {
      require(std::chrono::steady_clock::now() < deadline,
              "250 Hz offline runtime did not reach expected state");
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  };
  runtime.start();
  wait([&] { return runtime.controller().snapshot().feedback_valid; });
  runtime.controller().operate(Operation::Enable);
  wait([&] { return runtime.controller().snapshot().mode == Mode::Holding; });
  Vec target = home();
  target[0] = .02;
  require(runtime.controller().goal(1, target, .05), "250 Hz runtime rejected goal");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  runtime.stop();
  require(runtime.controller().snapshot().mode == Mode::AwaitResume,
          "250 Hz runtime did not finish controlled shutdown");
  require(!recorded->last_enabled, "runtime shutdown omitted passive output");
}
} // namespace

int main(int argc, char **argv) {
  try {
    require(argc == 2, "model path required");
    const auto model = defaultModelConfig(argv[1]);
    enableAndMotion(model);
    remainingProtections(model);
    configuredObstacle(model);
    runtimeCommissioning(model);
    std::cout << "Clearance switch: offline enable, motion, stop and retained protections passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
