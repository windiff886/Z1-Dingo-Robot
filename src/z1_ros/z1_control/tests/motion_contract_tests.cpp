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
            "torque limit lost");
    require(
        (output.dq.cwiseAbs().array() <= config.velocity.array() + 1e-8).all(),
        "velocity limit lost");
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
      require(mode != Mode::Fault, "motion faulted");
      if (mode == expected)
        return;
      require(expected == Mode::AwaitResume || mode != Mode::AwaitResume,
              "motion stopped unexpectedly");
    }
    throw std::runtime_error("offline motion did not finish");
  }
};

void enableAndMotion(const ModelConfig &model) {
  Fixture startup(model, Config{});
  startup.enable();
  require(startup.controller.snapshot().mode == Mode::Holding,
          "normal folded home pose must permit holding");

  Fixture f(model, commissioning());
  f.enable();
  require(std::isnan(f.controller.snapshot().collision_distance) &&
              f.controller.snapshot().collision_pair.empty(),
          "legacy geometry status must report unavailable");
  for (unsigned i = 0; i < 50; ++i)
    f.tick();
  require(f.controller.snapshot().mode == Mode::Holding,
          "holding did not remain active");

  Vec target = home();
  target[0] = .02;
  require(f.controller.goal(1, target, .05),
          "finite joint goal rejected");
  f.finish();
  require((f.output.q - target).norm() < 1e-8, "finite goal failed to track");
  target[0] += .01;
  Model kinematics(model);
  require(f.controller.poseGoal(2, kinematics.fk(target), .05),
          "RViz pose goal rejected");
  f.finish();
  target = f.output.q;
  target[0] += .01;
  require(f.controller.jointWaypoints(3, {target}, .05),
          "prepared waypoint rejected");
  f.finish();

  TrajectoryPoint point;
  point.position = f.output.q;
  point.position[0] += .01;
  point.time_from_start = 1;
  require(f.controller.trajectory(4, {point}),
          "timed path rejected");
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
            "stream tracking stopped unexpectedly");
  }
  f.controller.operate(Operation::Stop);
  f.tick();
  f.finish(Mode::AwaitResume);
  f.controller.operate(Operation::Hold);
  f.tick();
  require(f.controller.snapshot().mode == Mode::Holding,
          "hold after stop failed");
  f.controller.operate(Operation::Disable);
  f.tick();
  require(!f.output.enabled && f.controller.snapshot().mode == Mode::Disabled,
          "disable failed");
}

void remainingProtections(const ModelConfig &model) {
  Fixture f(model, commissioning());
  f.enable();
  Model bounds(model);
  Vec bad = home();
  bad[0] = bounds.upper()[0] + .1;
  require(!f.controller.goal(1, bad), "joint goal limit disabled");
  require(!f.controller.jointWaypoints(2, {bad}),
          "prepared joint limit disabled");
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
  require(f.controller.snapshot().reason.find("joint limits") !=
              std::string::npos,
          "braking test failed for unrelated reason");

  Fixture motor(model, commissioning());
  motor.enable();
  motor.feedback.motor_fault = true;
  motor.tick(false);
  require(motor.controller.snapshot().mode == Mode::Fault &&
              !motor.output.enabled,
          "motor fault protection disabled");

  Fixture stale(model, commissioning());
  stale.enable();
  stale.now += .09;
  stale.feedback.fresh = false;
  stale.output = stale.controller.step(stale.feedback, stale.now);
  require(stale.controller.snapshot().mode == Mode::Fault &&
              !stale.output.enabled,
          "feedback/deadline protection disabled");
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
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
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
  require(runtime.controller().goal(1, target, .05),
          "250 Hz runtime rejected goal");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  runtime.stop();
  require(runtime.controller().snapshot().mode == Mode::AwaitResume,
          "250 Hz runtime did not finish controlled shutdown");
  require(!recorded->last_enabled, "runtime shutdown omitted passive output");
}
void firstGoalAndTiming(const ModelConfig &model) {
  Fixture f(model, commissioning());
  Vec target = home();
  target[0] += .01;
  require(f.controller.goal(1, target, .05),
          "first joint goal requires manual enable");
  f.finish();
  f.controller.operate(Operation::Disable);
  f.tick();
  Model kinematics(model);
  target[0] += .01;
  require(f.controller.poseGoal(2, kinematics.fk(target), .05),
          "first pose goal requires manual enable");
  f.finish();
  require(!f.controller.goal(2, target) &&
              !f.controller.jointWaypoints(1, {target}),
          "finite command IDs were not shared across motion interfaces");

  Fixture prepared(model, commissioning());
  require(prepared.controller.jointWaypoints(1, {target}, .05),
          "first prepared path requires manual enable");
  prepared.finish();

  Fixture timed(model, commissioning());
  TrajectoryPoint endpoint;
  endpoint.time_from_start = 1;
  endpoint.position = target;
  require(timed.controller.trajectory(1, {endpoint}),
          "first timed path requires manual enable");
  timed.finish();

  Vec nominal;
  nominal << 0, .8, -1.1, .3, 0, 0;
  Fixture linear(model, commissioning(), nominal);
  auto pose_target = kinematics.fk(nominal);
  pose_target.translation().z() += .003;
  require(linear.controller.moveL(1, pose_target, .3),
          "first linear path requires manual enable");
  linear.finish();

  Fixture invalid(model, commissioning());
  auto bad = target;
  bad[0] = kinematics.upper()[0] + .1;
  require(!invalid.controller.goal(1, bad), "invalid first goal accepted");
  invalid.tick();
  require(!invalid.output.enabled &&
              invalid.controller.snapshot().mode == Mode::Disabled,
          "rejected first goal enabled output");
  invalid.feedback.dq[0] = .04;
  invalid.tick(false);
  require(!invalid.controller.goal(2, target),
          "moving passive arm accepted a first goal");
  invalid.feedback.dq.setZero();
  invalid.tick(false);
  invalid.now += .09;
  invalid.feedback.fresh = false;
  invalid.output = invalid.controller.step(invalid.feedback, invalid.now);
  require(!invalid.controller.goal(3, target),
          "stale passive feedback accepted a first goal");

  // Force a compute overrun with a synthetic 100 ns budget, without depending
  // on operating-system sleep precision. Valid feedback must retain output.
  auto short_period = commissioning();
  short_period.period = 1e-7;
  Fixture overrun(model, short_period);
  overrun.enable();
  require(overrun.controller.snapshot().overruns > 0 && overrun.output.enabled,
          "a compute overrun disabled otherwise valid control");

  Fixture gains(model, commissioning());
  gains.enable();
  require(gains.controller.gains(Vec::Constant(600), Vec::Constant(101)),
          "arbitrary gain ceiling remains");
  require(!gains.controller.gains(Vec::Constant(-1), Vec::Ones()),
          "negative impedance gain accepted");
  gains.tick();

  auto configured = commissioning();
  configured.kp.setConstant(6000);
  configured.kd.setConstant(201);
  configured.stiffness.setConstant(600);
  configured.damping.setConstant(101);
  Fixture tuned(model, configured);
  tuned.enable();
  tuned.feedback.q[0] -= .1;
  tuned.tick(false); // Fixture checks the physical torque limit on every tick.
}

void disablePriority(const ModelConfig &model) {
  Fixture f(model, commissioning());
  Vec target = home();
  target[0] += .01;
  require(f.controller.goal(100, target), "goal rejected before disable test");
  for (unsigned i = 0; i < 20; ++i)
    f.controller.operate(Operation::JointStream, i + 1);
  f.controller.operate(Operation::Disable, 200);
  auto bad = f.feedback;
  ++bad.sequence;
  bad.received_at = f.now + f.config.period;
  bad.q[0] = std::numeric_limits<double>::quiet_NaN();
  const auto out = f.controller.step(bad, bad.received_at);
  require(!out.enabled && f.controller.snapshot().mode == Mode::Disabled,
          "disable lost priority to invalid feedback or queued operations");
  int accepted = 0, canceled = 0;
  for (const auto &event : f.controller.events()) {
    accepted += event.id == 200 && event.outcome == Outcome::Accepted;
    canceled += event.id == 100 && event.outcome == Outcome::Canceled;
  }
  require(accepted == 1 && canceled == 1,
          "disable did not produce exactly one acknowledgment and cancellation");
}
} // namespace

int main(int argc, char **argv) {
  try {
    require(argc == 2, "model path required");
    const auto model = defaultModelConfig(argv[1]);
    enableAndMotion(model);
    remainingProtections(model);
    runtimeCommissioning(model);
    firstGoalAndTiming(model);
    disablePriority(model);
    std::cout << "Motion contract: first goals, stop and "
                 "physical limits passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
