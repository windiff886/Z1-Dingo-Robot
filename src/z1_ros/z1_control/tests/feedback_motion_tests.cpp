#include "z1_control/controller.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace z1;
namespace {
void require(bool value, const char *message) {
  if (!value)
    throw std::runtime_error(message);
}
Vec measuredHome() {
  Vec q;
  q << 0, 2.9508883017115295e-6, 0.00010619896784191951,
      -0.07443362474441528, 0, 1.179988612420857e-5;
  return q;
}
Config config() {
  Config c;
  c.period = .004;
  return c;
}
struct Fixture {
  Config limits{config()};
  Model model;
  Controller controller;
  Feedback feedback;
  Output output;
  double now{1};
  explicit Fixture(const ModelConfig &m)
      : model(m), controller(m, limits) {
    feedback.q = measuredHome();
  }
  void tick(double elapsed = .004) {
    now += elapsed;
    feedback.received_at = now;
    ++feedback.sequence;
    feedback.fresh = true;
    output = controller.step(feedback, now);
    if (output.enabled) {
      require(model.withinLimits(output.q), "command reference escaped bounds");
      require((output.tau.cwiseAbs().array() <= limits.torque.array()).all(),
              "torque limits changed");
    }
  }
  void warmNoise() {
    for (int i = 0; i < 40; ++i) {
      feedback.q = measuredHome();
      feedback.q[0] += (i % 2 ? 1 : -1) * 5e-6;
      feedback.dq.setConstant(.09);
      tick();
    }
  }
  Pose goal(double z = .3579) {
    Pose pose = model.fk(measuredHome());
    pose.translation() << .0377, 0, z;
    return pose;
  }
};

void boundedStarts(const ModelConfig &m) {
  Fixture pose(m);
  pose.tick();
  Vec solved;
  std::string failure;
  require(pose.model.ik(pose.goal(), measuredHome(), solved, .001, &failure),
          "reachable goal rejected for encoder-scale seed error");
  require(pose.model.withinLimits(solved), "IK relaxed target joint bounds");
  require(!pose.model.ik(pose.goal(), measuredHome(), solved, 0, &failure) &&
              failure.find("seed") != std::string::npos,
          "strict IK seed failure not distinguished");
  require(pose.controller.poseGoal(1, pose.goal(), .05),
          "measured RViz target still rejected at planning start");
  pose.tick();
  require(pose.output.enabled, "accepted first pose did not enable");

  for (int type = 0; type < 3; ++type) {
    Fixture f(m);
    f.tick();
    Vec target = measuredHome().cwiseMax(f.model.lower()).cwiseMin(f.model.upper());
    target[0] += .01;
    bool accepted = false;
    if (type == 0)
      accepted = f.controller.goal(1, target);
    else if (type == 1)
      accepted = f.controller.jointWaypoints(1, {target}, .5);
    else {
      TrajectoryPoint point;
      point.position = target;
      point.time_from_start = 1;
      accepted = f.controller.trajectory(1, {point});
    }
    require(accepted, "joint or prepared start still uses out-of-bounds feedback");
    f.tick();
    require(f.output.enabled, "bounded first reference faulted");
  }

  Fixture strict(m);
  strict.tick();
  require(!strict.controller.goal(1, measuredHome()),
          "measurement tolerance leaked into commanded goal bounds");
  strict.feedback.q[2] = .002;
  strict.tick();
  require(!strict.controller.snapshot().feedback_valid &&
              !strict.controller.poseGoal(2, strict.goal()) && !strict.output.enabled,
          "significant measured joint violation accepted");
}

void noisyStartSettleAndStop(const ModelConfig &m) {
  Fixture f(m);
  f.feedback.dq.setConstant(.09);
  f.tick();
  require(!f.controller.poseGoal(1, f.goal()),
          "noisy startup accepted before any stationary history");
  f.warmNoise();
  const Pose target = f.goal();
  require(f.controller.poseGoal(1, target, .05),
          "fixed position with raw velocity noise still rejected");
  bool succeeded = false;
  for (int i = 0; i < 6000; ++i) {
    if (f.output.enabled) {
      f.feedback.q = f.output.q;
      f.feedback.q[0] += (i % 2 ? 1 : -1) * 5e-6;
    }
    f.feedback.dq.setConstant(.09);
    f.tick();
    require(f.controller.snapshot().mode != Mode::Fault,
            "noisy feedback caused a false settling fault");
    if (f.controller.snapshot().mode == Mode::Holding) {
      succeeded = true;
      break;
    }
  }
  require(succeeded, "measured pose did not settle with noisy raw velocity");
  require(poseError(f.model.fk(f.feedback.q), target).head<3>().norm() < .005,
          "success reported away from measured TCP target");
  f.controller.operate(Operation::Stop);
  f.tick();
  for (int i = 0; i < 100 && f.controller.snapshot().mode == Mode::Stopping; ++i)
    f.tick();
  require(f.controller.snapshot().mode == Mode::AwaitResume,
          "stationary HOLD stop generated a brake from noisy raw velocity");
  f.controller.operate(Operation::Hold);
  f.tick();
  require(f.controller.snapshot().mode == Mode::Holding,
          "noise prevented explicit HOLD after stop");
}

void realMotionAndStaleHistory(const ModelConfig &m) {
  Fixture moving(m);
  for (int i = 0; i < 40; ++i) {
    moving.feedback.q[0] += .04 * moving.limits.period;
    moving.feedback.dq.setConstant(.09);
    moving.tick();
  }
  require(!moving.controller.poseGoal(1, moving.goal()),
          "real position travel was mistaken for velocity noise");
  moving.feedback.dq.setZero();
  moving.feedback.q[0] += .04 * moving.limits.period;
  moving.tick();
  require(!moving.controller.poseGoal(1, moving.goal()),
          "quiet velocity concealed measured motion");

  Fixture boundary(m);
  boundary.feedback.q[0] = boundary.model.upper()[0] - .004;
  boundary.tick();
  boundary.controller.operate(Operation::Enable);
  boundary.tick();
  for (int i = 0; i < 40; ++i) {
    boundary.feedback.q[0] += .02 * boundary.limits.period;
    boundary.feedback.dq[0] = .02;
    boundary.tick();
  }
  boundary.controller.operate(Operation::Stop);
  boundary.tick();
  require(boundary.controller.snapshot().mode == Mode::Fault &&
              !boundary.output.enabled,
          "slow real motion at the boundary lost measured-velocity braking");

  Fixture stale(m);
  stale.warmNoise();
  auto retained = stale.feedback;
  retained.fresh = false;
  stale.now += .09;
  stale.controller.step(retained, stale.now);
  stale.tick();
  require(!stale.controller.poseGoal(1, stale.goal()),
          "stale samples reused as a stationary window");

  Fixture repeated(m);
  repeated.feedback.dq.setConstant(.09);
  repeated.tick();
  for (int i = 0; i < 30; ++i) {
    repeated.now += .004;
    repeated.controller.step(repeated.feedback, repeated.now);
  }
  repeated.tick();
  require(!repeated.controller.poseGoal(1, repeated.goal()),
          "retained feedback manufactured stationary samples");
}

void gripperEncoderBounds(const ModelConfig &m) {
  Fixture f(m);
  f.feedback.has_gripper = true;
  f.feedback.gripper_q = 1.1803553206846118e-5;
  f.tick();
  f.controller.operate(Operation::Enable);
  f.tick();
  require(f.output.enabled && f.output.gripper_q == 0,
          "positive encoder noise escaped into a gripper command");
  require(f.controller.snapshot().feedback.gripper_q == f.feedback.gripper_q,
          "raw gripper feedback was overwritten");
  for (double measured : {-.5, -1.00001, .000011803553206846118}) {
    f.feedback.gripper_q = measured;
    f.tick();
    require(f.output.enabled && f.output.gripper_q >= -1 &&
                f.output.gripper_q <= 0,
            "unrequested gripper hold emitted an out-of-range command");
  }
  require(!f.controller.gripper(.00001) && !f.controller.gripper(-1.00001),
          "measurement allowance leaked into requested gripper targets");
  require(f.controller.gripper(-.3), "valid gripper target rejected");
  for (double measured : {.000011803553206846118, -1.00001}) {
    f.feedback.gripper_q = measured;
    f.tick();
    const double bounded = std::clamp(measured, -1.0, 0.0);
    require(f.output.enabled && f.output.gripper_q >= -1 &&
                f.output.gripper_q <= 0 &&
                std::abs(f.output.gripper_q - bounded) <=
                    .4 * f.limits.period + 1e-12,
            "explicit gripper target lost its bounds or rate limit");
  }
  f.feedback.gripper_q = .021;
  f.tick();
  require(!f.output.enabled && !f.controller.snapshot().feedback_valid,
          "significant gripper feedback violation accepted");
}

void stalledFeedbackIsNotSuccess(const ModelConfig &m) {
  Fixture f(m);
  f.tick();
  Pose target = Pose::Identity();
  target.translation() << .037793, .000004, .273669;
  target.linear() = Eigen::Quaterniond(.999335044, .000001936, -.036462385,
                                      .000053064).normalized().toRotationMatrix();
  require(f.controller.poseGoal(50, target, .05), "logged pose rejected");
  const auto deadline = f.now + f.controller.snapshot().trajectory_duration +
                        f.limits.settle_timeout + .1;
  bool moving_reference = false, nonzero_torque = false;
  while (f.now < deadline && f.controller.snapshot().mode != Mode::Fault) {
    // Deliberately retain real position while commands advance. A loopback
    // fixture following q directly cannot exercise this failure mode.
    f.tick();
    const auto s = f.controller.snapshot();
    moving_reference |= (s.reference_position - f.feedback.q).norm() > .1;
    nonzero_torque |= s.commanded_torque.norm() > 1;
  }
  const auto s = f.controller.snapshot();
  require(moving_reference && nonzero_torque,
          "stalled plant test did not exercise reference and torque telemetry");
  require(s.mode == Mode::Fault && !f.output.enabled,
          "stalled feedback was reported as successful motion");
  require(s.reason.find("tcp_position_error=") != std::string::npos &&
              s.reason.find("measured_q=[") != std::string::npos &&
              s.reason.find("reference_q=[") != std::string::npos &&
              s.reason.find("last_command_torque=[") != std::string::npos,
          "settling timeout discarded the tracking evidence");
  for (const auto &event : f.controller.events())
    require(event.outcome != Outcome::Succeeded,
            "stalled robot generated a false success event");
}

void replay(const ModelConfig &m, const char *path) {
  std::ifstream input(path);
  require(static_cast<bool>(input), "feedback replay file missing");
  Fixture f(m);
  std::string line;
  double previous = 0;
  unsigned count = 0;
  while (std::getline(input, line)) {
    std::istringstream sample(line);
    double time;
    require(static_cast<bool>(sample >> time), "invalid replay timestamp");
    for (int j = 0; j < 6; ++j)
      require(static_cast<bool>(sample >> f.feedback.q[j]), "invalid replay q");
    for (int j = 0; j < 6; ++j)
      require(static_cast<bool>(sample >> f.feedback.dq[j]), "invalid replay dq");
    f.tick(count ? time - previous : .004);
    require(f.controller.snapshot().feedback_valid, "captured feedback rejected");
    previous = time;
    ++count;
  }
  require(count > 20, "insufficient recorded feedback");
  require(f.controller.poseGoal(1, f.goal(), .05),
          "recorded real feedback still rejects reachable RViz target");
  f.tick();
  require(f.output.enabled, "replayed first target faulted");
  std::cout << "replayed_feedback_samples=" << count << '\n';
}
} // namespace

int main(int argc, char **argv) {
  try {
    require(argc >= 2, "model path required");
    const auto m = defaultModelConfig(argv[1]);
    boundedStarts(m);
    noisyStartSettleAndStop(m);
    realMotionAndStaleHistory(m);
    gripperEncoderBounds(m);
    stalledFeedbackIsNotSuccess(m);
    if (argc > 2)
      replay(m, argv[2]);
    std::cout << "Feedback motion regression tests passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
