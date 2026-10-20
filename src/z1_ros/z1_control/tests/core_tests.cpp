#include "z1_control/runtime.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace z1;
void require(bool b, const char *why) {
  if (!b)
    throw std::runtime_error(why);
}
struct Fixture {
  Config config;
  ModelConfig model;
  Controller c;
  Feedback f;
  double now{1};
  Output output;
  Fixture(const std::string &path)
      : model(defaultModelConfig(path)), c(model, config) {
    f.q << (0), 0.8, -1.1, 0.3, 0, 0;
    tick(false);
  }
  void tick(bool follow = true) {
    now += config.period;
    f.received_at = now;
    f.sequence++;
    f.fresh = true;
    if (follow && output.enabled) {
      f.q = output.q;
      f.dq = output.dq;
      f.tau = output.tau;
    }
    output = c.step(f, now);
  }
  void enable() {
    c.operate(Operation::Enable);
    tick();
    require(c.snapshot().mode == Mode::Holding, "enable failed");
  }
  void settle() {
    for (int i = 0; i < 2000; ++i) {
      tick();
      auto mode = c.snapshot().mode;
      if (mode == Mode::Holding || mode == Mode::AwaitResume)
        return;
      require(mode != Mode::Fault, "unexpected fault");
    }
    throw std::runtime_error("motion did not settle");
  }
};
int main(int argc, char **argv) {
  try {
    require(argc == 2, "model argument missing");
    auto mc = defaultModelConfig(argv[1]);
    Model m(mc);
    Vec q;
    q << 0, 0.8, -1.1, 0.3, 0, 0;
    require(m.withinLimits(q), "nominal state outside limits");
    std::cout << "nominal clearance=" << m.clearance(q).distance
              << " pair=" << m.clearance(q).pair << '\n';
    Vec solved;
    Pose target = m.fk(q);
    target.translation().z() += 0.003;
    require(m.ik(target, q, solved), "reachable IK failed");
    require(poseError(m.fk(solved), target).norm() < 0.001, "IK residual");
    Vec g = m.inverseDynamics(q, Vec::Zero(), Vec::Zero());
    require(m.forwardDynamics(q, Vec::Zero(), g).norm() < 1e-8,
            "gravity compensation counted incorrectly");
    Pose far = target;
    far.translation().x() = 100;
    require(!m.ik(far, q, solved), "unreachable IK accepted");
    auto obstacle = mc;
    FixedBox box;
    box.name = "mount";
    box.size = Eigen::Vector3d(0.1, 0.1, 0.1);
    box.placement.translation() = m.fk(q).translation();
    obstacle.fixed_boxes.push_back(box);
    Model blocked(obstacle);
    require(blocked.clearance(q).distance < 0, "fixed structure ignored");
    // Safe endpoints with an unsafe middle: a fixture intersects the TCP sweep,
    // not the endpoints.
    auto sweep = mc;
    box.name = "sweep_obstacle";
    box.size = Eigen::Vector3d::Constant(0.015);
    box.placement.translation() = m.fk(q) * Eigen::Vector3d(0.045, 0, 0);
    sweep.fixed_boxes.push_back(box);
    Model sweep_model(sweep);
    Vec from = q, to = q;
    from[0] = -1;
    to[0] = 1;
    require(sweep_model.clearance(from).distance > 0.03 &&
                sweep_model.clearance(to).distance > 0.03,
            "collision test endpoints are unsafe");
    Config cfg;
    Controller guarded(sweep, cfg);
    Feedback sf;
    sf.q = from;
    sf.fresh = true;
    sf.sequence = 1;
    sf.received_at = 1;
    guarded.step(sf, 1);
    guarded.operate(Operation::Enable);
    sf.sequence++;
    sf.received_at = 1.004;
    guarded.step(sf, 1.004);
    require(!guarded.goal(1, to, 0.3),
            "unsafe intermediate trajectory accepted");
    Fixture a(argv[1]);
    a.enable();
    Vec goal = q;
    goal[0] = 0.08;
    require(a.c.goal(10, goal, 0.3), "safe finite goal rejected");
    require(!a.c.goal(11, goal), "busy finite goal accepted");
    Vec previous_v = a.output.dq, previous_a = Vec::Zero(),
        previous_tau = a.output.tau;
    for (int i = 0; i < 2000 && a.c.snapshot().mode != Mode::Holding; ++i) {
      a.tick();
      require(a.c.snapshot().mode != Mode::Fault &&
                  a.c.snapshot().mode != Mode::AwaitResume,
              "safe goal stopped unexpectedly");
      Vec acceleration = (a.output.dq - previous_v) / a.config.period;
      require(
          (a.output.dq.cwiseAbs().array() <= a.config.velocity.array() + 1e-6)
              .all(),
          "velocity bound failed");
      require((acceleration.cwiseAbs().array() <=
               a.config.acceleration.array() + 1e-5)
                  .all(),
              "acceleration bound failed");
      require(
          ((acceleration - previous_a).cwiseAbs().array() / a.config.period <=
           a.config.jerk.array() + 1e-3)
              .all(),
          "jerk bound failed");
      require(
          ((a.output.tau - previous_tau).cwiseAbs().array() / a.config.period <=
           a.config.torque_rate.array() + 1e-3)
              .all(),
          "torque slew bound failed");
      previous_v = a.output.dq;
      previous_a = acceleration;
      previous_tau = a.output.tau;
    }
    require(a.c.snapshot().mode == Mode::Holding &&
                (a.f.q - goal).norm() < 0.02,
            "finite goal not reached");
    auto events = a.c.events();
    bool success = false;
    for (auto &e : events)
      success |= e.id == 10 && e.outcome == Outcome::Succeeded;
    require(success, "measured success event missing");
    Vec bad = goal;
    bad[2] = std::numeric_limits<double>::quiet_NaN();
    require(!a.c.goal(12, bad), "NaN target accepted");
    require(a.c.poseGoal(13, m.fk(goal), 0.3), "finite pose target rejected");
    a.settle();
    for (Mode mode : {Mode::JointStream, Mode::PoseStream, Mode::Impedance}) {
      Fixture x(argv[1]);
      x.enable();
      x.c.operate(mode == Mode::JointStream  ? Operation::JointStream
                  : mode == Mode::PoseStream ? Operation::PoseStream
                                             : Operation::Impedance);
      x.tick();
      if (x.c.snapshot().mode != mode)
        std::cerr << "mode entry " << static_cast<int>(mode) << " -> "
                  << static_cast<int>(x.c.snapshot().mode) << ": "
                  << x.c.snapshot().reason << '\n';
      auto session = x.c.snapshot().session;
      Pose p = m.fk(q);
      Vec joint = q;
      joint[0] += 0.015;
      require(
          mode == Mode::JointStream
              ? !x.c.stream(session, 1, joint, x.now - x.config.stream_timeout)
              : !x.c.stream(session, 1, p, x.now - x.config.stream_timeout),
          "expired native target accepted");
      require(mode == Mode::JointStream
                  ? x.c.stream(session, 1, joint, x.now - 0.01)
                  : x.c.stream(session, 1, p, x.now - 0.01),
              "valid aged or concurrent target rejected");
      require(mode == Mode::JointStream ? x.c.stream(session, 2, joint, x.now)
                                        : x.c.stream(session, 2, p, x.now),
              "stream target rejected");
      for (int i = 0; i < 30; ++i) {
        x.tick();
        require(x.output.tau.cwiseAbs().maxCoeff() <= 30,
                "torque limit failed");
      }
      for (int i = 0; i < 150; ++i)
        x.tick();
      x.settle();
      require(x.c.snapshot().mode == Mode::AwaitResume,
              "timeout did not latch");
      require(!x.c.stream(session, 100, joint, x.now),
              "stale session restarted motion");
      x.c.operate(Operation::Resume);
      x.tick();
      auto new_session = x.c.snapshot().session;
      require(new_session != session, "resume session unchanged");
      require(!x.c.stream(session, 101, p, x.now), "old pose token accepted");
      require(mode == Mode::JointStream ? x.c.stream(new_session, 1, q, x.now)
                                        : x.c.stream(new_session, 1, p, x.now),
              "fresh resumed target rejected");
    }
    Fixture lost(argv[1]);
    lost.enable();
    auto cached = lost.f;
    cached.fresh = false;
    for (int i = 0; i < 30; ++i) {
      lost.now += 0.004;
      lost.output = lost.c.step(cached, lost.now);
    }
    require(lost.c.snapshot().mode == Mode::Fault && !lost.output.enabled,
            "stale feedback did not fault");
    require(!lost.c.snapshot().feedback_valid,
            "stale feedback advertised as valid");
    Fixture corrupt(argv[1]);
    corrupt.enable();
    corrupt.f.q[0] = std::numeric_limits<double>::quiet_NaN();
    corrupt.tick(false);
    require(corrupt.c.snapshot().mode == Mode::Fault &&
                !corrupt.c.snapshot().feedback_valid && !corrupt.output.enabled,
            "nonfinite feedback advertised as valid");
    Fixture independent(argv[1]);
    independent.enable();
    require(independent.c.snapshot().mode == Mode::Holding,
            "instances share fault state");
    Config invalid;
    invalid.velocity[0] = -1;
    bool threw = false;
    try {
      Controller c(mc, invalid);
    } catch (const std::exception &) {
      threw = true;
    }
    require(threw, "invalid configuration accepted");
    bool factory_called = false;
    threw = false;
    try {
      Runtime runtime(mc, invalid, [&]() -> std::unique_ptr<Transport> {
        factory_called = true;
        return nullptr;
      });
    } catch (const std::exception &) {
      threw = true;
    }
    require(threw && !factory_called,
            "transport opened before configuration validation");
    std::cout << "PASS: model, IK, dynamics, fixed collision, finite goals, "
                 "both streams, impedance, session timeout, feedback failure "
                 "and instance isolation\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
