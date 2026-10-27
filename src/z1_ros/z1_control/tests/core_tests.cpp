#include "z1_control/runtime.hpp"
#include <cmath>
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
    require(guarded.goal(1, to, 0.3),
            "diagnostic geometry blocked a joint-valid trajectory");
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
    // A future q/dq/ddq stream must track without coming to rest at each
    // producer update, while retaining the configured motion bounds.
    Fixture scheduled(argv[1]);
    scheduled.enable();
    scheduled.c.operate(Operation::JointStream);
    scheduled.tick();
    const auto session = scheduled.c.snapshot().session;
    const double started = scheduled.now, omega = 3.141592653589793;
    uint64_t sequence = 0;
    Vec last_v = scheduled.output.dq, last_a = Vec::Zero();
    double squared_error = 0;
    unsigned samples = 0;
    for (int i = 0; i < 2500; ++i) {
      if (i % 5 == 0) {
        JointReference point;
        point.time = scheduled.now + 0.04;
        const double phase = omega * (point.time - started);
        point.position = q;
        point.position[0] += 0.03 * std::sin(phase);
        point.velocity[0] = 0.03 * omega * std::cos(phase);
        point.acceleration[0] = -0.03 * omega * omega * std::sin(phase);
        point.has_velocity = point.has_acceleration = true;
        require(scheduled.c.stream(session, ++sequence, point, scheduled.now),
                "future dynamic reference rejected");
      }
      scheduled.tick();
      require(scheduled.c.snapshot().mode == Mode::JointStream,
              "dynamic reference interrupted");
      const Vec acc = (scheduled.output.dq - last_v) / scheduled.config.period;
      require((scheduled.output.dq.cwiseAbs().array() <=
               scheduled.config.velocity.array() + 1e-6)
                  .all(),
              "stream velocity limit");
      require((acc.cwiseAbs().array() <=
               scheduled.config.acceleration.array() + 1e-5)
                  .all(),
              "stream acceleration limit");
      require(((acc - last_a).cwiseAbs().array() / scheduled.config.period <=
               scheduled.config.jerk.array() + 1e-3)
                  .all(),
              "stream jerk limit");
      last_v = scheduled.output.dq;
      last_a = acc;
      if (i > 1000) {
        const double ideal = 0.03 * std::sin(omega * (scheduled.now - started));
        squared_error += std::pow(scheduled.output.q[0] - ideal, 2);
        ++samples;
      }
    }
    require(std::sqrt(squared_error / samples) < 0.002,
            "scheduled reference tracking error");

    // Identical elapsed times must advance streaming references identically,
    // even when the configured scheduler periods differ. This catches a
    // nominal-period clock that drifts after delayed or skipped wakeups.
    Config slower;
    slower.period = 0.004;
    Controller fast_clock(mc, Config{}), slow_clock(mc, slower);
    Feedback fast_feedback, slow_feedback;
    fast_feedback.q = slow_feedback.q = q;
    fast_feedback.fresh = slow_feedback.fresh = true;
    Output fast_output, slow_output;
    double clock_time = 1;
    auto advance_clocks = [&](double dt) {
      clock_time += dt;
      for (auto *feedback : {&fast_feedback, &slow_feedback}) {
        feedback->received_at = clock_time;
        ++feedback->sequence;
      }
      if (fast_output.enabled) {
        fast_feedback.q = fast_output.q;
        fast_feedback.dq = fast_output.dq;
      }
      if (slow_output.enabled) {
        slow_feedback.q = slow_output.q;
        slow_feedback.dq = slow_output.dq;
      }
      fast_output = fast_clock.step(fast_feedback, clock_time);
      slow_output = slow_clock.step(slow_feedback, clock_time);
    };
    advance_clocks(0.002);
    fast_clock.operate(Operation::Enable);
    slow_clock.operate(Operation::Enable);
    advance_clocks(0.002);
    fast_clock.operate(Operation::JointStream);
    slow_clock.operate(Operation::JointStream);
    advance_clocks(0.002);
    const auto fast_session = fast_clock.snapshot().session;
    const auto slow_session = slow_clock.snapshot().session;
    JointReference stationary_target;
    stationary_target.position = q;
    stationary_target.position[0] += 0.03;
    stationary_target.has_velocity = stationary_target.has_acceleration = true;
    const double intervals[] = {0.001, 0.003, 0.006, 0.002};
    Vec previous_jitter_velocity = Vec::Zero();
    Vec previous_jitter_acceleration = Vec::Zero();
    double previous_interval = 0.002;
    for (int i = 0; i < 200; ++i) {
      stationary_target.time = clock_time;
      require(fast_clock.stream(fast_session, i + 1, stationary_target,
                                clock_time) &&
                  slow_clock.stream(slow_session, i + 1, stationary_target,
                                    clock_time),
              "jitter reference rejected");
      advance_clocks(intervals[i % 4]);
      require(fast_clock.snapshot().mode == Mode::JointStream &&
                  slow_clock.snapshot().mode == Mode::JointStream,
              "jittered stream interrupted");
      require((fast_output.q - slow_output.q).norm() < 1e-10 &&
                  (fast_output.dq - slow_output.dq).norm() < 1e-9,
              "streaming reference clock depends on nominal scheduler period");
      const double interval = intervals[i % 4];
      const Vec acceleration =
          (fast_output.dq - previous_jitter_velocity) / interval;
      require((acceleration.cwiseAbs().array() <=
               slower.acceleration.array() + 1e-6)
                  .all(),
              "jittered reference acceleration limit");
      require(
          ((acceleration - previous_jitter_acceleration).cwiseAbs().array() /
               (0.5 * (interval + previous_interval)) <=
           slower.jerk.array() + 1e-4)
              .all(),
          "jittered reference jerk limit");
      previous_jitter_velocity = fast_output.dq;
      previous_jitter_acceleration = acceleration;
      previous_interval = interval;
    }
    // STOP must still take effect when ordinary mode requests fill the mailbox.
    for (int i = 0; i < 16; ++i)
      scheduled.c.operate(Operation::JointStream);
    scheduled.c.operate(Operation::Stop);
    scheduled.tick();
    require(scheduled.c.snapshot().mode == Mode::Stopping ||
                scheduled.c.snapshot().mode == Mode::AwaitResume,
            "stop lost behind queued mode requests");

    Fixture compliant(argv[1]);
    compliant.enable();
    compliant.c.operate(Operation::Impedance);
    compliant.tick();
    const auto impedance_session = compliant.c.snapshot().session;
    CartesianReference equilibrium;
    equilibrium.pose = m.fk(q);
    equilibrium.has_velocity = equilibrium.has_acceleration = true;
    compliant.f.q = q;
    compliant.f.q[0] += 0.01;
    compliant.f.dq.setZero();
    for (int i = 0; i < 100; ++i) {
      equilibrium.time = compliant.now;
      require(compliant.c.stream(impedance_session, i + 1, equilibrium,
                                 compliant.now),
              "impedance reference rejected");
      compliant.tick(false);
      require(compliant.c.snapshot().mode == Mode::Impedance,
              "rigid common joint motion caused a false collision stop");
    }
    const Vec before_gain = compliant.output.tau;
    require(compliant.c.gains(2 * compliant.config.stiffness,
                              compliant.config.damping),
            "online impedance gain update rejected");
    compliant.tick(false);
    const Vec error = poseError(m.fk(compliant.f.q), equilibrium.pose);
    const double alpha =
        1 - std::exp(-compliant.config.period /
                     compliant.config.impedance_gain_time_constant);
    const Vec expected_change = alpha * m.jacobian(compliant.f.q).transpose() *
                                compliant.config.stiffness.cwiseProduct(error);
    require((compliant.output.tau - before_gain - expected_change).norm() <
                1e-8,
            "impedance gain change was not smoothly filtered");
    require(expected_change.norm() > 1e-6,
            "impedance gain test has no torque signal");

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
    std::cout
        << "PASS: model, IK, dynamics, distance diagnostics, finite goals, "
           "dynamic references and limits, online impedance gains, "
           "stop priority, session timeout, feedback failure "
           "and instance isolation\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
