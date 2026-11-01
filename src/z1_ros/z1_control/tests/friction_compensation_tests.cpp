#include "z1_control/controller.hpp"
#include "z1_control/friction.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
using namespace z1;
namespace {
void require(bool value, const char *reason) {
  if (!value)
    throw std::runtime_error(reason);
}
Feedback feedback(const Vec &q, double now, uint64_t sequence) {
  Feedback f;
  f.q = q;
  f.received_at = now;
  f.sequence = sequence;
  f.fresh = true;
  return f;
}
std::vector<TrajectoryPoint> path(const Vec &start) {
  const double speed = .04, distance = .2, ramp = std::sqrt(6 * speed / 2),
               cruise = distance / speed - ramp;
  std::vector<TrajectoryPoint> result;
  Vec from = start;
  double elapsed = 0;
  for (int direction : {1, -1}) {
    for (int k = 0; k < 4; ++k) {
      if (direction < 0 && k == 0)
        continue;
      const double t[] = {0, ramp, ramp + cruise, 2 * ramp + cruise};
      const double offset[] = {0, .5 * speed * ramp,
                               distance - .5 * speed * ramp, distance};
      TrajectoryPoint p;
      p.time_from_start = elapsed + t[k];
      p.position = from;
      p.position[5] += direction * offset[k];
      p.velocity[5] = (k == 1 || k == 2) ? direction * speed : 0;
      p.has_velocity = p.has_acceleration = true;
      result.push_back(p);
    }
    from = result.back().position;
    elapsed = result.back().time_from_start;
  }
  return result;
}
void contracts(const std::string &urdf, const std::string &yaml) {
  auto left = loadFrictionConfig(yaml, "left"),
       right = loadFrictionConfig(yaml, "right");
  require(left.coulomb.isZero(0) && left.viscous.isZero(0) &&
              right.coulomb.isZero(0) && right.viscous.isZero(0),
          "shipped coefficients must be zero");
  FrictionConfig c;
  c.coulomb.setConstant(.3);
  c.viscous.setConstant(.8);
  require(frictionFeedforward(c, Vec::Zero()).isZero(0),
          "no breakaway kick at rest");
  const Vec v = Vec::Constant(.04);
  require((frictionFeedforward(c, v) + frictionFeedforward(c, -v)).norm() <
              1e-12,
          "friction must be odd");
  require((frictionFeedforward(c, v).array() > 0).all(), "friction sign");
  c.coulomb[0] = -.1;
  try {
    validateFrictionConfig(c);
    throw std::runtime_error("negative friction accepted");
  } catch (const std::invalid_argument &) {
  }
  auto mc = defaultModelConfig(urdf);
  Config cfg;
  cfg.period = .004;
  Vec q;
  q << 0, .8, -1.2, .4, 0, -.1;
  Controller zero(mc, cfg);
  cfg.friction.enabled = false;
  Controller disabled(mc, cfg);
  auto f = feedback(q, 1, 1);
  zero.step(f, 1);
  disabled.step(f, 1);
  zero.operate(Operation::Enable);
  disabled.operate(Operation::Enable);
  f = feedback(q, 1.004, 2);
  require((zero.step(f, 1.004).tau - disabled.step(f, 1.004).tau).isZero(0),
          "zero profile changed old output");
  require(zero.frictionCompensation(false) && zero.frictionCompensation(true),
          "holding toggle rejected");
  require(zero.trajectory(1, path(q)), "trajectory rejected");
  require(!zero.frictionCompensation(false), "switch accepted while moving");
  cfg.friction.enabled = false;
  cfg.friction.coulomb[5] = .3;
  cfg.friction.has_gripper = 0;
  Controller stale(mc, cfg);
  require(!stale.frictionCompensation(true),
          "stale attachment enabled after disabled startup");
  cfg.friction.enabled = true;
  try {
    Controller mismatch(mc, cfg);
    throw std::runtime_error("attachment mismatch accepted");
  } catch (const std::invalid_argument &) {
  }
  // Saturation applies to the combined torque; disabled output is still
  // passive.
  cfg.friction.has_gripper = -1;
  cfg.friction.coulomb.setConstant(100);
  cfg.torque.setConstant(.1);
  cfg.torque_rate.setConstant(.2);
  Controller bounded(mc, cfg);
  f = feedback(q, 1, 1);
  bounded.step(f, 1);
  bounded.trajectory(1, path(q));
  for (int i = 1; i < 15; ++i) {
    f = feedback(q, 1 + i * .004, i + 1);
    const auto out = bounded.step(f, f.received_at);
    require(out.enabled && out.tau.cwiseAbs().maxCoeff() <= .1 + 1e-12,
            "combined amplitude limit bypassed");
    require(out.tau.cwiseAbs().maxCoeff() <= i * .004 * .2 + 1e-12,
            "combined rate limit bypassed");
  }
  bounded.operate(Operation::Disable);
  f = feedback(q, 1.064, 17);
  const auto passive = bounded.step(f, f.received_at);
  require(!passive.enabled && passive.tau.isZero(0) &&
              passive.friction_feedforward.isZero(0),
          "disabled was not passive");
}
double tracking(const std::string &urdf, bool gripper, bool impedance,
                bool compensated, std::ofstream &csv) {
  auto mc = defaultModelConfig(urdf);
  mc.has_gripper = gripper;
  Model model(mc);
  Config cfg;
  cfg.period = .004;
  cfg.friction.enabled = compensated;
  cfg.friction.coulomb[5] = .35;
  cfg.friction.viscous[5] = .08;
  // This isolated low-inertia wrist fixture needs modest sampled impedance
  // gains; the same values are used in its off/on cases.
  cfg.stiffness.setConstant(5);
  cfg.damping.setConstant(.1);
  Controller controller(mc, cfg);
  Vec q;
  q << 0, .8, -1.2, .4, 0, -.1;
  Vec dq = Vec::Zero();
  auto points = path(q);
  auto oracle =
      JointTrajectory::timed(points, cfg, model.lower(), model.upper());
  auto f = feedback(q, 1, 1);
  controller.step(f, 1);
  uint64_t sequence = 1, stream_sequence = 0;
  if (impedance) {
    controller.operate(Operation::Enable);
    f = feedback(q, 1.004, ++sequence);
    controller.step(f, f.received_at);
    controller.operate(Operation::Impedance);
    f = feedback(q, 1.008, ++sequence);
    controller.step(f, f.received_at);
  } else
    require(controller.trajectory(1, points), "fixture trajectory rejected");
  const double began = f.received_at;
  double error2 = 0;
  int count = 0;
  Vec previous_friction = Vec::Zero();
  for (int step = 1; step <= std::ceil(oracle.duration() / cfg.period);
       ++step) {
    const double t = step * cfg.period, now = began + t;
    const auto reference = oracle.sample(t);
    if (impedance) {
      CartesianReference target;
      target.time = now;
      target.pose = model.fk(reference.position);
      target.has_velocity = target.has_acceleration = true;
      target.velocity = model.jacobian(reference.position) * reference.velocity;
      require(controller.stream(controller.snapshot().session,
                                ++stream_sequence, target, now),
              "impedance reference rejected");
    }
    f = feedback(q, now, ++sequence);
    f.dq = dq;
    const auto out = controller.step(f, now);
    if (!out.enabled || controller.snapshot().mode == Mode::Fault)
      throw std::runtime_error(
          "fixture faulted: " + controller.snapshot().reason +
          "; t=" + std::to_string(t) + " q6=" + std::to_string(q[5]) +
          " dq6=" + std::to_string(dq[5]));
    if (impedance)
      require(out.kp.isZero(0) && out.kd.isZero(0),
              "impedance got MCU position gains");
    if (impedance)
      require((out.friction_feedforward - previous_friction)
                      .cwiseAbs()
                      .maxCoeff() <=
                  cfg.friction.impedance_torque_rate * cfg.period + 1e-12,
              "impedance friction slew limit bypassed");
    previous_friction = out.friction_feedforward;
    const double error = q[5] - reference.position[5];
    error2 += error * error;
    ++count;
    if (csv)
      csv << gripper << ',' << impedance << ',' << compensated << ',' << t
          << ',' << reference.position[5] << ',' << q[5] << ',' << dq[5]
          << '\n';
    // Isolated J6 fixture: other five axes mechanically locked. Backward Euler
    // handles motor damping; a dry-friction impulse clamps zero-velocity
    // stiction. This is a regression plant, not a full-arm or hardware
    // performance result.
    for (int sub = 0; sub < 40; ++sub) {
      const double h = cfg.period / 40;
      const Vec bias = model.inverseDynamics(q, dq, Vec::Zero());
      Vec unit = Vec::Zero();
      unit[5] = 1;
      const double inertia = model.inverseDynamics(q, dq, unit)[5] - bias[5];
      const double drive = out.tau[5] + out.kp[5] * (out.q[5] - q[5]) +
                           out.kd[5] * out.dq[5] - bias[5];
      const double impulse = inertia * dq[5] + h * drive;
      dq[5] =
          std::copysign(std::max(0., std::abs(impulse) - h * .35), impulse) /
          (inertia + h * (out.kd[5] + .08));
      q[5] += h * dq[5];
    }
  }
  return std::sqrt(error2 / count);
}
} // namespace
int main(int argc, char **argv) {
  try {
    require(argc >= 3, "URDF and zero friction YAML required");
    contracts(argv[1], argv[2]);
    std::ofstream csv;
    if (argc > 3) {
      csv.open(argv[3]);
      csv << "has_gripper,impedance,compensated,time,reference,position,"
             "velocity\n";
    }
    for (bool gripper : {true, false})
      for (bool impedance : {false, true}) {
        const double off = tracking(argv[1], gripper, impedance, false, csv);
        const double on = tracking(argv[1], gripper, impedance, true, csv);
        std::cout << "isolated J6 fixture gripper=" << gripper
                  << " impedance=" << impedance << " RMS " << off << " -> "
                  << on << " rad\n";
        require(
            on < off * .7,
            "friction fixture did not improve at identical gains/reference");
      }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
