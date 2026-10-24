// Offline audit only: uses synthetic feedback and Pinocchio simulation.
// GNU ld wrapping measures Model::clearance without changing production code.
#include "z1_control/controller.hpp"
#include "z1_control/transport.hpp"
#include <Eigen/Cholesky>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>
using namespace z1;
using Clock = std::chrono::steady_clock;
static double micros(Clock::time_point a) {
  return std::chrono::duration<double, std::micro>(Clock::now() - a).count();
}
static volatile double sink = 0;
static uint64_t collision_calls = 0;
static double collision_us = 0;
extern "C" Clearance real_clearance(Model *, const Vec &) asm(
    "__real__ZN2z15Model9clearanceERKN5Eigen6MatrixIdLi6ELi1ELi0ELi6ELi1EEE");
extern "C" Clearance wrap_clearance(Model *, const Vec &) asm(
    "__wrap__ZN2z15Model9clearanceERKN5Eigen6MatrixIdLi6ELi1ELi0ELi6ELi1EEE");
extern "C" Clearance real_bound(Model *, const Vec &, const Vec &) asm(
    "__real__"
    "ZN2z15Model9clearanceERKN5Eigen6MatrixIdLi6ELi1ELi0ELi6ELi1EEES5_");
extern "C" Clearance wrap_bound(Model *, const Vec &, const Vec &) asm(
    "__wrap__"
    "ZN2z15Model9clearanceERKN5Eigen6MatrixIdLi6ELi1ELi0ELi6ELi1EEES5_");
extern "C" ClearanceSample real_sample(Model *, const Vec &, const Vec &) asm(
    "__real__"
    "ZN2z15Model15clearanceSampleERKN5Eigen6MatrixIdLi6ELi1ELi0ELi6ELi1EEES5_");
extern "C" ClearanceSample wrap_sample(Model *, const Vec &, const Vec &) asm(
    "__wrap__"
    "ZN2z15Model15clearanceSampleERKN5Eigen6MatrixIdLi6ELi1ELi0ELi6ELi1EEES5_");
extern "C" Clearance wrap_clearance(Model *m, const Vec &q) {
  auto a = Clock::now();
  auto r = real_clearance(m, q);
  collision_us += micros(a);
  ++collision_calls;
  return r;
}
extern "C" Clearance wrap_bound(Model *m, const Vec &q, const Vec &bound) {
  auto a = Clock::now();
  auto r = real_bound(m, q, bound);
  collision_us += micros(a);
  ++collision_calls;
  return r;
}
extern "C" ClearanceSample wrap_sample(Model *m, const Vec &q,
                                       const Vec &bound) {
  auto a = Clock::now();
  auto r = real_sample(m, q, bound);
  collision_us += micros(a);
  ++collision_calls;
  return r;
}
template <class F> void bench(const char *name, F f, int n = 3000) {
  for (int i = 0; i < 100; ++i)
    f(i);
  std::vector<double> t;
  t.reserve(n);
  for (int i = 0; i < n; ++i) {
    auto a = Clock::now();
    f(i);
    t.push_back(micros(a));
  }
  std::sort(t.begin(), t.end());
  double total = 0;
  for (double x : t)
    total += x;
  std::cout << "micro," << name << ",median_us=" << t[n / 2]
            << ",p99_us=" << t[size_t(n * .99)] << ",mean_us=" << total / n
            << "\n";
}
void trace(const char *urdf, double dt, double hz, bool direct,
           bool pose_mode = false, bool derivatives = false,
           bool scheduled = false) {
  auto mc = defaultModelConfig(urdf);
  Config cfg;
  cfg.period = dt;
  Controller c(mc, cfg);
  Simulation sim(mc, dt);
  Model model(mc);
  double now = 1;
  Output out;
  Feedback f;
  for (int i = 0; i < int(4 / dt); ++i) {
    f = sim.exchange(out, now).feedback;
    if (i == 0)
      c.operate(Operation::Enable);
    out = c.step(f, now);
    now += dt;
  }
  f = sim.exchange(out, now).feedback;
  out = c.step(f, now);
  now += dt;
  if (c.snapshot().mode != Mode::Holding) {
    std::cout << "trace_failed,warmup," << c.snapshot().reason << "\n";
    return;
  }
  Vec base = f.q;
  double amplitude = 0.03, w = 2 * M_PI * hz;
  c.operate(pose_mode ? Operation::PoseStream : Operation::JointStream);
  f = sim.exchange(out, now).feedback;
  out = c.step(f, now);
  now += dt;
  uint64_t session = c.snapshot().session, seq = 0;
  int every = std::lround(0.02 / dt);
  Eigen::Matrix3d gram = Eigen::Matrix3d::Zero();
  Eigen::Vector3d fit = Eigen::Vector3d::Zero();
  double err = 0, ref_err = 0, ref_actual_err = 0, tcp_pos_err = 0,
         tcp_rot_err = 0;
  int count = 0;
  bool ended = false;
  std::string reason;
  double started = now;
  uint64_t nc = collision_calls;
  double cu = collision_us;
  std::vector<double> computes;
  const int n = int(12 / dt);
  for (int i = 0; i < n; ++i) {
    double t = now - started;
    f = sim.exchange(out, now).feedback;
    Vec target = base;
    target[0] += amplitude * std::sin(w * t);
    if (direct) {
      Vec vr = Vec::Zero(), ar = Vec::Zero();
      vr[0] = amplitude * w * std::cos(w * t);
      ar[0] = -amplitude * w * w * std::sin(w * t);
      Vec acc = ar + cfg.kp.cwiseProduct(target - f.q) +
                cfg.kd.cwiseProduct(vr - f.dq);
      Vec torque = model.inverseDynamics(f.q, f.dq, acc)
                       .cwiseMax(-cfg.torque)
                       .cwiseMin(cfg.torque);
      Vec delta = cfg.torque_rate * dt;
      out.tau = torque.cwiseMax(out.tau - delta).cwiseMin(out.tau + delta);
      out.q = target;
      out.dq = vr;
      out.enabled = true;
    } else {
      if (i % every == 0) {
        if (derivatives) {
          const double offset = scheduled ? 0.04 : 0;
          const double phase = w * (t + offset);
          JointReference reference;
          reference.time = now + offset;
          reference.position = base;
          reference.position[0] += amplitude * std::sin(phase);
          reference.velocity[0] = amplitude * w * std::cos(phase);
          reference.acceleration[0] = -amplitude * w * w * std::sin(phase);
          reference.has_velocity = reference.has_acceleration = true;
          c.stream(session, ++seq, reference, now);
        } else if (pose_mode)
          c.stream(session, ++seq, model.fk(target), now);
        else
          c.stream(session, ++seq, target, now);
      }
      out = c.step(f, now);
      auto s = c.snapshot();
      computes.push_back(s.compute_time * 1e6);
      if (s.mode != (pose_mode ? Mode::PoseStream : Mode::JointStream)) {
        std::cerr << "trace exit t=" << t << " actual=" << f.q.transpose()
                  << " reference=" << out.q.transpose()
                  << " reason=" << s.reason << '\n';
        ended = true;
        reason = s.reason;
        break;
      }
    }
    if (t >= 4) {
      double x = f.q[0] - base[0];
      Eigen::Vector3d b(std::sin(w * t), std::cos(w * t), 1);
      gram += b * b.transpose();
      fit += b * x;
      err += std::pow(target[0] - f.q[0], 2);
      ref_err += std::pow(target[0] - out.q[0], 2);
      ref_actual_err += std::pow(out.q[0] - f.q[0], 2);
      Vec tcp_error = poseError(model.fk(f.q), model.fk(target));
      tcp_pos_err += tcp_error.head<3>().squaredNorm();
      tcp_rot_err += tcp_error.tail<3>().squaredNorm();
      ++count;
    }
    now += dt;
  }
  if (count == 0) {
    std::cout << "trace_failed,no_samples,reason=" << reason << "\n";
    return;
  }
  auto coef = gram.ldlt().solve(fit).eval();
  double lag = -std::atan2(coef[1], coef[0]) / w;
  double compute_total = 0;
  for (double x : computes)
    compute_total += x;
  std::sort(computes.begin(), computes.end());
  std::cout << "trace,"
            << (direct        ? "analytic_q_dq_ddq"
                : derivatives ? (scheduled ? "scheduled_joint_reference"
                                           : "dynamic_joint_reference")
                              : (pose_mode ? "current_pose_stream"
                                           : "current_position_stream"))
            << ",simulation_rate_hz=" << 1 / dt << ",signal_hz=" << hz
            << ",rms_mrad=" << (count ? 1000 * std::sqrt(err / count) : 0)
            << ",target_ref_rms_mrad="
            << (count ? 1000 * std::sqrt(ref_err / count) : 0)
            << ",ref_actual_rms_mrad="
            << (count ? 1000 * std::sqrt(ref_actual_err / count) : 0)
            << ",tcp_position_rms_mm="
            << (count ? 1000 * std::sqrt(tcp_pos_err / count) : 0)
            << ",tcp_rotation_rms_mrad="
            << (count ? 1000 * std::sqrt(tcp_rot_err / count) : 0)
            << ",lag_ms=" << lag * 1000
            << ",amplitude_ratio=" << coef.head<2>().norm() / amplitude
            << ",stopped=" << ended << ",reason=" << reason;
  if (!computes.empty())
    std::cout << ",step_median_us=" << computes[computes.size() / 2]
              << ",step_p99_us=" << computes[size_t(computes.size() * .99)]
              << ",clearance_calls_per_step="
              << double(collision_calls - nc) / computes.size()
              << ",collision_fraction=" << (collision_us - cu) / compute_total;
  std::cout << "\n";
}
int main(int argc, char **argv) {
  if (argc != 2)
    return 2;
  std::cout << std::fixed << std::setprecision(5);
  auto mc = defaultModelConfig(argv[1]);
  Model m(mc);
  Vec q;
  q << 0, .8, -1.1, .3, 0, 0;
  Vec zero = Vec::Zero();
  Pose target = m.fk(q);
  target.translation().z() += .005;
  std::cout << "nominal_clearance=" << m.clearance(q).distance << "\n";
  bench("fk", [&](int i) {
    q[0] = .05 * std::sin(i * .001);
    sink = m.fk(q).translation().norm();
  });
  bench("jacobian", [&](int i) {
    q[0] = .05 * std::sin(i * .001);
    sink = m.jacobian(q).norm();
  });
  bench("rnea", [&](int i) {
    q[0] = .05 * std::sin(i * .001);
    sink = m.inverseDynamics(q, zero, zero).norm();
  });
  bench("differential_ik", [&](int i) {
    q[0] = .05 * std::sin(i * .001);
    sink = m.differentialIk(q, target, 1).norm();
  });
  bench("clearance", [&](int i) {
    q[0] = .05 * std::sin(i * .001);
    sink = m.clearance(q).distance;
  });
  for (double hz : {.25, .5})
    for (double dt : {.004, .002, .001})
      trace(argv[1], dt, hz, false);
  for (double hz : {.25, .5})
    trace(argv[1], .004, hz, true);
  for (double hz : {.25, .5})
    trace(argv[1], .002, hz, false, true);
  for (bool scheduled : {false, true})
    for (double hz : {.25, .5})
      trace(argv[1], .002, hz, false, false, true, scheduled);
  {
    Controller c(mc, Config{});
    Feedback f;
    f.q << 0, .8, -1.1, .3, 0, 0;
    f.fresh = true;
    f.sequence = 1;
    f.received_at = 1;
    c.operate(Operation::Enable);
    c.step(f, 1);
    Pose equilibrium = m.fk(f.q);
    c.operate(Operation::Impedance);
    f.sequence++;
    f.received_at = 1.004;
    c.step(f, 1.004);
    c.stream(c.snapshot().session, 1, equilibrium, 1.004);
    f.sequence++;
    f.received_at = 1.008;
    c.step(f, 1.008);
    double before = m.clearance(f.q).distance;
    f.q[0] += .01;
    f.sequence++;
    f.received_at = 1.012;
    c.step(f, 1.012);
    Vec error = poseError(equilibrium, m.fk(f.q));
    auto state = c.snapshot();
    std::cout << "compliance,feedback_offset_rad=.01,before_clearance="
              << before << ",after_clearance=" << m.clearance(f.q).distance
              << ",tcp_displacement_mm=" << error.head<3>().norm() * 1000
              << ",mode=" << int(state.mode) << ",reason=" << state.reason
              << "\n";
  }
}
