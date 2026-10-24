// Offline allocation probe: counts ordinary C++ new/new[], not direct malloc.
// Stable JointStream case uses exact synthetic reference-following feedback.
#include "z1_control/controller.hpp"
#include <cstdio>
#include <cstdlib>
#include <new>
static thread_local bool measure = false;
static thread_local unsigned allocations = 0;
void *operator new(std::size_t n) {
  if (measure)
    ++allocations;
  if (auto p = std::malloc(n))
    return p;
  throw std::bad_alloc();
}
void *operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void *p) noexcept { std::free(p); }
void operator delete[](void *p) noexcept { std::free(p); }
void operator delete(void *p, std::size_t) noexcept { std::free(p); }
void operator delete[](void *p, std::size_t) noexcept { std::free(p); }
int main(int argc, char **argv) {
  if (argc != 2)
    return 2;
  using namespace z1;
  auto mc = defaultModelConfig(argv[1]);
  Model m(mc);
  Vec q;
  q << 0, .8, -1.1, .3, 0, 0;
  auto initial = m.clearance(q);
  allocations = 0;
  measure = true;
  auto cl = m.clearance(q);
  measure = false;
  std::printf("clearance_same_q_new_calls=%u,pair=%s\n", allocations,
              cl.pair.c_str());
  allocations = 0;
  measure = true;
  auto view = m.clearanceSample(q);
  measure = false;
  std::printf("clearance_sample_same_q_new_calls=%u\n", allocations);
  allocations = 0;
  measure = true;
  auto fk = m.fk(q);
  auto J = m.jacobian(q);
  auto tau = m.inverseDynamics(q, Vec::Zero(), Vec::Zero());
  measure = false;
  std::printf("fk_jacobian_rnea_new_calls=%u\n", allocations);
  Config cfg;
  Controller c(mc, cfg);
  Feedback f;
  f.q = q;
  f.fresh = true;
  double now = 1;
  unsigned seq = 0;
  Output out;
  auto tick = [&]() {
    now += cfg.period;
    f.sequence++;
    f.received_at = now;
    if (out.enabled) {
      f.q = out.q;
      f.dq = out.dq;
    }
    out = c.step(f, now);
  };
  tick();
  c.operate(Operation::Enable);
  tick();
  c.operate(Operation::JointStream);
  tick();
  auto session = c.snapshot().session;
  Vec target = q;
  target[0] += .03;
  unsigned total = 0, minimum = 99999, maximum = 0;
  for (int i = 0; i < 1200; i++) {
    c.stream(session, ++seq, target, now);
    tick();
    allocations = 0;
    measure = true;
    tick();
    measure = false;
    if (i >= 1000) {
      total += allocations;
      minimum = std::min(minimum, allocations);
      maximum = std::max(maximum, allocations);
    }
  }
  std::printf("joint_stream_step_new_calls,min=%u,mean=%.2f,max=%u,mode=%u\n",
              minimum, total / 200., maximum, unsigned(c.snapshot().mode));
  total = 0;
  minimum = 99999;
  maximum = 0;
  const double started = now, omega = 3.141592653589793;
  for (int i = 0; i < 2000; ++i) {
    if (i % 10 == 0) {
      JointReference point;
      point.time = now + 0.04;
      point.position = q;
      const double phase = omega * (point.time - started);
      point.position[0] += 0.03 * std::sin(phase);
      point.velocity[0] = 0.03 * omega * std::cos(phase);
      point.acceleration[0] = -0.03 * omega * omega * std::sin(phase);
      point.has_velocity = point.has_acceleration = true;
      if (!c.stream(session, ++seq, point, now))
        return 1;
    }
    allocations = 0;
    measure = true;
    tick();
    measure = false;
    if (i >= 1000) {
      total += allocations;
      minimum = std::min(minimum, allocations);
      maximum = std::max(maximum, allocations);
    }
  }
  std::printf("dynamic_joint_step_new_calls,min=%u,mean=%.2f,max=%u,mode=%u\n",
              minimum, total / 1000., maximum, unsigned(c.snapshot().mode));
  return c.snapshot().mode == Mode::JointStream ? 0 : 1;
}
