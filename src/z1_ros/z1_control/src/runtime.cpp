#include "z1_control/runtime.hpp"
#include <chrono>
#include <stdexcept>
namespace z1 {
double monotonicTime() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
struct Simulation::Impl {
  Model model;
  Vec q{(Vec() << 0, 0.8, -1.1, 0.3, 0, 0).finished()}, dq{Vec::Zero()};
  double dt;
  uint64_t seq{};
  Impl(ModelConfig m, double p) : model(m), dt(p) {}
};
Simulation::Simulation(ModelConfig m, double p) : impl_(new Impl(m, p)) {}
Simulation::~Simulation() = default;
Exchange Simulation::exchange(const Output &o, double now) {
  auto &p = *impl_;
  Vec tau = o.enabled ? o.tau : Vec::Zero();
  if (o.enabled) {
    // Eight substeps improve stability; no contact dynamics or firmware
    // emulation.
    for (int i = 0; i < 8; ++i) {
      Vec a = p.model.forwardDynamics(p.q, p.dq, tau - 0.002 * p.dq);
      p.dq += a * (p.dt / 8);
      p.q += p.dq * (p.dt / 8);
    }
  } else
    p.dq.setZero(); // Simulator has an ideal passive brake, not a Z1 hardware
                    // guarantee.
  Feedback f;
  f.q = p.q;
  f.dq = p.dq;
  f.tau = tau;
  f.received_at = now;
  f.sequence = ++p.seq;
  f.fresh = true;
  return {f, true, {}};
}
Runtime::Runtime(ModelConfig m, Config c, std::unique_ptr<Transport> t)
    : controller_(m, c), config_(c), transport_(std::move(t)) {
  if (!transport_)
    throw std::invalid_argument("missing transport");
}
Runtime::~Runtime() { stop(); }
Runtime::Runtime(ModelConfig m, Config c,
                 std::function<std::unique_ptr<Transport>()> factory)
    : controller_(m, c), config_(c), transport_(factory()) {
  if (!transport_)
    throw std::invalid_argument("missing transport");
}
void Runtime::start() {
  if (running_.exchange(true))
    return;
  thread_ = std::thread([this] {
    using Clock = std::chrono::steady_clock;
    auto next = Clock::now();
    Output out;
    while (running_) {
      double now = monotonicTime();
      Exchange e;
      try {
        e = transport_->exchange(out, now);
      } catch (const std::exception &) {
        e.feedback.fresh = false;
        e.feedback.transport_fault = true;
      }
      // Receive time belongs to the transport; evaluate validity after the
      // bounded exchange.
      if (!e.sent)
        e.feedback.transport_fault = true;
      out = controller_.step(e.feedback, monotonicTime());
      if (!e.sent)
        out = Output{};
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(config_.period));
      std::this_thread::sleep_until(next);
      if (Clock::now() - next >
          std::chrono::duration<double>(config_.feedback_timeout))
        next = Clock::now();
    }
    // Normal process shutdown attempts a bounded controlled stop while feedback
    // is valid.
    auto mode = controller_.snapshot().mode;
    if (mode != Mode::Disabled && mode != Mode::Fault &&
        mode != Mode::AwaitResume) {
      controller_.operate(Operation::Stop);
      auto deadline = Clock::now() + std::chrono::seconds(2);
      while (Clock::now() < deadline) {
        try {
          auto exchange = transport_->exchange(out, monotonicTime());
          if (!exchange.sent)
            exchange.feedback.transport_fault = true;
          out = controller_.step(exchange.feedback, monotonicTime());
        } catch (const std::exception &) {
          out = Output{};
          break;
        }
        auto stopped = controller_.snapshot().mode;
        if (stopped == Mode::AwaitResume || stopped == Mode::Disabled ||
            stopped == Mode::Fault)
          break;
        std::this_thread::sleep_for(
            std::chrono::duration<double>(config_.period));
      }
    }
    // Attempt a final passive packet; this is not a firmware stop
    // acknowledgment.
    try {
      transport_->exchange(Output{}, monotonicTime());
    } catch (const std::exception &) {
    }
  });
}
void Runtime::stop() {
  running_ = false;
  if (thread_.joinable())
    thread_.join();
}
} // namespace z1
