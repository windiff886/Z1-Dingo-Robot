#include "z1_control/controller.hpp"
#include "z1_control/sample_queue.hpp"
#include <atomic>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace z1;
namespace {
void require(bool okay, const char *message) {
  if (!okay) throw std::runtime_error(message);
}
void modelSelection(const char *path) {
  auto with_config = defaultModelConfig(path);
  auto without_config = with_config;
  without_config.has_gripper = false;
  Model with(with_config), without(without_config);
  Vec q; q << .1, .8, -1.1, .3, .2, -.1;
  const auto zero = Vec::Zero();
  const Vec a = with.inverseDynamics(q, zero, zero);
  const Vec b = without.inverseDynamics(q, zero, zero);
  require((a - b).norm() > .5, "removed gripper still contributes gravity");
  Vec acceleration = Vec::Zero(); acceleration[5] = 1;
  require(with.inverseDynamics(q, zero, acceleration)[5] - a[5] >
              without.inverseDynamics(q, zero, acceleration)[5] - b[5],
          "removed gripper still contributes wrist inertia");
  require((with.fk(q).matrix() - without.fk(q).matrix()).norm() < 1e-12,
          "gripper selection moved the TCP");
  require(with.lower() == without.lower() && with.upper() == without.upper(),
          "gripper selection changed arm limits");
  require(with.revision() != without.revision(), "model revisions alias different payloads");
  const auto xml = effectiveModelUrdf(without_config);
  require(xml.find("gripperStator") == std::string::npos &&
              xml.find("gripperMover") == std::string::npos,
          "gripper geometry or descendants were retained");
}
void queueContract() {
  SampleQueue<3> queue;
  RuntimeSample sample, received;
  sample.feedback.sequence = 1; require(queue.push(sample), "first sample lost");
  sample.feedback.sequence = 2; require(queue.push(sample), "second sample lost");
  require(!queue.push(sample) && queue.dropped() == 1, "queue silently overwrote samples");
  require(queue.pop(received) && received.feedback.sequence == 1, "queue lost FIFO order");
  require(queue.pop(received) && received.feedback.sequence == 2, "queue lost second sample");
  require(!queue.pop(received), "empty queue returned a sample");
}
struct Following final : Transport {
  uint64_t sequence{};
  Vec q{(Vec() << 0, .8, -1.1, .3, 0, 0).finished()};
  Exchange exchange(const Output &command, double) override {
    if (command.enabled) q = command.q;
    Feedback f;
    f.q = q; f.dq = command.dq; f.tau = command.tau;
    f.received_at = monotonicTime(); f.sequence = ++sequence;
    f.fresh = sequence % 3 != 0;
    return {f, true, {}};
  }
};
void runtimeRecording(const char *path) {
  std::vector<RuntimeSample> recorded;
  recorded.reserve(2048);
  RuntimeOptions options;
  options.sample_callback = [&](const RuntimeSample &sample) { recorded.push_back(sample); };
  Runtime runtime(defaultModelConfig(path), Config{}, std::make_unique<Following>(), options);
  runtime.start();
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!runtime.controller().snapshot().feedback_valid) {
    require(std::chrono::steady_clock::now() < deadline, "runtime did not get feedback");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  runtime.controller().operate(Operation::Enable);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  runtime.stop();
  require(recorded.size() > 10, "native recorder did not capture control cycles");
  bool enabled = false;
  uint64_t sequence = 0;
  for (const auto &sample : recorded) {
    require(sample.feedback.fresh && sample.feedback.sequence > sequence &&
                sample.feedback.sequence % 3 != 0, "retained feedback manufactured samples");
    require(sample.feedback.received_at >= sample.sent_at, "receive time precedes send");
    if (sample.command.enabled) {
      enabled = true;
      require((sample.command.q - sample.feedback.q).norm() < 1e-12 &&
                  (sample.command.tau - sample.feedback.tau).norm() < 1e-12,
              "recorder captured next-cycle command rather than sent output");
    }
    sequence = sample.feedback.sequence;
  }
  require(enabled, "recorder never exercised enabled commands");
}
} // namespace
int main(int argc, char **argv) {
  try {
    require(argc == 2, "model path required");
    modelSelection(argv[1]); queueContract(); runtimeRecording(argv[1]);
    std::cout << "PASS: hardware payload models, bounded sample queue and sent-command timestamps\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
