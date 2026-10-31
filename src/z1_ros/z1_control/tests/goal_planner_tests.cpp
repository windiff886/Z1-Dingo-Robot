#include "z1_control/controller.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace z1;
void require(bool condition, const char *reason) {
  if (!condition)
    throw std::runtime_error(reason);
}
int main(int argc, char **argv) {
  try {
    require(argc == 2, "model path required");
    const auto mc = defaultModelConfig(argv[1]);
    Model model(mc);
    Config cfg;
    cfg.period = .004;
    Vec start, end, home;
    start << 0, .8, -1.1, .3, 0, 0;
    end = start;
    end[0] += .06;
    home << 0, 0, 0, -.07435, 0, 0;
    const auto home_pose = model.fk(home);
    const Eigen::Quaterniond home_rotation(home_pose.linear());
    std::cout << "Home xyz=" << home_pose.translation().transpose()
              << " quaternion_xyzw=" << home_rotation.coeffs().transpose() << '\n';
    for (const Pose &target : {model.fk(end), home_pose}) {
      const auto preview = planPoseGoal(model, cfg, start, target, .3);
      Controller controller(mc, cfg);
      Feedback feedback;
      feedback.q = start;
      feedback.fresh = true;
      feedback.sequence = 1;
      feedback.received_at = 1;
      auto out = controller.step(feedback, 1);
      controller.operate(Operation::Enable);
      feedback.received_at += cfg.period;
      ++feedback.sequence;
      controller.step(feedback, feedback.received_at);
      require(controller.poseGoal(1, target, .3), "controller rejected previewed pose");
      require(std::abs(controller.snapshot().trajectory_duration - preview.get_duration()) < 1e-12,
              "preview and execution durations differ");
      const double begin = feedback.received_at;
      for (int i = 1; i <= std::ceil(preview.get_duration() / cfg.period) + 2; ++i) {
        feedback.received_at = begin + i * cfg.period;
        ++feedback.sequence;
        feedback.q = i == 1 ? start : out.q;
        feedback.dq = i == 1 ? Vec::Zero() : out.dq;
        feedback.tau = out.tau;
        out = controller.step(feedback, feedback.received_at);
        std::array<double, 6> q, v, a;
        preview.at_time(feedback.received_at - begin, q, v, a);
        require(out.enabled && controller.snapshot().mode != Mode::Fault,
                "previewed motion faulted during reference following");
        require((out.q - Eigen::Map<Vec>(q.data())).norm() < 1e-10 &&
                    (out.dq - Eigen::Map<Vec>(v.data())).norm() < 1e-10,
                "preview differs from SDK execution reference");
      }
      const auto error = poseError(model.fk(out.q), target);
      require(error.head<3>().norm() < 1e-4 && error.tail<3>().norm() < 5e-4,
              "Home/pose preview endpoint does not reach requested TCP pose");
    }
    bool rejected = false;
    Pose far = Pose::Identity();
    far.translation().x() = 100;
    try { planPoseGoal(model, cfg, start, far); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "unreachable preview target accepted");
    rejected = false;
    try { planJointGoal(model, cfg, start, end, 0); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "invalid preview speed accepted");
    std::cout << "PASS: shared planning, Home pose and execution reference equivalence\n";
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
