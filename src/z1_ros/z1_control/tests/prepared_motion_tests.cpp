#include "z1_control/controller.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace z1;
namespace {
void require(bool value, const char *message) {
  if (!value)
    throw std::runtime_error(message);
}
struct Fixture {
  Config config;
  ModelConfig model;
  Controller controller;
  Feedback feedback;
  Output output;
  double now{1};
  Fixture(const ModelConfig &m) : model(m), controller(m, config) {
    feedback.q << 0, .8, -1.1, .3, 0, 0;
    tick();
    controller.operate(Operation::Enable);
    tick();
    require(controller.snapshot().mode == Mode::Holding, "enable failed");
  }
  void tick(double elapsed = .002) {
    now += elapsed;
    if (output.enabled) {
      feedback.q = output.q;
      feedback.dq = output.dq;
      feedback.tau = output.tau;
    }
    feedback.received_at = now;
    feedback.sequence++;
    feedback.fresh = true;
    output = controller.step(feedback, now);
    require(controller.snapshot().mode != Mode::Fault, "prepared motion fault");
  }
  void finish() {
    for (int i = 0; i < 15000; ++i) {
      if (controller.snapshot().mode == Mode::Holding)
        return;
      tick();
      require(controller.snapshot().mode != Mode::AwaitResume,
              "prepared motion stopped unexpectedly");
    }
    throw std::runtime_error("prepared motion did not finish");
  }
};
void timedExecution(const ModelConfig &model) {
  Fixture f(model);
  Model m(model);
  std::vector<TrajectoryPoint> points(3);
  for (auto &p : points)
    p.position = f.feedback.q;
  points[1].time_from_start = .6;
  points[1].position[0] += .015;
  points[2].time_from_start = 1.2;
  points[2].position[0] += .03;
  const auto expected =
      JointTrajectory::timed(points, f.config, m.lower(), m.upper());
  const double started = f.now;
  require(f.controller.trajectory(10, points), "timed trajectory rejected");
  require(f.controller.snapshot().trajectory_duration == 1.2,
          "strict timing was changed");
  double max_position_error = 0, max_velocity_error = 0;
  for (int i = 0; f.now - started < 1.2; ++i) {
    const double intervals[] = {.001, .003, .005, .002};
    f.tick(intervals[i % 4]);
    const auto sample = expected.sample(f.now - started);
    max_position_error =
        std::max(max_position_error, (f.output.q - sample.position).norm());
    max_velocity_error =
        std::max(max_velocity_error, (f.output.dq - sample.velocity).norm());
  }
  require(max_position_error < 1e-12 && max_velocity_error < 1e-12,
          "prepared trajectory was reshaped or did not follow actual time");
  f.finish();
  bool succeeded = false;
  for (const auto &event : f.controller.events())
    succeeded |= event.id == 10 && event.outcome == Outcome::Succeeded;
  require(succeeded, "prepared trajectory completion event missing");

  TrajectoryPoint short_target;
  short_target.time_from_start = .02;
  short_target.position = f.output.q;
  short_target.position[0] += .03;
  require(!f.controller.trajectory(11, {short_target}),
          "strict infeasible timing accepted");
  require(f.controller.trajectory(11, {short_target}, true),
          "retimed trajectory rejected");
  require(f.controller.snapshot().trajectory_time_scale > 1 &&
              f.controller.snapshot().trajectory_duration > .02,
          "retiming not exposed");
  f.finish();
  std::cout << "timed_reference_position_error=" << max_position_error
            << ",velocity_error=" << max_velocity_error << '\n';
}
void passingWaypoints(const ModelConfig &model) {
  Fixture f(model);
  Vec first = f.feedback.q, last = first;
  first[0] += .02;
  last[0] += .04;
  require(f.controller.jointWaypoints(20, {first, last}, .5),
          "joint waypoint plan rejected");
  double nearest = 1, speed = 0;
  while (f.controller.snapshot().mode != Mode::Holding) {
    f.tick();
    const double distance = std::abs(f.output.q[0] - first[0]);
    if (distance < nearest) {
      nearest = distance;
      speed = std::abs(f.output.dq[0]);
    }
  }
  require(nearest < .0001 && speed > .001,
          "continuous waypoint unnecessarily stopped");
  require((f.output.q - last).norm() < 1e-10, "waypoint endpoint incorrect");
  std::cout << "interior_waypoint_speed=" << speed << '\n';
}
void linearMotion(const ModelConfig &model) {
  Fixture f(model);
  Model m(model);
  const Pose start = m.fk(f.feedback.q);
  Pose target = start;
  target.translation() += Eigen::Vector3d(.003, 0, .002);
  require(f.controller.moveL(30, target, .3), "MoveL rejected");
  require(f.controller.snapshot().mode == Mode::CartesianPath,
          "MoveL mode incorrect");
  const Eigen::Vector3d direction = target.translation() - start.translation();
  double max_error = 0, max_rotation = 0;
  while (f.controller.snapshot().mode != Mode::Holding) {
    f.tick();
    const Pose actual = m.fk(f.output.q);
    const Eigen::Vector3d delta = actual.translation() - start.translation();
    const double along =
        std::clamp(delta.dot(direction) / direction.squaredNorm(), 0., 1.);
    max_error = std::max(max_error, (delta - along * direction).norm());
    max_rotation =
        std::max(max_rotation, poseError(actual, target).tail<3>().norm());
  }
  require(max_error < .00011 && max_rotation < .0011,
          "MoveL contour or orientation error");
  require(poseError(m.fk(f.output.q), target).norm() < .0011,
          "MoveL endpoint incorrect");
  target.translation().z() += .003;
  require(f.controller.moveL(31, target, .3), "second MoveL rejected");
  for (int i = 0; i < 30; ++i)
    f.tick();
  f.controller.operate(Operation::Stop);
  for (int i = 0; i < 3000 && f.controller.snapshot().mode != Mode::AwaitResume;
       ++i)
    f.tick();
  require(f.controller.snapshot().mode == Mode::AwaitResume,
          "MoveL did not use controlled stop");
  std::cout << "MoveL_contour_error_m=" << max_error
            << ",rotation_error_rad=" << max_rotation << '\n';
}
void intermediateCollision(const ModelConfig &model) {
  Model m(model);
  Vec center;
  center << 0, .8, -1.1, .3, 0, 0;
  auto obstructed = model;
  FixedBox box;
  box.name = "prepared_sweep_obstacle";
  box.size = Eigen::Vector3d::Constant(.015);
  box.placement.translation() = m.fk(center) * Eigen::Vector3d(.045, 0, 0);
  obstructed.fixed_boxes.push_back(box);
  Controller controller(obstructed, Config{});
  Feedback feedback;
  feedback.q = center;
  feedback.q[0] = -1;
  feedback.fresh = true;
  feedback.sequence = 1;
  feedback.received_at = 1;
  controller.step(feedback, 1);
  controller.operate(Operation::Enable);
  feedback.received_at = 1.002;
  feedback.sequence++;
  controller.step(feedback, 1.002);
  Vec end = feedback.q;
  end[0] = 1;
  require(!controller.jointWaypoints(40, {end}, .3),
          "prepared path ignored intermediate collision");
}
} // namespace
int main(int argc, char **argv) {
  try {
    require(argc == 2, "model argument missing");
    const auto model = defaultModelConfig(argv[1]);
    timedExecution(model);
    passingWaypoints(model);
    linearMotion(model);
    intermediateCollision(model);
    std::cout << "prepared controller motion tests passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
