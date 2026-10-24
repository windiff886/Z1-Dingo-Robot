#include "z1_control/cartesian_path.hpp"
#include "z1_control/controller.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace z1;
namespace {
void require(bool value, const char *reason) {
  if (!value)
    throw std::runtime_error(reason);
}

void endpoint(Model &model, const JointTrajectory &path, const Vec &start,
              const Pose &target) {
  require((path.sample(0).position - start).norm() < 1e-10,
          "MoveL changed its starting configuration");
  const auto end = path.sample(path.duration());
  require(poseError(model.fk(end.position), target).norm() < 1e-7,
          "MoveL endpoint mismatch");
  require(end.velocity.norm() < 1e-8 && end.acceleration.norm() < 1e-7,
          "MoveL endpoint is not at rest");
}

void limits(Model &model, const JointTrajectory &path, const Config &config) {
  for (int i = 0; i <= 2000; ++i) {
    const auto sample = path.sample(path.duration() * i / 2000.);
    require(model.withinLimits(sample.position), "MoveL joint position limit");
    require((sample.velocity.cwiseAbs().array() <=
             config.velocity.array() * (1 + 1e-6))
                .all(),
            "MoveL joint velocity limit");
    require((sample.acceleration.cwiseAbs().array() <=
             config.acceleration.array() * (1 + 1e-6))
                .all(),
            "MoveL joint acceleration limit");
    require((sample.jerk.cwiseAbs().array() <= config.jerk.array() * (1 + 1e-6))
                .all(),
            "MoveL joint jerk limit");
    const Vec twist = model.jacobian(sample.position) * sample.velocity;
    require(twist.head<3>().norm() <= 0.12 * (1 + 1e-3) &&
                twist.tail<3>().norm() <= 0.5 * (1 + 1e-3),
            "MoveL TCP speed limit");
  }
}
} // namespace

int main(int argc, char **argv) {
  try {
    require(argc == 2, "model argument missing");
    Model model(defaultModelConfig(argv[1]));
    Config config;
    Vec start;
    start << 0.1, 0.8, -1.1, 0.3, 0.35, -0.2;
    const Pose initial = model.fk(start);
    Pose target = initial;
    target.translation() += Eigen::Vector3d(0.008, 0.006, 0.004);
    const auto preparation_start = std::chrono::steady_clock::now();
    const auto line = planLinearPath(model, start, {target}, config);
    const double preparation_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - preparation_start)
            .count();
    endpoint(model, line, start, target);
    limits(model, line, config);
    const Eigen::Vector3d delta = target.translation() - initial.translation();
    double previous_progress = -1e-7;
    double max_contour_error = 0, max_orientation_error = 0;
    for (int i = 0; i <= 1200; ++i) {
      const Pose actual =
          model.fk(line.sample(line.duration() * i / 1200.).position);
      const Eigen::Vector3d offset =
          actual.translation() - initial.translation();
      const double progress = offset.dot(delta) / delta.squaredNorm();
      max_contour_error =
          std::max(max_contour_error, (offset - progress * delta).norm());
      max_orientation_error = std::max(
          max_orientation_error, poseError(actual, initial).tail<3>().norm());
      require((offset - progress * delta).norm() < 1e-4,
              "MoveL fixed-orientation translation left the straight line");
      require(progress >= previous_progress - 1e-6 && progress <= 1 + 1e-6,
              "MoveL translation reverses or overshoots");
      require(poseError(actual, initial).tail<3>().norm() < 1e-3,
              "MoveL changed fixed orientation");
      previous_progress = progress;
    }

    // Simultaneous translation and shortest-path rotation.
    Pose rotated = target;
    rotated.linear() =
        (Eigen::AngleAxisd(0.12, Eigen::Vector3d(1, 2, -1).normalized()) *
         initial.linear())
            .eval();
    const auto coupled = planLinearPath(model, start, {rotated}, config);
    endpoint(model, coupled, start, rotated);
    limits(model, coupled, config);
    const Vec total = poseError(initial, rotated);
    for (int i = 0; i <= 1000; ++i) {
      const Pose actual =
          model.fk(coupled.sample(coupled.duration() * i / 1000.).position);
      const double progress =
          (actual.translation() - initial.translation()).dot(delta) /
          delta.squaredNorm();
      const Pose expected = [&] {
        Pose value = Pose::Identity();
        value.translation() = initial.translation() + progress * delta;
        value.linear() = Eigen::AngleAxisd(progress * total.tail<3>().norm(),
                                           total.tail<3>().normalized())
                             .toRotationMatrix() *
                         initial.linear();
        return value;
      }();
      require(poseError(actual, expected).head<3>().norm() < 1e-4 &&
                  poseError(actual, expected).tail<3>().norm() < 1e-3,
              "MoveL translation and rotation lost their shared progress");
    }

    Pose corner = initial, last = initial;
    corner.translation().x() += 0.012;
    last.translation() += Eigen::Vector3d(0.012, 0.012, 0);
    const auto stopped = planLinearPath(model, start, {corner, last}, config);
    bool stopped_at_corner = false;
    for (double time : stopped.times()) {
      const auto sample = stopped.sample(time);
      if (poseError(model.fk(sample.position), corner).norm() < 1e-7)
        stopped_at_corner |=
            sample.velocity.norm() < 1e-8 && sample.acceleration.norm() < 1e-7;
    }
    require(stopped_at_corner, "zero-blend path does not stop at the waypoint");

    LinearPathOptions options;
    options.blend_radius = 0.004;
    const auto blended =
        planLinearPath(model, start, {corner, last}, config, options);
    endpoint(model, blended, start, last);
    limits(model, blended, config);
    double nearest = std::numeric_limits<double>::infinity(), corner_speed = 0;
    for (int i = 1; i < 2000; ++i) {
      const auto sample = blended.sample(blended.duration() * i / 2000.);
      const Pose actual = model.fk(sample.position);
      const double distance =
          (actual.translation() - corner.translation()).norm();
      if (distance < nearest) {
        nearest = distance;
        corner_speed = (model.jacobian(sample.position) * sample.velocity)
                           .head<3>()
                           .norm();
      }
      const Eigen::Vector3d a = actual.translation() - initial.translation();
      const double to_incoming =
          (a - Eigen::Vector3d(std::clamp(a.x(), 0., 0.012), 0, 0)).norm();
      const Eigen::Vector3d b = actual.translation() - corner.translation();
      const double to_outgoing =
          (b - Eigen::Vector3d(0, std::clamp(b.y(), 0., 0.012), 0)).norm();
      require(std::min(to_incoming, to_outgoing) <= options.blend_radius + 1e-4,
              "MoveL blend leaves its advertised corner envelope");
    }
    require(nearest > 1e-5 && nearest < options.blend_radius &&
                corner_speed > 1e-4,
            "blended corner stops instead of passing continuously");

    Pose pure_rotation = initial;
    pure_rotation.linear() =
        Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        initial.linear();
    const auto turn = planLinearPath(model, start, {pure_rotation}, config);
    endpoint(model, turn, start, pure_rotation);
    for (int i = 0; i <= 1000; ++i)
      require((model.fk(turn.sample(turn.duration() * i / 1000.).position)
                   .translation() -
               initial.translation())
                      .norm() < 1e-4,
              "pure MoveL rotation translated the TCP");

    Pose rotation_corner = initial, rotation_last = initial;
    rotation_corner.linear() =
        Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitX()).toRotationMatrix() *
        initial.linear();
    rotation_last.linear() =
        Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitY()).toRotationMatrix() *
        rotation_corner.linear();
    const auto rotation_blend = planLinearPath(
        model, start, {rotation_corner, rotation_last}, config, options);
    endpoint(model, rotation_blend, start, rotation_last);
    limits(model, rotation_blend, config);
    double nearest_rotation = std::numeric_limits<double>::infinity();
    double rotation_corner_speed = 0;
    for (int i = 1; i < 1500; ++i) {
      const auto sample =
          rotation_blend.sample(rotation_blend.duration() * i / 1500.);
      const Pose actual = model.fk(sample.position);
      require((actual.translation() - initial.translation()).norm() < 1e-4,
              "blended pure rotation translated the TCP");
      const double angle = poseError(actual, rotation_corner).tail<3>().norm();
      if (angle < nearest_rotation) {
        nearest_rotation = angle;
        rotation_corner_speed =
            (model.jacobian(sample.position) * sample.velocity)
                .tail<3>()
                .norm();
      }
    }
    require(nearest_rotation < options.blend_radius / 0.24 &&
                rotation_corner_speed > 1e-4,
            "pure rotation blend does not pass its corner continuously");

    Pose unreachable = initial;
    unreachable.translation().x() = 100;
    bool rejected = false;
    try {
      (void)planLinearPath(model, start, {unreachable}, config);
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "unreachable MoveL path was accepted");
    std::cout << "MoveL line preparation_ms=" << preparation_ms
              << " duration=" << line.duration()
              << " contour_m=" << max_contour_error
              << " orientation_rad=" << max_orientation_error
              << " stopped_duration=" << stopped.duration()
              << " blended_duration=" << blended.duration()
              << " corner_speed_mps=" << corner_speed
              << " rotation_corner_speed_radps=" << rotation_corner_speed
              << '\n';
    std::cout << "MoveL line/coupled rotation, stopped/blended corners, limits "
                 "and unreachable tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
