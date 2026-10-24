#include "z1_control/cartesian_path.hpp"
#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace z1 {
namespace {
constexpr double rotation_metres = 0.24;
constexpr double translation_speed = 0.12;
constexpr double rotation_speed = 0.5;

Pose interpolate(const Pose &a, const Pose &b, double u) {
  Pose result = Pose::Identity();
  result.translation() = (1 - u) * a.translation() + u * b.translation();
  result.linear() = Eigen::Quaterniond(a.linear())
                        .slerp(u, Eigen::Quaterniond(b.linear()))
                        .normalized()
                        .toRotationMatrix();
  return result;
}

double length(const Pose &a, const Pose &b) {
  const Vec delta = poseError(a, b);
  return std::max(delta.head<3>().norm(),
                  rotation_metres * delta.tail<3>().norm());
}

struct Piece {
  std::array<Pose, 6> control;
  double start{}, duration{};
  bool bezier{}, stop_after{};
  double initial_slope{}, final_slope{};

  Pose at(double time) const {
    const double u = std::clamp((time - start) / duration, 0., 1.);
    if (bezier) {
      auto values = control;
      // Geodesic de Casteljau: the endpoint triples follow a common geodesic,
      // matching the straight pieces' velocity and zero acceleration.
      for (size_t level = 5; level > 0; --level)
        for (size_t i = 0; i < level; ++i)
          values[i] = interpolate(values[i], values[i + 1], u);
      return values[0];
    }
    const double m0 = initial_slope, m1 = final_slope;
    const double c3 = 10 - 6 * m0 - 4 * m1;
    const double c4 = -15 + 8 * m0 + 7 * m1;
    const double c5 = 6 - 3 * m0 - 3 * m1;
    const double s = u * (m0 + u * u * (c3 + u * (c4 + u * c5)));
    return interpolate(control.front(), control.back(), s);
  }
};

std::vector<Piece> makePath(const std::vector<Pose> &poses,
                            const LinearPathOptions &options) {
  const size_t n = poses.size();
  std::vector<double> lengths(n - 1), trims(n, 0);
  for (size_t i = 0; i + 1 < n; ++i)
    lengths[i] = length(poses[i], poses[i + 1]);
  for (size_t i = 1; i + 1 < n; ++i)
    trims[i] = std::min(
        {options.blend_radius, 0.45 * lengths[i - 1], 0.45 * lengths[i]});

  std::vector<Piece> path;
  double clock = 0;
  const double rate = translation_speed * options.speed;
  for (size_t i = 0; i + 1 < n; ++i) {
    const double usable = lengths[i] - trims[i] - trims[i + 1];
    const Pose from =
        interpolate(poses[i], poses[i + 1], trims[i] / lengths[i]);
    const Pose to =
        interpolate(poses[i], poses[i + 1], 1 - trims[i + 1] / lengths[i]);
    const bool moving_start = trims[i] > 0, moving_end = trims[i + 1] > 0;
    // A one-ended ramp has s(u)=2u^3-u^4 (or its reverse); a two-ended
    // ramp is the quintic smoothstep. Their peak Cartesian speed is rate.
    const double duration = usable / rate *
                            (moving_start && moving_end   ? 1
                             : moving_start || moving_end ? 2
                                                          : 1.875);
    Piece line;
    line.control.front() = from;
    line.control.back() = to;
    line.start = clock;
    line.duration = duration;
    line.initial_slope = moving_start ? duration * rate / usable : 0;
    line.final_slope = moving_end ? duration * rate / usable : 0;
    line.stop_after = !moving_end;
    path.push_back(line);
    clock += duration;

    if (moving_end) {
      const size_t corner = i + 1;
      const double trim = trims[corner];
      Piece blend;
      blend.bezier = true;
      blend.start = clock;
      blend.duration = 2 * trim / rate;
      blend.control[0] = to;
      blend.control[1] =
          interpolate(poses[i], poses[corner], 1 - 0.6 * trim / lengths[i]);
      blend.control[2] =
          interpolate(poses[i], poses[corner], 1 - 0.2 * trim / lengths[i]);
      blend.control[3] = interpolate(poses[corner], poses[corner + 1],
                                     0.2 * trim / lengths[corner]);
      blend.control[4] = interpolate(poses[corner], poses[corner + 1],
                                     0.6 * trim / lengths[corner]);
      blend.control[5] =
          interpolate(poses[corner], poses[corner + 1], trim / lengths[corner]);
      path.push_back(blend);
      clock += blend.duration;
    }
  }
  return path;
}

Pose at(const std::vector<Piece> &path, double time) {
  const auto found = std::upper_bound(
      path.begin(), path.end(), time,
      [](double t, const Piece &piece) { return t < piece.start; });
  return (found == path.begin() ? path.front() : *std::prev(found)).at(time);
}

// Model::ik intentionally has a looser goal tolerance. Offline contour fitting
// needs a smaller residual so the requested path tolerance remains meaningful.
Vec preciseIk(Model &model, const Pose &target, const Vec &seed) {
  Vec q = seed;
  for (int iteration = 0; iteration < 100; ++iteration) {
    const Vec error = poseError(model.fk(q), target);
    if (error.head<3>().norm() < 1e-9 && error.tail<3>().norm() < 1e-9)
      return q;
    const Jacobian J = model.jacobian(q);
    Vec delta =
        J.transpose() *
        (J * J.transpose() + 1e-10 * Jacobian::Identity()).ldlt().solve(error);
    if (delta.norm() > 0.15)
      delta *= 0.15 / delta.norm();
    bool improved = false;
    for (int trial = 0; trial < 12; ++trial) {
      const Vec next =
          (q + delta).cwiseMax(model.lower()).cwiseMin(model.upper());
      if (poseError(model.fk(next), target).squaredNorm() <
          error.squaredNorm()) {
        q = next;
        improved = true;
        break;
      }
      delta *= 0.5;
    }
    if (!improved)
      break;
  }
  throw std::runtime_error(
      "MoveL path is unreachable on the continuous IK branch");
}

std::vector<TrajectoryPoint> fit(Model &model, const Vec &start,
                                 const std::vector<Piece> &path,
                                 double interval) {
  std::vector<TrajectoryPoint> points;
  std::vector<bool> stops;
  points.push_back({0, start, Vec::Zero(), Vec::Zero(), true, true});
  stops.push_back(true);
  Vec seed = start;
  for (const auto &piece : path) {
    const size_t count =
        std::max<size_t>(8, std::ceil(piece.duration / interval));
    for (size_t i = 1; i <= count; ++i) {
      const double time = piece.start + piece.duration * i / count;
      const Vec q = preciseIk(model, piece.at(time), seed);
      if ((q - seed).cwiseAbs().maxCoeff() > 0.35)
        throw std::runtime_error(
            "MoveL path crosses an IK branch discontinuity");
      points.push_back({time, q, Vec::Zero(), Vec::Zero(), true, true});
      stops.push_back(i == count && piece.stop_after);
      seed = q;
    }
  }
  // Five-point local polynomial differentiation on the actual, possibly
  // nonuniform knot times. This keeps the interpolant C2 across piece joins.
  for (size_t i = 1; i + 1 < points.size(); ++i) {
    if (stops[i])
      continue;
    const size_t begin = std::min(points.size() - 5, i > 1 ? i - 2 : size_t(0));
    const double scale =
        points[begin + 4].time_from_start - points[begin].time_from_start;
    Eigen::Matrix<double, 5, 5> powers;
    Eigen::Matrix<double, 5, 6> positions;
    for (size_t k = 0; k < 5; ++k) {
      const double u =
          (points[begin + k].time_from_start - points[i].time_from_start) /
          scale;
      double power = 1;
      for (size_t j = 0; j < 5; ++j) {
        powers(k, j) = power;
        power *= u;
      }
      positions.row(k) =
          (points[begin + k].position - points[i].position).transpose();
    }
    const Eigen::Matrix<double, 5, 6> coefficients =
        powers.colPivHouseholderQr().solve(positions);
    points[i].velocity = coefficients.row(1).transpose() / scale;
    points[i].acceleration =
        2 * coefficients.row(2).transpose() / (scale * scale);
  }
  return points;
}
} // namespace

JointTrajectory planLinearPath(Model &model, const Vec &start,
                               const std::vector<Pose> &targets,
                               const Config &config,
                               const LinearPathOptions &options) {
  if (!model.withinLimits(start) || targets.empty() ||
      !std::isfinite(options.speed) || options.speed <= 0 ||
      options.speed > 1 || !std::isfinite(options.blend_radius) ||
      options.blend_radius < 0 ||
      !std::isfinite(options.translation_tolerance) ||
      options.translation_tolerance <= 0 ||
      !std::isfinite(options.rotation_tolerance) ||
      options.rotation_tolerance <= 0)
    throw std::invalid_argument("invalid MoveL path options or start");
  std::vector<Pose> poses{model.fk(start)};
  for (const auto &target : targets) {
    if (!validPose(target))
      throw std::invalid_argument("invalid MoveL target pose");
    if (length(poses.back(), target) > 1e-10)
      poses.push_back(target);
  }
  if (poses.size() == 1) {
    const std::vector<TrajectoryPoint> still{
        {0, start, Vec::Zero(), Vec::Zero(), true, true},
        {config.period, start, Vec::Zero(), Vec::Zero(), true, true}};
    return JointTrajectory::timed(still, config, model.lower(), model.upper(),
                                  true);
  }
  const auto path = makePath(poses, options);
  for (int refinement = 0; refinement < 5; ++refinement) {
    auto points = fit(model, start, path, 0.02 / (1 << refinement));
    auto result = JointTrajectory::timed(points, config, model.lower(),
                                         model.upper(), true);
    const double scale = result.timeScale();
    bool accurate = true;
    double task_scale = 1;
    // Check the actual joint interpolant, not just its IK knots. Refining
    // samples reduces contour error without modifying the requested path.
    for (size_t i = 0; i + 1 < points.size(); ++i)
      for (int sample = 0; sample <= 8; ++sample) {
        const double time =
            points[i].time_from_start +
            sample / 8. *
                (points[i + 1].time_from_start - points[i].time_from_start);
        const auto q = result.sample(time * scale);
        const Vec error = poseError(model.fk(q.position), at(path, time));
        accurate &= error.head<3>().norm() <= options.translation_tolerance &&
                    error.tail<3>().norm() <= options.rotation_tolerance;
        const Vec twist = model.jacobian(q.position) * q.velocity;
        task_scale = std::max(
            {task_scale,
             twist.head<3>().norm() / (translation_speed * options.speed),
             twist.tail<3>().norm() / (rotation_speed * options.speed)});
      }
    if (accurate) {
      if (task_scale > 1 + 1e-6)
        result = result.scaled(task_scale * (1 + 1e-6));
      return result;
    }
  }
  throw std::runtime_error(
      "MoveL joint interpolation exceeds Cartesian path tolerance");
}

} // namespace z1
