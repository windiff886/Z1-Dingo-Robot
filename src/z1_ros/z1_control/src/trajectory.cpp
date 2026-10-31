#include "z1_control/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace z1 {
namespace {

struct Polynomial {
  std::array<double, 6> coefficients{};
  int degree{};

  double at(double s) const {
    double result = coefficients[degree];
    for (int i = degree - 1; i >= 0; --i)
      result = result * s + coefficients[i];
    return result;
  }

  Polynomial derivative() const {
    Polynomial result;
    result.degree = std::max(0, degree - 1);
    for (int i = 1; i <= degree; ++i)
      result.coefficients[i - 1] = i * coefficients[i];
    return result;
  }
};

// Derivative roots split a polynomial into monotone intervals. Isolating roots
// on those intervals also finds tangential/repeated roots, without a time grid.
std::vector<double> roots(Polynomial polynomial) {
  double magnitude = 0;
  for (int i = 0; i <= polynomial.degree; ++i)
    magnitude = std::max(magnitude, std::abs(polynomial.coefficients[i]));
  if (magnitude == 0)
    return {};
  const double tolerance =
      magnitude * 64 * std::numeric_limits<double>::epsilon();
  while (polynomial.degree > 0 &&
         std::abs(polynomial.coefficients[polynomial.degree]) <= tolerance)
    --polynomial.degree;
  if (polynomial.degree == 0)
    return {};

  auto critical = roots(polynomial.derivative());
  std::vector<double> boundaries{0};
  boundaries.insert(boundaries.end(), critical.begin(), critical.end());
  boundaries.push_back(1);
  std::vector<double> result;
  for (double boundary : boundaries)
    if (std::abs(polynomial.at(boundary)) <= tolerance)
      result.push_back(boundary);
  for (size_t i = 1; i < boundaries.size(); ++i) {
    double left = boundaries[i - 1], right = boundaries[i];
    double left_value = polynomial.at(left);
    const double right_value = polynomial.at(right);
    if (left_value == 0 || right_value == 0 ||
        std::signbit(left_value) == std::signbit(right_value))
      continue;
    for (unsigned iteration = 0; iteration < 64; ++iteration) {
      const double middle = (left + right) / 2;
      const double value = polynomial.at(middle);
      if (value == 0) {
        left = right = middle;
        break;
      }
      if (std::signbit(value) == std::signbit(left_value)) {
        left = middle;
        left_value = value;
      } else {
        right = middle;
      }
    }
    result.push_back((left + right) / 2);
  }
  std::sort(result.begin(), result.end());
  result.erase(
      std::unique(result.begin(), result.end(),
                  [](double a, double b) { return std::abs(a - b) < 1e-13; }),
      result.end());
  return result;
}

std::pair<double, double> extrema(const Polynomial &polynomial) {
  double lower = std::min(polynomial.at(0), polynomial.at(1));
  double upper = std::max(polynomial.at(0), polynomial.at(1));
  for (double root : roots(polynomial.derivative())) {
    const double value = polynomial.at(root);
    lower = std::min(lower, value);
    upper = std::max(upper, value);
  }
  return {lower, upper};
}

double absoluteMaximum(const Polynomial &polynomial) {
  const auto bounds = extrema(polynomial);
  return std::max(std::abs(bounds.first), std::abs(bounds.second));
}

void validateLimits(const Config &config, const Vec &lower, const Vec &upper) {
  if (!lower.allFinite() || !upper.allFinite() ||
      (lower.array() >= upper.array()).any() || !config.velocity.allFinite() ||
      !config.acceleration.allFinite() || !config.jerk.allFinite() ||
      (config.velocity.array() <= 0).any() ||
      (config.acceleration.array() <= 0).any() ||
      (config.jerk.array() <= 0).any())
    throw std::invalid_argument("invalid trajectory limits");
}

double interiorVelocity(double previous_slope, double next_slope,
                        double previous_duration, double next_duration) {
  if (previous_slope == 0 || next_slope == 0 ||
      std::signbit(previous_slope) != std::signbit(next_slope))
    return 0;
  // Weighted harmonic mean limits the speed approaching a short/slow segment.
  const double weight_previous = 2 * next_duration + previous_duration;
  const double weight_next = next_duration + 2 * previous_duration;
  const double harmonic =
      (weight_previous + weight_next) /
      (weight_previous / previous_slope + weight_next / next_slope);
  const double bound =
      2 * std::min(std::abs(previous_slope), std::abs(next_slope));
  return std::copysign(std::min(std::abs(harmonic), bound), harmonic);
}

} // namespace

JointTrajectory
JointTrajectory::timed(const std::vector<TrajectoryPoint> &points,
                       const Config &config, const Vec &lower, const Vec &upper,
                       bool retime) {
  validateLimits(config, lower, upper);
  if (points.size() < 2 || points.front().time_from_start != 0)
    throw std::invalid_argument(
        "trajectory requires at least two points at t=0");
  for (size_t i = 0; i < points.size(); ++i) {
    const auto &point = points[i];
    if (!std::isfinite(point.time_from_start) || !point.position.allFinite() ||
        !point.velocity.allFinite() || !point.acceleration.allFinite() ||
        (point.has_acceleration && !point.has_velocity) ||
        (i && point.time_from_start <= points[i - 1].time_from_start))
      throw std::invalid_argument("invalid trajectory point or time order");
  }

  std::vector<Vec> velocities(points.size(), Vec::Zero());
  for (size_t i = 0; i < points.size(); ++i) {
    if (points[i].has_velocity) {
      velocities[i] = points[i].velocity;
    } else if (i > 0 && i + 1 < points.size()) {
      const double previous_duration =
          points[i].time_from_start - points[i - 1].time_from_start;
      const double next_duration =
          points[i + 1].time_from_start - points[i].time_from_start;
      const Vec previous_slope =
          (points[i].position - points[i - 1].position) / previous_duration;
      const Vec next_slope =
          (points[i + 1].position - points[i].position) / next_duration;
      for (int axis = 0; axis < 6; ++axis)
        velocities[i][axis] =
            interiorVelocity(previous_slope[axis], next_slope[axis],
                             previous_duration, next_duration);
      if (!velocities[i].allFinite())
        throw std::invalid_argument("trajectory derivatives overflow");
    }
  }

  JointTrajectory result;
  result.segments_.reserve(points.size() - 1);
  result.times_.reserve(points.size());
  for (const auto &point : points)
    result.times_.push_back(point.time_from_start);
  double scale = 1;
  for (size_t i = 1; i < points.size(); ++i) {
    Segment segment;
    segment.duration =
        points[i].time_from_start - points[i - 1].time_from_start;
    const double dt = segment.duration;
    auto &c = segment.coefficients;
    c[0] = points[i - 1].position;
    c[1] = velocities[i - 1] * dt;
    c[2] = points[i - 1].has_acceleration
               ? Vec(0.5 * points[i - 1].acceleration * dt * dt)
               : Vec::Zero();
    const Vec position_delta = points[i].position - c[0] - c[1] - c[2];
    const Vec velocity_delta = velocities[i] * dt - c[1] - 2 * c[2];
    const Vec acceleration_delta =
        (points[i].has_acceleration ? Vec(points[i].acceleration * dt * dt)
                                    : Vec::Zero()) -
        2 * c[2];
    c[3] = 10 * position_delta - 4 * velocity_delta + 0.5 * acceleration_delta;
    c[4] = -15 * position_delta + 7 * velocity_delta - acceleration_delta;
    c[5] = 6 * position_delta - 3 * velocity_delta + 0.5 * acceleration_delta;
    for (const auto &coefficient : c)
      if (!coefficient.allFinite())
        throw std::invalid_argument("trajectory coefficients overflow");

    for (int axis = 0; axis < 6; ++axis) {
      Polynomial position;
      position.degree = 5;
      for (int power = 0; power <= 5; ++power)
        position.coefficients[power] = c[power][axis];
      const auto range = extrema(position);
      const double tolerance =
          1e-10 * (1 + std::max(std::abs(lower[axis]), std::abs(upper[axis])));
      if (range.first < lower[axis] - tolerance ||
          range.second > upper[axis] + tolerance)
        throw std::invalid_argument("trajectory exceeds joint position limit");
      const auto velocity = position.derivative();
      const auto acceleration = velocity.derivative();
      const auto jerk = acceleration.derivative();
      const double speed = absoluteMaximum(velocity) / dt;
      const double accel = absoluteMaximum(acceleration) / (dt * dt);
      const double jerk_value = absoluteMaximum(jerk) / (dt * dt * dt);
      if (!std::isfinite(speed) || !std::isfinite(accel) ||
          !std::isfinite(jerk_value))
        throw std::invalid_argument("trajectory derivatives overflow");
      scale = std::max({scale, speed / config.velocity[axis],
                        std::sqrt(accel / config.acceleration[axis]),
                        std::cbrt(jerk_value / config.jerk[axis])});
    }
    result.segments_.push_back(std::move(segment));
  }

  if (!retime && scale > 1 + 1e-10)
    throw std::invalid_argument(
        "trajectory exceeds velocity/acceleration/jerk limit");
  if (retime && scale > 1) {
    // Normalized polynomial coefficients remain unchanged. Stretching each
    // segment scales v, a and jerk by 1/s, 1/s^2 and 1/s^3 respectively.
    scale *= 1 + 1e-12;
    result.time_scale_ = scale;
    for (double &time : result.times_)
      time *= scale;
    for (auto &segment : result.segments_)
      segment.duration *= scale;
    if (!std::isfinite(result.duration()))
      throw std::invalid_argument("trajectory duration overflow");
  }
  return result;
}

JointTrajectory JointTrajectory::waypoints(const std::vector<Vec> &points,
                                           const Config &config,
                                           const Vec &lower, const Vec &upper,
                                           double speed) {
  validateLimits(config, lower, upper);
  if (points.size() < 2 || !std::isfinite(speed) || speed <= 0 || speed > 1 ||
      !std::isfinite(config.period) || config.period <= 0)
    throw std::invalid_argument("invalid trajectory waypoints or speed");
  Config scaled = config;
  scaled.velocity *= speed;
  scaled.acceleration *= speed * speed;
  scaled.jerk *= speed * speed * speed;
  std::vector<TrajectoryPoint> timed_points(points.size());
  double time = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    if (!points[i].allFinite())
      throw std::invalid_argument("invalid trajectory waypoint");
    if (i) {
      const Vec distance = (points[i] - points[i - 1]).cwiseAbs();
      double duration = config.period / speed;
      for (int axis = 0; axis < 6; ++axis)
        duration =
            std::max({duration, 1.875 * distance[axis] / scaled.velocity[axis],
                      std::sqrt(5.773502691896258 * distance[axis] /
                                scaled.acceleration[axis]),
                      std::cbrt(60 * distance[axis] / scaled.jerk[axis])});
      time += duration;
    }
    timed_points[i].time_from_start = time;
    timed_points[i].position = points[i];
  }
  return timed(timed_points, scaled, lower, upper, true);
}

TrajectorySample JointTrajectory::sample(double time) const {
  if (empty() || !std::isfinite(time))
    throw std::invalid_argument(
        "cannot sample empty trajectory or invalid time");
  const bool holding = time < 0 || time > duration();
  time = std::clamp(time, 0.0, duration());
  const auto next = std::upper_bound(times_.begin(), times_.end(), time);
  const size_t index = std::min(static_cast<size_t>(next - times_.begin() - 1),
                                segments_.size() - 1);
  const auto &segment = segments_[index];
  const auto &c = segment.coefficients;
  const double s = (time - times_[index]) / segment.duration;
  TrajectorySample result;
  result.position = c[5];
  for (int power = 4; power >= 0; --power)
    result.position = result.position * s + c[power];
  if (holding)
    return result;
  result.velocity = 5 * c[5];
  for (int power = 4; power >= 1; --power)
    result.velocity = result.velocity * s + power * c[power];
  result.velocity /= segment.duration;
  result.acceleration = 20 * c[5];
  for (int power = 4; power >= 2; --power)
    result.acceleration =
        result.acceleration * s + power * (power - 1) * c[power];
  result.acceleration /= segment.duration * segment.duration;
  result.jerk = 60 * c[5];
  for (int power = 4; power >= 3; --power)
    result.jerk =
        result.jerk * s + power * (power - 1) * (power - 2) * c[power];
  result.jerk /= segment.duration * segment.duration * segment.duration;
  return result;
}

double JointTrajectory::duration() const {
  return times_.empty() ? 0 : times_.back();
}

JointTrajectory JointTrajectory::scaled(double factor) const {
  if (!std::isfinite(factor) || factor < 1 ||
      !std::isfinite(duration() * factor) ||
      !std::isfinite(time_scale_ * factor))
    throw std::invalid_argument("invalid trajectory time scale");
  JointTrajectory result = *this;
  result.time_scale_ *= factor;
  for (double &time : result.times_)
    time *= factor;
  for (auto &segment : result.segments_)
    segment.duration *= factor;
  return result;
}

} // namespace z1
