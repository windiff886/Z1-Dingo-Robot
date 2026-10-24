#include "z1_control/reference.hpp"
#include <algorithm>
#include <cmath>
#include <pinocchio/spatial/explog.hpp>

namespace z1 {
namespace {
Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}
Eigen::Matrix3d leftJacobian(const Eigen::Vector3d &r) {
  const double s = r.squaredNorm();
  const auto K = skew(r);
  if (s < 1e-8)
    return Eigen::Matrix3d::Identity() + (0.5 - s / 24) * K +
           (1.0 / 6 - s / 120) * K * K;
  const double t = std::sqrt(s);
  return Eigen::Matrix3d::Identity() + (1 - std::cos(t)) / s * K +
         (t - std::sin(t)) / (s * t) * K * K;
}
Eigen::Matrix3d inverseLeftJacobian(const Eigen::Vector3d &r) {
  const double s = r.squaredNorm();
  const double c =
      s < 1e-8 ? 1.0 / 12 + s / 720
               : (1 - 0.5 * std::sqrt(s) / std::tan(0.5 * std::sqrt(s))) / s;
  const auto K = skew(r);
  return Eigen::Matrix3d::Identity() - 0.5 * K + c * K * K;
}
Eigen::Matrix3d leftJacobianDerivative(const Eigen::Vector3d &r,
                                       const Eigen::Vector3d &v) {
  const double s = r.squaredNorm(), dot = r.dot(v);
  double a, b, da, db;
  if (s < 1e-8) {
    a = 0.5 - s / 24;
    b = 1.0 / 6 - s / 120;
    da = -dot / 12;
    db = -dot / 60;
  } else {
    const double t = std::sqrt(s), dt = dot / t;
    a = (1 - std::cos(t)) / s;
    b = (t - std::sin(t)) / (s * t);
    da = (std::sin(t) / s - 2 * a / t) * dt;
    db = ((1 - std::cos(t)) / (s * t) - 3 * b / t) * dt;
  }
  const auto K = skew(r), dK = skew(v);
  return da * K + a * dK + db * K * K + b * (dK * K + K * dK);
}
Vec difference(const JointReference &a, const JointReference &b) {
  return b.position - a.position;
}
Vec difference(const CartesianReference &a, const CartesianReference &b) {
  Vec d;
  d.head<3>() = b.pose.translation() - a.pose.translation();
  d.tail<3>() = pinocchio::log3(
      Eigen::Matrix3d(b.pose.linear() * a.pose.linear().transpose()));
  return d;
}
JointReference polynomial(const JointReference &a, const JointReference &b,
                          double time) {
  const double h = b.time - a.time, x = time - a.time;
  const Vec delta = b.position - a.position;
  Vec c0 = a.position, c1 = a.velocity, c2, c3, c4 = Vec::Zero(),
      c5 = Vec::Zero();
  if (a.has_acceleration && b.has_acceleration) {
    c2 = a.acceleration / 2;
    const Vec p = delta - c1 * h - c2 * h * h;
    const Vec v = b.velocity - c1 - 2 * c2 * h;
    const Vec acc = b.acceleration - 2 * c2;
    c3 = 10 * p / std::pow(h, 3) - 4 * v / (h * h) + acc / (2 * h);
    c4 = -15 * p / std::pow(h, 4) + 7 * v / std::pow(h, 3) - acc / (h * h);
    c5 = 6 * p / std::pow(h, 5) - 3 * v / std::pow(h, 4) +
         acc / (2 * std::pow(h, 3));
  } else {
    c2 = 3 * delta / (h * h) - (2 * a.velocity + b.velocity) / h;
    c3 = -2 * delta / std::pow(h, 3) + (a.velocity + b.velocity) / (h * h);
  }
  JointReference out;
  out.time = time;
  out.position = c0 + x * (c1 + x * (c2 + x * (c3 + x * (c4 + x * c5))));
  out.velocity = c1 + x * (2 * c2 + x * (3 * c3 + x * (4 * c4 + x * 5 * c5)));
  out.acceleration = 2 * c2 + x * (6 * c3 + x * (12 * c4 + x * 20 * c5));
  out.has_velocity = out.has_acceleration = true;
  return out;
}
JointReference interpolate(const JointReference &a, const JointReference &b,
                           double time) {
  return polynomial(a, b, time);
}
CartesianReference interpolate(const CartesianReference &a,
                               const CartesianReference &b, double time) {
  JointReference x, y;
  x.time = a.time;
  y.time = b.time;
  x.position.head<3>() = a.pose.translation();
  y.position.head<3>() = b.pose.translation();
  y.position.tail<3>() = difference(a, b).tail<3>();
  x.velocity = a.velocity;
  y.velocity = b.velocity;
  const auto inverse = inverseLeftJacobian(y.position.tail<3>());
  y.velocity.tail<3>() = inverse * b.velocity.tail<3>();
  x.acceleration = a.acceleration;
  y.acceleration = b.acceleration;
  y.acceleration.tail<3>() =
      inverse *
      (b.acceleration.tail<3>() -
       leftJacobianDerivative(y.position.tail<3>(), y.velocity.tail<3>()) *
           y.velocity.tail<3>());
  x.has_acceleration = a.has_acceleration;
  y.has_acceleration = b.has_acceleration;
  const auto state = polynomial(x, y, time);
  CartesianReference out;
  out.time = time;
  out.pose.translation() = state.position.head<3>();
  out.pose.linear() =
      pinocchio::exp3(state.position.tail<3>().eval()) * a.pose.linear();
  out.velocity = state.velocity;
  out.acceleration = state.acceleration;
  const auto J = leftJacobian(state.position.tail<3>());
  out.velocity.tail<3>() = J * state.velocity.tail<3>();
  out.acceleration.tail<3>() =
      J * state.acceleration.tail<3>() +
      leftJacobianDerivative(state.position.tail<3>(),
                             state.velocity.tail<3>()) *
          state.velocity.tail<3>();
  out.has_velocity = out.has_acceleration = true;
  return out;
}
JointReference extrapolate(JointReference p, double time, double maximum) {
  const double t = std::clamp(time - p.time, 0.0, maximum);
  p.position += p.velocity * t + 0.5 * p.acceleration * t * t;
  p.velocity += p.acceleration * t;
  if (time - p.time > maximum) {
    p.velocity.setZero();
    p.acceleration.setZero();
  }
  p.time = time;
  return p;
}
CartesianReference extrapolate(CartesianReference p, double time,
                               double maximum) {
  const double t = std::clamp(time - p.time, 0.0, maximum);
  p.pose.translation() +=
      p.velocity.head<3>() * t + 0.5 * p.acceleration.head<3>() * t * t;
  const Eigen::Vector3d r =
      p.velocity.tail<3>() * t + 0.5 * p.acceleration.tail<3>() * t * t;
  const Eigen::Vector3d dr =
      p.velocity.tail<3>() + p.acceleration.tail<3>() * t;
  p.pose.linear() = pinocchio::exp3(r) * p.pose.linear();
  p.velocity.head<3>() += p.acceleration.head<3>() * t;
  p.velocity.tail<3>() = leftJacobian(r) * dr;
  p.acceleration.tail<3>() = (leftJacobian(r) * p.acceleration.tail<3>() +
                              leftJacobianDerivative(r, dr) * dr)
                                 .eval();
  if (time - p.time > maximum) {
    p.velocity.setZero();
    p.acceleration.setZero();
  }
  p.time = time;
  return p;
}
} // namespace

template <class R> void ReferenceTimeline<R>::reset(const R &initial) {
  count_ = 1;
  points_[0] = initial;
}
template <class R>
bool ReferenceTimeline<R>::push(R point, double filter_time) {
  size_t keep = count_;
  while (keep && points_[keep - 1].time >= point.time)
    --keep;
  if (keep == points_.size())
    return false;
  if (!point.has_velocity) {
    if (keep) {
      const auto &previous = points_[keep - 1];
      const double dt = point.time - previous.time;
      const double alpha = dt / (filter_time + dt);
      point.velocity =
          previous.velocity +
          alpha * (difference(previous, point) / dt - previous.velocity);
    } else
      point.velocity.setZero();
  }
  if (!point.has_acceleration) {
    if (keep) {
      const auto &previous = points_[keep - 1];
      const double dt = point.time - previous.time;
      const double alpha = dt / (filter_time + dt);
      point.acceleration = previous.acceleration +
                           alpha * ((point.velocity - previous.velocity) / dt -
                                    previous.acceleration);
    } else
      point.acceleration.setZero();
  }
  points_[keep] = point;
  count_ = keep + 1;
  return true;
}
template <class R>
R ReferenceTimeline<R>::sample(double time, double maximum) const {
  if (time <= points_[0].time) {
    auto out = points_[0];
    out.time = time;
    return out;
  }
  for (size_t i = 1; i < count_; ++i)
    if (time <= points_[i].time)
      return interpolate(points_[i - 1], points_[i], time);
  return extrapolate(points_[count_ - 1], time, maximum);
}
template <class R> void ReferenceTimeline<R>::prune(double time) {
  size_t first = 0;
  while (first + 1 < count_ && points_[first + 1].time <= time)
    ++first;
  for (size_t i = first; i < count_; ++i)
    points_[i - first] = points_[i];
  count_ -= first;
}
template class ReferenceTimeline<JointReference>;
template class ReferenceTimeline<CartesianReference>;
} // namespace z1
