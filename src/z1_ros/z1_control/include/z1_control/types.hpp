#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace z1 {
using Vec = Eigen::Matrix<double, 6, 1>;
using Jacobian = Eigen::Matrix<double, 6, 6>;
using Pose = Eigen::Isometry3d;
enum class Mode {
  Disabled,
  Holding,
  JointGoal,
  PoseGoal,
  JointStream,
  PoseStream,
  Impedance,
  Stopping,
  AwaitResume,
  Fault,
  JointTrajectory,
  CartesianPath
};
enum class Operation {
  Enable,
  Disable,
  Stop,
  Resume,
  JointStream,
  PoseStream,
  Impedance,
  Hold
};
enum class Outcome { Accepted, Succeeded, Rejected, Canceled, Stopped, Failed };
struct Event {
  uint64_t id{};
  Outcome outcome{};
  std::string reason;
};
struct Feedback {
  Vec q{Vec::Zero()}, dq{Vec::Zero()}, tau{Vec::Zero()};
  double received_at{};
  uint64_t sequence{};
  bool fresh{}, motor_fault{}, transport_fault{}, has_gripper{};
  double gripper_q{}, gripper_dq{}, gripper_tau{};
};
struct Output {
  Vec q{Vec::Zero()}, dq{Vec::Zero()}, tau{Vec::Zero()};
  bool enabled{};
  double gripper_q{}, gripper_kp{}, gripper_kd{};
};
struct FixedBox {
  std::string name;
  Pose placement{Pose::Identity()};
  Eigen::Vector3d size;
};
struct ModelConfig {
  std::string urdf, tool_frame{"tool"};
  Eigen::Vector3d gravity{0, 0, -9.81};
  std::vector<std::pair<std::string, std::string>> ignored_pairs;
  std::vector<FixedBox> fixed_boxes;
};
struct Config {
  double period{0.002}, feedback_timeout{0.08}, stream_timeout{0.2};
  double reference_preview{0.03}, reference_extrapolation{0.1},
      reference_velocity_filter{0.01}, reference_horizon{0.5},
      impedance_gain_time_constant{0.1};
  double collision_margin{0.008}, collision_warning{0.03};
  double position_tolerance{0.015}, velocity_tolerance{0.03},
      settle_timeout{3.0};
  double tcp_position_tolerance{0.005}, tcp_orientation_tolerance{0.025};
  double geometry_radius_bound{1.5}, validation_budget{2.0},
      lookahead_budget{0.0005};
  unsigned max_collision_samples{20000};
  Vec velocity{Vec::Constant(0.8)}, acceleration{Vec::Constant(1.5)},
      jerk{Vec::Constant(8)};
  Vec torque{(Vec() << 20, 30, 20, 12, 8, 8).finished()},
      torque_rate{Vec::Constant(1000)};
  Vec kp{(Vec() << 50, 80, 60, 35, 20, 20).finished()},
      kd{(Vec() << 8, 12, 10, 6, 3, 3).finished()};
  Vec stiffness{(Vec() << 30, 30, 30, 0.2, 0.2, 0.2).finished()};
  Vec damping{(Vec() << 4, 4, 4, 0.02, 0.02, 0.02).finished()};
};
struct Snapshot {
  Mode mode{Mode::Disabled};
  bool feedback_valid{};
  Feedback feedback;
  Pose tcp{Pose::Identity()};
  uint64_t session{}, active_id{};
  std::string reason;
  double collision_distance{}, compute_time{}, loop_period{}, max_loop_period{};
  uint64_t overruns{};
  std::string collision_pair;
  std::string model_revision;
  // Last accepted finite plan; seconds and dimensionless global time dilation.
  double trajectory_duration{}, trajectory_time_scale{1};
};
inline bool validPose(const Pose &p) {
  return p.matrix().allFinite() &&
         std::abs(p.linear().determinant() - 1) < 1e-6 &&
         (p.linear().transpose() * p.linear() - Eigen::Matrix3d::Identity())
                 .norm() < 1e-6;
}
} // namespace z1
