#include "z1_control/runtime.hpp"
#include <XmlRpcValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <set>
#include <stdexcept>
#include <z1_msgs/CartesianReference.h>
#include <z1_msgs/ControlCommand.h>
#include <z1_msgs/GripperGoal.h>
#include <z1_msgs/ImpedanceGains.h>
#include <z1_msgs/JointGoal.h>
#include <z1_msgs/JointReference.h>
#include <z1_msgs/JointStream.h>
#include <z1_msgs/JointTrajectory.h>
#include <z1_msgs/JointWaypoints.h>
#include <z1_msgs/LinearPath.h>
#include <z1_msgs/MotionResult.h>
#include <z1_msgs/PoseGoal.h>
#include <z1_msgs/PoseStream.h>
#include <z1_msgs/Status.h>
namespace {
double number(const XmlRpc::XmlRpcValue &v) {
  if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    return static_cast<double>(v);
  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
    return static_cast<int>(v);
  throw std::runtime_error("configuration value must be numeric");
}
template <class V>
void readVector(ros::NodeHandle &p, const std::string &name, V &v) {
  XmlRpc::XmlRpcValue a;
  if (!p.getParam(name, a))
    return;
  if (a.getType() != XmlRpc::XmlRpcValue::TypeArray || a.size() != v.size())
    throw std::runtime_error("invalid vector parameter: " + name);
  for (int i = 0; i < a.size(); ++i)
    v[i] = number(a[i]);
}
template <class A> z1::Vec vector(const A &a) {
  z1::Vec out;
  for (int i = 0; i < 6; ++i)
    out[i] = a[i];
  return out;
}
template <class Derivative> z1::Vec cartesianVector(const Derivative &value) {
  z1::Vec out;
  out << value.linear.x, value.linear.y, value.linear.z, value.angular.x,
      value.angular.y, value.angular.z;
  return out;
}
bool pose(const geometry_msgs::Pose &msg, z1::Pose &p) {
  Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y,
                       msg.orientation.z);
  if (!q.coeffs().allFinite() || std::abs(q.norm() - 1) > 0.001)
    return false;
  p = z1::Pose::Identity();
  p.linear() = q.normalized().toRotationMatrix();
  p.translation() << msg.position.x, msg.position.y, msg.position.z;
  return z1::validPose(p);
}
geometry_msgs::Pose poseMessage(const z1::Pose &p) {
  geometry_msgs::Pose out;
  auto t = p.translation();
  Eigen::Quaterniond q(p.linear());
  out.position.x = t.x();
  out.position.y = t.y();
  out.position.z = t.z();
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
  out.orientation.w = q.w();
  return out;
}
class Node {
  ros::CallbackQueue planning_queue_;
  ros::NodeHandle nh_, params_{"~"}, planning_nh_;
  z1::Config config_;
  std::unique_ptr<z1::Runtime> runtime_;
  std::string prefix_, base_frame_;
  double command_age_{0.2}, publish_rate_{100};
  ros::Publisher joints_, tcp_, status_, results_;
  std::vector<ros::Subscriber> subscriptions_;
  // Serialize low-rate result messages between callback threads; ROS publisher
  // itself is thread-safe.
  bool header(const std_msgs::Header &h, bool stream, bool base_required) {
    if (base_required && h.frame_id != base_frame_)
      return false;
    if (stream && h.stamp.isZero())
      return false;
    if (!h.stamp.isZero()) {
      double age = (ros::Time::now() - h.stamp).toSec();
      if (age > command_age_ || age < -0.02)
        return false;
    }
    return true;
  }
  double streamTime(const std_msgs::Header &h) {
    // Preserve the remaining validity of an aged target on the monotonic clock.
    return z1::monotonicTime() -
           std::max(0.0, (ros::Time::now() - h.stamp).toSec());
  }
  void result(uint64_t id, bool okay, const std::string &why) {
    z1_msgs::MotionResult out;
    out.header.stamp = ros::Time::now();
    out.command_id = id;
    out.outcome = okay ? z1_msgs::MotionResult::ACCEPTED
                       : z1_msgs::MotionResult::REJECTED;
    out.reason = why;
    results_.publish(out);
  }

public:
  Node() {
    planning_nh_.setCallbackQueue(&planning_queue_);
    std::string urdf, ip;
    int port{}, budget{};
    z1::RuntimeOptions runtime_options;
    params_.param("control_cpu", runtime_options.cpu_affinity, -1);
    params_.param("realtime_priority", runtime_options.realtime_priority, 0);
    if (!params_.getParam("model_file", urdf) || urdf.empty())
      throw std::runtime_error("model_file is required");
    params_.param<std::string>("joint_prefix", prefix_, "");
    base_frame_ = prefix_ + "link00";
    params_.param("command_max_age", command_age_, 0.2);
    params_.param("publish_rate", publish_rate_, 100.0);
    if (command_age_ <= 0 || !std::isfinite(command_age_) || publish_rate_ <= 0 ||
        !std::isfinite(publish_rate_))
      throw std::runtime_error("invalid adapter timing");
    for (auto entry :
         {std::make_pair("period", &config_.period),
          {"feedback_timeout", &config_.feedback_timeout},
          {"stream_timeout", &config_.stream_timeout},
          {"reference_preview", &config_.reference_preview},
          {"reference_extrapolation", &config_.reference_extrapolation},
          {"reference_velocity_filter", &config_.reference_velocity_filter},
          {"reference_horizon", &config_.reference_horizon},
          {"impedance_gain_time_constant",
           &config_.impedance_gain_time_constant},
          {"position_tolerance", &config_.position_tolerance},
          {"velocity_tolerance", &config_.velocity_tolerance},
          {"settle_timeout", &config_.settle_timeout},
          {"tcp_position_tolerance", &config_.tcp_position_tolerance},
          {"tcp_orientation_tolerance", &config_.tcp_orientation_tolerance}})
      params_.param(entry.first, *entry.second, *entry.second);
    readVector(params_, "velocity", config_.velocity);
    readVector(params_, "acceleration", config_.acceleration);
    readVector(params_, "jerk", config_.jerk);
    readVector(params_, "torque", config_.torque);
    readVector(params_, "torque_rate", config_.torque_rate);
    readVector(params_, "kp", config_.kp);
    readVector(params_, "kd", config_.kd);
    readVector(params_, "stiffness", config_.stiffness);
    readVector(params_, "damping", config_.damping);
    auto model = z1::defaultModelConfig(urdf);
    params_.param<std::string>("tool_frame", model.tool_frame, "tool");
    readVector(params_, "gravity", model.gravity);
    XmlRpc::XmlRpcValue pairs;
    if (params_.getParam("ignored_collision_pairs", pairs)) {
      if (pairs.getType() != XmlRpc::XmlRpcValue::TypeArray)
        throw std::runtime_error("collision pairs must be an array");
      model.ignored_pairs.clear();
      for (int i = 0; i < pairs.size(); ++i) {
        auto &a = pairs[i];
        if (a.getType() != XmlRpc::XmlRpcValue::TypeArray || a.size() != 2 ||
            a[0].getType() != XmlRpc::XmlRpcValue::TypeString ||
            a[1].getType() != XmlRpc::XmlRpcValue::TypeString)
          throw std::runtime_error("invalid collision pair");
        model.ignored_pairs.emplace_back(static_cast<std::string>(a[0]),
                                         static_cast<std::string>(a[1]));
      }
    }
    XmlRpc::XmlRpcValue boxes;
    if (params_.getParam("fixed_boxes", boxes)) {
      if (boxes.getType() != XmlRpc::XmlRpcValue::TypeArray)
        throw std::runtime_error("fixed_boxes must be an array");
      for (int i = 0; i < boxes.size(); ++i) {
        auto &b = boxes[i];
        if (b.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            !b.hasMember("name") || !b.hasMember("size") ||
            !b.hasMember("xyz") || !b.hasMember("rpy"))
          throw std::runtime_error("invalid fixed box");
        z1::FixedBox box;
        box.name = static_cast<std::string>(b["name"]);
        Eigen::Vector3d xyz, rpy;
        for (auto key : {"size", "xyz", "rpy"})
          if (b[key].getType() != XmlRpc::XmlRpcValue::TypeArray ||
              b[key].size() != 3)
            throw std::runtime_error("fixed box vector must have 3 entries");
        for (int j = 0; j < 3; ++j) {
          box.size[j] = number(b["size"][j]);
          xyz[j] = number(b["xyz"][j]);
          rpy[j] = number(b["rpy"][j]);
        }
        box.placement.translation() = xyz;
        box.placement.linear() =
            (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
                .toRotationMatrix();
        model.fixed_boxes.push_back(box);
      }
    }
    if (!params_.getParam("mcu_ip", ip) ||
        !params_.getParam("local_port", port) ||
        !params_.getParam("receive_budget_us", budget))
      throw std::runtime_error("mcu_ip, local_port and receive_budget_us are "
                               "required in the communication YAML");
    if (port <= 0 || port > 65535 || budget <= 0 ||
        budget * 1e-6 >= config_.period)
      throw std::runtime_error(
          "invalid port or receive budget leaves no control time");
    z1::McuConfig mcu{ip, static_cast<unsigned>(port), config_.period,
                      static_cast<unsigned>(budget)};
    runtime_ = std::make_unique<z1::Runtime>(
        model, config_,
        [mcu] { return std::make_unique<z1::McuTransport>(mcu); },
        runtime_options);
    joints_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    tcp_ = nh_.advertise<geometry_msgs::PoseStamped>("tcp_pose", 1);
    status_ = nh_.advertise<z1_msgs::Status>("status", 1, true);
    results_ = nh_.advertise<z1_msgs::MotionResult>("result", 128);
    subscriptions_.push_back(planning_nh_.subscribe<z1_msgs::JointGoal>(
        "joint_goal", 1, [this](const z1_msgs::JointGoal::ConstPtr &m) {
          if (!header(m->header, false, false)) {
            result(m->command_id, false, "expired goal");
            return;
          }
          runtime_->controller().goal(m->command_id, vector(m->positions),
                                      m->speed_scale);
        }));
    subscriptions_.push_back(planning_nh_.subscribe<z1_msgs::PoseGoal>(
        "pose_goal", 1, [this](const z1_msgs::PoseGoal::ConstPtr &m) {
          z1::Pose p;
          if (!header(m->header, false, true) || !pose(m->pose, p)) {
            result(m->command_id, false, "invalid pose, frame or timestamp");
            return;
          }
          runtime_->controller().poseGoal(m->command_id, p, m->speed_scale);
        }));
    subscriptions_.push_back(planning_nh_.subscribe<z1_msgs::JointTrajectory>(
        "joint_trajectory", 1,
        [this](const z1_msgs::JointTrajectory::ConstPtr &m) {
          if (!header(m->header, false, false)) {
            result(m->command_id, false, "expired trajectory");
            return;
          }
          std::vector<z1::TrajectoryPoint> points;
          points.reserve(m->points.size());
          for (const auto &point : m->points) {
            z1::TrajectoryPoint target;
            target.time_from_start = point.time_from_start.toSec();
            target.position = vector(point.positions);
            target.velocity = vector(point.velocities);
            target.acceleration = vector(point.accelerations);
            target.has_velocity = point.has_velocity;
            target.has_acceleration = point.has_acceleration;
            points.push_back(target);
          }
          runtime_->controller().trajectory(m->command_id, points, m->retime);
        }));
    subscriptions_.push_back(planning_nh_.subscribe<z1_msgs::JointWaypoints>(
        "joint_waypoints", 1,
        [this](const z1_msgs::JointWaypoints::ConstPtr &m) {
          if (!header(m->header, false, false)) {
            result(m->command_id, false, "expired waypoints");
            return;
          }
          std::vector<z1::Vec> points;
          points.reserve(m->waypoints.size());
          for (const auto &point : m->waypoints)
            points.push_back(vector(point.positions));
          runtime_->controller().jointWaypoints(m->command_id, points,
                                                m->speed_scale);
        }));
    subscriptions_.push_back(planning_nh_.subscribe<z1_msgs::LinearPath>(
        "linear_path", 1, [this](const z1_msgs::LinearPath::ConstPtr &m) {
          if (!header(m->header, false, true)) {
            result(m->command_id, false, "invalid path frame or timestamp");
            return;
          }
          std::vector<z1::Pose> points;
          points.reserve(m->poses.size());
          for (const auto &message : m->poses) {
            z1::Pose target;
            if (!pose(message, target)) {
              result(m->command_id, false, "invalid path pose");
              return;
            }
            points.push_back(target);
          }
          z1::LinearPathOptions options;
          options.speed = m->speed_scale;
          options.blend_radius = m->blend_radius;
          runtime_->controller().linearWaypoints(m->command_id, points,
                                                 options);
        }));
    subscriptions_.push_back(nh_.subscribe<z1_msgs::JointStream>(
        "joint_stream", 1,
        [this](const z1_msgs::JointStream::ConstPtr &m) {
          if (!header(m->header, true, false) ||
              !runtime_->controller().stream(m->session_id, m->sequence,
                                             vector(m->positions),
                                             streamTime(m->header)))
            ROS_WARN_THROTTLE(1, "Rejected joint stream (timestamp, session, "
                                 "sequence, mode or target)");
        },
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
    subscriptions_.push_back(nh_.subscribe<z1_msgs::PoseStream>(
        "pose_stream", 1,
        [this](const z1_msgs::PoseStream::ConstPtr &m) {
          z1::Pose p;
          if (!header(m->header, true, true) || !pose(m->pose, p) ||
              !runtime_->controller().stream(m->session_id, m->sequence, p,
                                             streamTime(m->header)))
            ROS_WARN_THROTTLE(1, "Rejected pose stream");
        },
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
    subscriptions_.push_back(nh_.subscribe<z1_msgs::JointReference>(
        "joint_reference", 32,
        [this](const z1_msgs::JointReference::ConstPtr &m) {
          if (!header(m->header, true, false)) {
            ROS_WARN_THROTTLE(1, "Rejected joint reference timestamp");
            return;
          }
          const auto ros_now = ros::Time::now();
          const double now = z1::monotonicTime();
          const double source =
              now - std::max(0.0, (ros_now - m->header.stamp).toSec());
          z1::JointReference target;
          target.time = m->target_time.isZero()
                            ? source
                            : now + (m->target_time - ros_now).toSec();
          target.position = vector(m->positions);
          target.has_velocity = m->has_velocity;
          target.velocity = vector(m->velocities);
          target.has_acceleration = m->has_acceleration;
          target.acceleration = vector(m->accelerations);
          if (!runtime_->controller().stream(m->session_id, m->sequence, target,
                                             source))
            ROS_WARN_THROTTLE(1, "Rejected joint reference");
        },
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
    subscriptions_.push_back(nh_.subscribe<z1_msgs::CartesianReference>(
        "pose_reference", 32,
        [this](const z1_msgs::CartesianReference::ConstPtr &m) {
          z1::CartesianReference target;
          if (!header(m->header, true, true) || !pose(m->pose, target.pose)) {
            ROS_WARN_THROTTLE(
                1, "Rejected Cartesian reference frame or timestamp");
            return;
          }
          const auto ros_now = ros::Time::now();
          const double now = z1::monotonicTime();
          const double source =
              now - std::max(0.0, (ros_now - m->header.stamp).toSec());
          target.time = m->target_time.isZero()
                            ? source
                            : now + (m->target_time - ros_now).toSec();
          target.has_velocity = m->has_velocity;
          target.velocity = cartesianVector(m->velocity);
          target.has_acceleration = m->has_acceleration;
          target.acceleration = cartesianVector(m->acceleration);
          if (!runtime_->controller().stream(m->session_id, m->sequence, target,
                                             source))
            ROS_WARN_THROTTLE(1, "Rejected Cartesian reference");
        },
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
    subscriptions_.push_back(nh_.subscribe<z1_msgs::ControlCommand>(
        "control", 16, [this](const z1_msgs::ControlCommand::ConstPtr &m) {
          if (m->operation > 7) {
            result(m->command_id, false, "unknown operation");
            return;
          }
          // STOP/DISABLE remain available even when the ROS clock or command
          // stamp is invalid.
          if (m->operation != z1_msgs::ControlCommand::STOP &&
              m->operation != z1_msgs::ControlCommand::DISABLE &&
              !header(m->header, false, false)) {
            result(m->command_id, false, "expired control command");
            return;
          }
          runtime_->controller().operate(
              static_cast<z1::Operation>(m->operation), m->command_id);
        }));
    subscriptions_.push_back(nh_.subscribe<z1_msgs::ImpedanceGains>(
        "impedance_gains", 1,
        [this](const z1_msgs::ImpedanceGains::ConstPtr &m) {
          bool okay = header(m->header, false, false) &&
                      runtime_->controller().gains(vector(m->stiffness),
                                                   vector(m->damping));
          result(m->command_id, okay,
                 okay ? "gains applied"
                      : "requires holding/impedance and finite positive gains");
        }));
    subscriptions_.push_back(nh_.subscribe<z1_msgs::GripperGoal>(
        "gripper_goal", 1, [this](const z1_msgs::GripperGoal::ConstPtr &m) {
          bool okay = header(m->header, false, false) &&
                      runtime_->controller().gripper(m->position);
          result(m->command_id, okay,
                 okay ? "gripper target accepted; no arrival result"
                      : "gripper absent, invalid target or mode");
        }));
    runtime_->start();
  }
  void run() {
    ros::AsyncSpinner spinner(2);
    // Offline preparation has its own queue. A long Cartesian fit or another
    // pending plan cannot occupy callbacks serving operations and live streams.
    ros::AsyncSpinner planner(1, &planning_queue_);
    spinner.start();
    planner.start();
    ros::WallRate rate(publish_rate_);
    sensor_msgs::JointState joints;
    while (ros::ok()) {
      auto s = runtime_->controller().snapshot();
      const auto timing = runtime_->timing();
      auto stamp = ros::Time::now();
      double age = z1::monotonicTime() - s.feedback.received_at;
      z1_msgs::Status status;
      status.header.stamp = stamp;
      status.header.frame_id = base_frame_;
      status.mode = static_cast<uint8_t>(s.mode);
      status.reason = s.reason;
      status.session_id = s.session;
      status.active_command_id = s.active_id;
      status.feedback_sequence = s.feedback.sequence;
      status.feedback_valid =
          s.feedback_valid && age >= 0 && age <= config_.feedback_timeout;
      status.new_feedback = s.feedback.fresh;
      status.feedback_age = s.feedback.sequence ? age : -1;
      status.has_gripper = s.feedback.has_gripper;
      status.collision_distance = s.collision_distance;
      status.collision_pair = s.collision_pair;
      status.compute_time = s.compute_time;
      status.overruns = s.overruns;
      status.model_revision = s.model_revision;
      status.loop_period = s.loop_period;
      status.max_loop_period = s.max_loop_period;
      status.runtime_cycles = timing.cycles;
      status.missed_control_slots = timing.missed_slots;
      status.exchange_time = timing.exchange_us * 1e-6;
      status.max_exchange_time = timing.max_exchange_us * 1e-6;
      status.wakeup_lateness = timing.wakeup_lateness_us * 1e-6;
      status.max_wakeup_lateness = timing.max_wakeup_lateness_us * 1e-6;
      status.realtime_scheduling = timing.realtime_applied;
      status.cpu_affinity_applied = timing.affinity_applied;
      status.realtime_error = timing.realtime_error;
      status.cpu_affinity_error = timing.affinity_error;
      status.trajectory_duration = s.trajectory_duration;
      status.trajectory_time_scale = s.trajectory_time_scale;
      status_.publish(status);
      if (s.feedback.sequence > 0) {
        joints.header.stamp = stamp;
        const size_t count = s.feedback.has_gripper ? 7 : 6;
        if (joints.name.size() != count) {
          joints.name.resize(count);
          joints.position.resize(count);
          joints.velocity.resize(count);
          joints.effort.resize(count);
          for (int i = 0; i < 6; ++i)
            joints.name[i] = prefix_ + "joint" + std::to_string(i + 1);
          if (s.feedback.has_gripper)
            joints.name[6] = prefix_ + "gripper_motor";
        }
        for (int i = 0; i < 6; ++i) {
          joints.position[i] = s.feedback.q[i];
          joints.velocity[i] = s.feedback.dq[i];
          joints.effort[i] = s.feedback.tau[i];
        }
        if (s.feedback.has_gripper) {
          joints.position[6] = s.feedback.gripper_q;
          joints.velocity[6] = s.feedback.gripper_dq;
          joints.effort[6] = s.feedback.gripper_tau;
        }
        joints_.publish(joints);
        if (status.feedback_valid) {
          geometry_msgs::PoseStamped p;
          p.header.stamp = stamp;
          p.header.frame_id = base_frame_;
          p.pose = poseMessage(s.tcp);
          tcp_.publish(p);
        }
      }
      for (auto &e : runtime_->controller().events()) {
        z1_msgs::MotionResult m;
        m.header.stamp = stamp;
        m.command_id = e.id;
        m.outcome = static_cast<uint8_t>(e.outcome);
        m.reason = e.reason;
        results_.publish(m);
      }
      rate.sleep();
    }
    spinner.stop();
    runtime_->stop();
    planner.stop();
  }
};
} // namespace
int main(int argc, char **argv) {
  ros::init(argc, argv, "z1_topic_node");
  try {
    Node node;
    node.run();
    return 0;
  } catch (const std::exception &e) {
    ROS_FATAL("%s", e.what());
    return 1;
  }
}
