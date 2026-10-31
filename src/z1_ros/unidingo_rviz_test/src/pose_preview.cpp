#include "z1_control/goal_planner.hpp"
#include <ros/ros.h>
#include <unidingo_rviz_test/PreviewPose.h>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {
template <class Vector>
void readVector(const ros::NodeHandle &node, const char *name, Vector &out) {
  // Match the controller's defaults when an optional vector is omitted.
  if (!node.hasParam(name))
    return;
  std::vector<double> values;
  if (!node.getParam(name, values) || values.size() != size_t(out.size()))
    throw std::runtime_error(std::string(name) + " must match the model dimensions");
  for (int i = 0; i < out.size(); ++i)
    out[i] = values[i];
}
class Preview {
  ros::NodeHandle private_{"~"};
  z1::Config config_;
  std::unique_ptr<z1::Model> model_;
  std::string prefix_, base_;
  ros::ServiceServer service_;

public:
  Preview() {
    std::string ns;
    if (!private_.getParam("controller_namespace", ns))
      throw std::runtime_error("controller_namespace is required");
    const ros::NodeHandle controller(ns);
    z1::ModelConfig model;
    if (!controller.getParam("model_file", model.urdf) ||
        !controller.getParam("joint_prefix", prefix_))
      throw std::runtime_error("controller model_file and joint_prefix are required");
    base_ = prefix_ + "link00";
    controller.param<std::string>("tool_frame", model.tool_frame, "tool");
    readVector(controller, "gravity", model.gravity);
    controller.param("period", config_.period, config_.period);
    controller.param("joint_limit_tolerance", config_.joint_limit_tolerance,
                      config_.joint_limit_tolerance);
    controller.param("position_tolerance", config_.position_tolerance,
                      config_.position_tolerance);
    readVector(controller, "velocity", config_.velocity);
    readVector(controller, "acceleration", config_.acceleration);
    readVector(controller, "jerk", config_.jerk);
    model_ = std::make_unique<z1::Model>(model);
    service_ = private_.advertiseService("preview", &Preview::preview, this);
  }
  bool preview(unidingo_rviz_test::PreviewPose::Request &req,
               unidingo_rviz_test::PreviewPose::Response &out) {
    out.model_revision = model_->revision();
    out.start_tolerance = config_.position_tolerance;
    try {
      if (req.header.frame_id != base_)
        throw std::invalid_argument("preview target must use " + base_);
      const auto &p = req.target_pose;
      const Eigen::Quaterniond q(p.orientation.w, p.orientation.x,
                                  p.orientation.y, p.orientation.z);
      if (!q.coeffs().allFinite() || std::abs(q.norm() - 1) > .001)
        throw std::invalid_argument("target quaternion must have unit length");
      z1::Pose target = z1::Pose::Identity();
      target.translation() << p.position.x, p.position.y, p.position.z;
      target.linear() = q.normalized().toRotationMatrix();
      z1::Vec start = Eigen::Map<const z1::Vec>(req.start_positions.data());
      if (!model_->withinLimits(start, -config_.joint_limit_tolerance))
        throw std::invalid_argument("preview start outside measured joint tolerance");
      start = start.cwiseMax(model_->lower()).cwiseMin(model_->upper());
      const auto plan = z1::planPoseGoal(*model_, config_, start, target,
                                         req.speed_scale);
      out.trajectory.header = req.header;
      for (int i = 1; i <= 6; ++i)
        out.trajectory.joint_names.push_back(prefix_ + "joint" + std::to_string(i));
      constexpr int segments = 100;
      const int count = plan.get_duration() > 0 ? segments : 0;
      for (int i = 0; i <= count; ++i) {
        const double t = count ? plan.get_duration() * i / count : 0;
        std::array<double, 6> position, velocity, acceleration;
        plan.at_time(t, position, velocity, acceleration);
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.assign(position.begin(), position.end());
        point.velocities.assign(velocity.begin(), velocity.end());
        point.accelerations.assign(acceleration.begin(), acceleration.end());
        point.time_from_start = ros::Duration(t);
        out.trajectory.points.push_back(point);
      }
      out.success = true;
    } catch (const std::exception &error) {
      out.reason = error.what();
    }
    return true;
  }
};
} // namespace
int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_preview");
  try {
    Preview preview;
    ros::spin();
  } catch (const std::exception &error) {
    ROS_FATAL_STREAM(error.what());
    return 1;
  }
}
