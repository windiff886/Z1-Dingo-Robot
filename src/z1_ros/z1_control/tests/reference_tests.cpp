#include "z1_control/controller.hpp"
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <fstream>
#include <iostream>
#include <limits>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <random>
#include <stdexcept>
using namespace z1;
void require(bool condition, const char *reason) {
  if (!condition)
    throw std::runtime_error(reason);
}
JointReference jointPoint(double t) {
  JointReference p;
  p.time = t;
  p.position.setConstant(std::sin(t));
  p.velocity.setConstant(std::cos(t));
  p.acceleration.setConstant(-std::sin(t));
  p.has_velocity = p.has_acceleration = true;
  return p;
}
CartesianReference posePoint(double t) {
  CartesianReference p;
  p.time = 10 + t;
  p.pose.translation() << t * t, 0, 0;
  p.pose.linear() = (Eigen::AngleAxisd(0.4 * t, Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(0.3 * t, Eigen::Vector3d::UnitY()))
                        .toRotationMatrix();
  p.velocity << 2 * t, 0, 0, -0.3 * std::sin(0.4 * t), 0.3 * std::cos(0.4 * t),
      0.4;
  p.acceleration << 2, 0, 0, -0.12 * std::cos(0.4 * t),
      -0.12 * std::sin(0.4 * t), 0;
  p.has_velocity = p.has_acceleration = true;
  return p;
}

// Full-frame FK and Jacobian provide an independent kinematics reference.
struct FullKinematics {
  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  ModelConfig config;
  FullKinematics(const ModelConfig &c) : config(c) {
    pinocchio::urdf::buildModel(c.urdf, model);
    data.reset(new pinocchio::Data(model));
  }
  // Match the previous separate FK and Jacobian evaluation, independently of
  // the SDK's combined tool-frame evaluation.
  Pose kinematics(const Vec &q, Jacobian &J) {
    const auto tool = model.getFrameId(config.tool_frame);
    pinocchio::framesForwardKinematics(model, *data, q);
    Pose pose;
    pose.matrix() = data->oMf[tool].toHomogeneousMatrix();
    pinocchio::computeJointJacobians(model, *data, q);
    pinocchio::updateFramePlacements(model, *data);
    J.setZero();
    pinocchio::getFrameJacobian(model, *data, tool,
                                pinocchio::LOCAL_WORLD_ALIGNED, J);
    return pose;
  }
};

// Restore the gripper coordinate to independently check the six-axis rigid
// reduction against a seven-coordinate model at q_gripper=dq_gripper=ddq_gripper=0.
pinocchio::Model articulatedGripper(const ModelConfig &config) {
  std::ifstream input(config.urdf);
  std::string xml((std::istreambuf_iterator<char>(input)), {});
  const std::string fixed = "<joint name=\"gripper_closed_mount\" type=\"fixed\">";
  const auto at = xml.find(fixed);
  require(at != std::string::npos, "official closed-gripper fixture missing");
  xml.replace(at, fixed.size(), "<joint name=\"jointGripper\" type=\"revolute\">");
  xml.insert(xml.find("</joint>", at),
             "<axis xyz=\"0 1 0\"/><limit lower=\"-1.5707963267948966\" "
             "upper=\"0\" effort=\"30\" velocity=\"3.1415\"/>");
  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(xml, model);
  model.gravity.linear() = config.gravity;
  require(model.nq == 7 && model.names[7] == "jointGripper",
          "articulated gripper coordinates differ from official arm order");
  return model;
}
int main(int argc, char **argv) {
  try {
    require(argc == 2, "model required");
    ReferenceTimeline<JointReference> joints;
    joints.reset(jointPoint(1));
    require(joints.push(jointPoint(1.1), 0.01), "joint point rejected");
    auto sample = joints.sample(1.05, 0.1), exact = jointPoint(1.05);
    require((sample.position - exact.position).norm() < 1e-8 &&
                (sample.velocity - exact.velocity).norm() < 1e-7 &&
                (sample.acceleration - exact.acceleration).norm() < 1e-5,
            "quintic interpolation does not preserve derivatives");
    const auto last_joint = jointPoint(1.1);
    const auto extrapolated = joints.sample(1.15, 0.1);
    require((extrapolated.position - last_joint.position -
             0.05 * last_joint.velocity - 0.5 * 0.05 * 0.05 *
                                               last_joint.acceleration)
                    .norm() < 1e-12,
            "joint reference extrapolation changed");
    require(joints.sample(1.3, 0.1).velocity.isZero(),
            "extrapolation did not end");
    auto replacement = jointPoint(1.08);
    joints.push(replacement, 0.01);
    require(joints.endTime() == replacement.time, "future replacement failed");
    joints.reset(jointPoint(1));
    for (int i = 1; i < 32; ++i)
      require(joints.push(jointPoint(1 + i * 0.01), 0.01),
              "timeline filled early");
    require(!joints.push(jointPoint(2), 0.01), "timeline is unbounded");
    joints.prune(1.3);
    require(joints.push(jointPoint(1.4), 0.01),
            "consumed capacity not reusable");

    ReferenceTimeline<CartesianReference> poses;
    poses.reset(posePoint(0));
    poses.push(posePoint(0.2), 0.01);
    const double t = 10.08, epsilon = 1e-5;
    auto pose = poses.sample(t, 0.1), before = poses.sample(t - epsilon, 0.1),
         after = poses.sample(t + epsilon, 0.1);
    require(validPose(pose.pose), "interpolation left SO(3)");
    const Vec numerical_velocity =
        poseError(before.pose, after.pose) / (2 * epsilon);
    require((numerical_velocity - pose.velocity).norm() < 1e-6,
            "Cartesian twist frame/convention mismatch");
    require(
        ((after.velocity - before.velocity) / (2 * epsilon) - pose.acceleration)
                .norm() < 1e-5,
        "Cartesian acceleration mismatch");
    require(poseError(pose.pose, posePoint(0.08).pose).norm() < 1e-7,
            "noncommuting rotation interpolation inaccurate");
    const auto last_pose = posePoint(0.2);
    const auto extrapolated_pose = poses.sample(10.25, 0.1);
    require((extrapolated_pose.pose.translation() -
             last_pose.pose.translation() -
             0.05 * last_pose.velocity.head<3>() -
             0.5 * 0.05 * 0.05 * last_pose.acceleration.head<3>())
                    .norm() < 1e-12 &&
                validPose(extrapolated_pose.pose),
            "Cartesian reference extrapolation changed");

    auto config = defaultModelConfig(argv[1]);
    Model optimized(config);
    auto articulated = articulatedGripper(config);
    pinocchio::Data articulated_data(articulated);
    require((optimized.velocityLimit().array() - 3.1415).abs().maxCoeff() < 1e-12,
            "control model lost official velocity limits");
    auto excessive = Config{};
    excessive.velocity.setConstant(4);
    bool excessive_rejected = false;
    try { Controller invalid(config, excessive); }
    catch (const std::invalid_argument &) { excessive_rejected = true; }
    require(excessive_rejected, "configuration above official velocity limit accepted");
    auto accelerated = config;
    accelerated.gravity << 0, 0, -25;
    Model accelerated_model(accelerated);
    const Vec middle = (optimized.lower() + optimized.upper()) * 0.5;
    require((accelerated_model.inverseDynamics(middle, Vec::Zero(),
                                               Vec::Zero()) -
             optimized.inverseDynamics(middle, Vec::Zero(), Vec::Zero()) *
                 (25 / 9.81))
                    .norm() < 1e-10,
            "finite base acceleration was rejected or changed dynamics");
    accelerated.gravity[2] = std::numeric_limits<double>::infinity();
    bool invalid_gravity_rejected = false;
    try {
      Model invalid(accelerated);
    } catch (const std::runtime_error &) {
      invalid_gravity_rejected = true;
    }
    require(invalid_gravity_rejected, "nonfinite gravity was accepted");
    FullKinematics full(config);
    std::mt19937 random(8675309);
    std::uniform_real_distribution<double> uniform(0, 1);
    for (int sample = 0; sample < 250; ++sample) {
      Vec q, uncertainty, delta;
      for (int j = 0; j < 6; ++j) {
        q[j] = optimized.lower()[j] +
               uniform(random) * (optimized.upper()[j] - optimized.lower()[j]);
        uncertainty[j] = 0.01 + 0.02 * uniform(random);
        delta[j] = (2 * uniform(random) - 1) * uncertainty[j];
      }
      Jacobian expected_jacobian;
      Eigen::Matrix<double, 7, 1> q7, v7, a7;
      q7 << q, 0;
      v7 << Vec::Constant(.02), 0;
      a7 << Vec::Constant(-.03), 0;
      const Vec torque = pinocchio::rnea(articulated, articulated_data, q7, v7, a7).head<6>();
      require((optimized.inverseDynamics(q, v7.head<6>(), a7.head<6>()) - torque).norm() < 1e-10,
              "six-axis gripper reduction differs from articulated closed gripper");
      const Pose expected_pose = full.kinematics(q, expected_jacobian);
      require((optimized.fk(q).matrix() - expected_pose.matrix()).norm() <
                      1e-12 &&
                  (optimized.jacobian(q) - expected_jacobian).norm() < 1e-12,
              "tool-only kinematics differs from full-frame evaluation");
      const Pose target = optimized.fk(q + delta);
      const Vec twist = delta * 0.1;
      Vec desired = poseError(expected_pose, target);
      desired.head<3>() *= 4;
      desired.tail<3>() *= 3;
      desired += twist;
      if (desired.head<3>().norm() > 0.12)
        desired.head<3>() *= 0.12 / desired.head<3>().norm();
      if (desired.tail<3>().norm() > 0.5)
        desired.tail<3>() *= 0.5 / desired.tail<3>().norm();
      Eigen::JacobiSVD<Jacobian> svd(expected_jacobian);
      const double ratio =
          std::min(1.0, svd.singularValues().minCoeff() / 0.05);
      const double damping = 0.002 + 0.002 * (1 - ratio) * (1 - ratio);
      const Vec expected_velocity =
          expected_jacobian.transpose() *
          (expected_jacobian * expected_jacobian.transpose() +
           damping * Jacobian::Identity())
              .ldlt()
              .solve(desired);
      require(
          (optimized.differentialIk(q, target, 1, twist) - expected_velocity)
                  .norm() < 1e-12,
          "combined differential IK changed numerical output");
      if (sample < 20) {
        const Vec nearby =
            (q + delta).cwiseMax(optimized.lower()).cwiseMin(optimized.upper());
        const Pose goal = optimized.fk(nearby);
        Vec solution;
        require(optimized.ik(goal, q, solution),
                "combined iterative IK failed on nearby reachable target");
        const Vec error = poseError(optimized.fk(solution), goal);
        require(error.head<3>().norm() < 1e-4 && error.tail<3>().norm() < 5e-4,
                "combined iterative IK violated convergence tolerance");
      }
    }
    std::cout
        << "PASS: timed interpolation, SO(3) derivatives, replacement, "
           "gravity validation, official gripper reduction, and tool kinematics/IK equivalence\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
