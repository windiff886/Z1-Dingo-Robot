#include "z1_control/controller.hpp"
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <coal/distance.h>
#include <coal/shape/geometric_shapes.h>
#include <iostream>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
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

// Independent, unpruned Coal queries validate the optimized query result,
// including negative signed distances and fixed world geometry.
struct BruteGeometry {
  pinocchio::Model model;
  pinocchio::GeometryModel geometry;
  std::unique_ptr<pinocchio::Data> data;
  std::unique_ptr<pinocchio::GeometryData> geometry_data;
  ModelConfig config;
  BruteGeometry(const ModelConfig &c) : config(c) {
    pinocchio::urdf::buildModel(c.urdf, model);
    pinocchio::urdf::buildGeom(model, c.urdf, pinocchio::COLLISION, geometry);
    data.reset(new pinocchio::Data(model));
    geometry_data.reset(new pinocchio::GeometryData(geometry));
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
  double distance(const Vec &q) {
    pinocchio::updateGeometryPlacements(model, *data, geometry, *geometry_data,
                                        q);
    double best = std::numeric_limits<double>::infinity();
    auto query = [&](const auto *a, const auto &ap, const auto *b,
                     const auto &bp) {
      coal::DistanceRequest request;
      request.enable_signed_distance = true;
      coal::DistanceResult result;
      best = std::min(
          best,
          coal::distance(a, coal::Transform3s(ap.rotation(), ap.translation()),
                         b, coal::Transform3s(bp.rotation(), bp.translation()),
                         request, result));
    };
    for (size_t i = 0; i < geometry.geometryObjects.size(); ++i)
      for (size_t j = i + 1; j < geometry.geometryObjects.size(); ++j) {
        const auto &a = geometry.geometryObjects[i],
                   &b = geometry.geometryObjects[j];
        const auto an = model.frames[a.parentFrame].name,
                   bn = model.frames[b.parentFrame].name;
        bool skip = an == bn;
        for (const auto &pair : config.ignored_pairs)
          skip |= (pair.first == an && pair.second == bn) ||
                  (pair.second == an && pair.first == bn);
        if (!skip)
          query(a.geometry.get(), geometry_data->oMg[i], b.geometry.get(),
                geometry_data->oMg[j]);
      }
    for (const auto &box : config.fixed_boxes) {
      coal::Box shape(box.size.x(), box.size.y(), box.size.z());
      pinocchio::SE3 transform(box.placement.linear(),
                               box.placement.translation());
      for (size_t i = 0; i < geometry.geometryObjects.size(); ++i)
        query(geometry.geometryObjects[i].geometry.get(), geometry_data->oMg[i],
              &shape, transform);
    }
    return best;
  }
};
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

    auto config = defaultModelConfig(argv[1]);
    FixedBox box;
    box.name = "test_world";
    box.size = Eigen::Vector3d(0.08, 0.1, 0.12);
    box.placement.translation() << -0.1, 0.1, 0.3;
    config.fixed_boxes.push_back(box);
    Model optimized(config);
    BruteGeometry brute(config);
    std::mt19937 random(8675309);
    std::uniform_real_distribution<double> uniform(0, 1);
    bool penetration = false;
    std::string_view retained_pair;
    std::string retained_name;
    for (int sample = 0; sample < 250; ++sample) {
      Vec q, uncertainty, delta;
      for (int j = 0; j < 6; ++j) {
        q[j] = optimized.lower()[j] +
               uniform(random) * (optimized.upper()[j] - optimized.lower()[j]);
        uncertainty[j] = 0.01 + 0.02 * uniform(random);
        delta[j] = (2 * uniform(random) - 1) * uncertainty[j];
      }
      const double expected = brute.distance(q);
      penetration |= expected < 0;
      const auto light = optimized.clearanceSample(q);
      const auto owning = optimized.clearance(q);
      require(std::abs(light.distance - expected) < 1e-8,
              "broadphase/cached distance differs from brute force");
      require(light.distance == owning.distance && light.pair == owning.pair,
              "lightweight clearance changed public query semantics");
      const auto light_bound = optimized.clearanceSample(q, uncertainty);
      const auto owning_bound = optimized.clearance(q, uncertainty);
      require(light_bound.distance == owning_bound.distance &&
                  light_bound.pair == owning_bound.pair,
              "lightweight displacement bound differs from owning result");
      const double lower = light_bound.distance;
      require(lower <= brute.distance(q + delta) + 1e-8,
              "pair-specific displacement bound is not conservative");
      if (sample == 0) {
        retained_pair = light.pair;
        retained_name = light.pair;
      }
      require(retained_pair == retained_name,
              "clearance pair view invalidated by later queries");

      Jacobian expected_jacobian;
      const Pose expected_pose = brute.kinematics(q, expected_jacobian);
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
    require(penetration, "signed penetration cases not covered");
    // Exercise the same persistent pair solvers through a continuous path,
    // then revisit its first configuration. Request-local initialization must
    // keep the signed result independent of the solver's previous queries.
    const Vec center = (optimized.lower() + optimized.upper()) * 0.5;
    const Vec amplitude = (optimized.upper() - optimized.lower()) * 0.35;
    Vec first_q;
    for (int sample = 0; sample < 300; ++sample) {
      Vec q;
      for (int j = 0; j < 6; ++j)
        q[j] = center[j] + amplitude[j] * std::sin(sample * 0.03 + j * 0.7);
      if (sample == 0)
        first_q = q;
      const double expected = brute.distance(q);
      const auto result = optimized.clearanceSample(q);
      require(std::abs(result.distance - expected) < 1e-8,
              "persistent pair solver differs along continuous path");
      require(optimized.clearanceSample(q).distance == result.distance,
              "same-configuration geometry cache changed distance");
    }
    require(std::abs(optimized.clearanceSample(first_q).distance -
                     brute.distance(first_q)) < 1e-8,
            "persistent pair solver retained query-dependent distance");
    std::cout
        << "PASS: timed interpolation, SO(3) derivatives, replacement, "
           "broadphase/owning/view equivalence, displacement bounds, "
           "persistent pair solvers, and tool kinematics/IK equivalence\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
