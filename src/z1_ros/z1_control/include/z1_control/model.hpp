#pragma once
#include "types.hpp"
#include <memory>
#include <string_view>
namespace z1 {
struct Clearance {
  double distance;
  std::string pair;
};
struct ClearanceSample {
  double distance;
  std::string_view pair;
};
class Model {
public:
  explicit Model(const ModelConfig &);
  ~Model();
  Model(const Model &) = delete;
  Model &operator=(const Model &) = delete;
  Vec lower() const;
  Vec upper() const;
  Vec effort() const;
  Vec velocityLimit() const;
  std::string revision() const;
  bool withinLimits(const Vec &, double margin = 0) const;
  Pose fk(const Vec &);
  Jacobian jacobian(const Vec &);
  Vec inverseDynamics(const Vec &, const Vec &, const Vec &);
  Vec forwardDynamics(const Vec &, const Vec &, const Vec &);
  bool ik(const Pose &, const Vec &seed, Vec &result,
          double seed_tolerance = 0.001, std::string *failure = nullptr);
  Vec differentialIk(const Vec &, const Pose &target, double speed,
                     const Vec &twist = Vec::Zero());
  Clearance clearance(const Vec &);
  // Lower distance bound over independent joint displacements about q.
  Clearance clearance(const Vec &q, const Vec &joint_displacement);
  // The pair view remains valid for this Model's lifetime.
  ClearanceSample clearanceSample(const Vec &q,
                                  const Vec &joint_displacement = Vec::Zero());

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
Vec poseError(const Pose &actual, const Pose &desired);
} // namespace z1
