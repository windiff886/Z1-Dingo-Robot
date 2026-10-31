#pragma once
#include "types.hpp"
#include <memory>
namespace z1 {
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

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
Vec poseError(const Pose &actual, const Pose &desired);
} // namespace z1
