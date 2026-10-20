#include "model_internal.hpp"
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
namespace z1 {
Vec Model::inverseDynamics(const Vec &q, const Vec &v, const Vec &a) {
  auto &p = *impl_;
  return pinocchio::rnea(p.model, *p.data, q, v, a);
}
Vec Model::forwardDynamics(const Vec &q, const Vec &v, const Vec &t) {
  auto &p = *impl_;
  return pinocchio::aba(p.model, *p.data, q, v, t);
}
} // namespace z1
