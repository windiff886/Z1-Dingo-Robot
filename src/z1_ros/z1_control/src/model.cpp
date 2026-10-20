#include "model_internal.hpp"
#include <Eigen/Cholesky>
#include <fstream>
#include <iomanip>
#include <openssl/evp.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
namespace z1 {
Model::Model(const ModelConfig &c) : impl_(new Impl) {
  auto &p = *impl_;
  pinocchio::urdf::buildModel(c.urdf, p.model);
  if (p.model.nq != 6 || p.model.nv != 6)
    throw std::runtime_error("model must contain exactly six scalar joints");
  for (int i = 1; i <= 6; ++i)
    if (p.model.names[i] != "joint" + std::to_string(i))
      throw std::runtime_error("model joint order must be joint1..joint6");
  if (!c.gravity.allFinite() || c.gravity.norm() > 20)
    throw std::runtime_error("invalid base-frame gravity");
  p.model.gravity.linear() = c.gravity;
  if (!p.model.existFrame(c.tool_frame))
    throw std::runtime_error("missing TCP frame");
  p.tool = p.model.getFrameId(c.tool_frame);
  p.data.reset(new pinocchio::Data(p.model));
  pinocchio::urdf::buildGeom(p.model, c.urdf, pinocchio::COLLISION, p.geometry);
  double translations = 0;
  for (auto &t : p.model.jointPlacements)
    translations += t.translation().norm();
  for (auto &g : p.geometry.geometryObjects) {
    g.geometry->computeLocalAABB();
    p.radius_bound =
        std::max(p.radius_bound,
                 translations + g.placement.translation().norm() +
                     g.geometry->aabb_center.norm() + g.geometry->aabb_radius);
  }
  std::set<std::string> covered;
  for (auto &g : p.geometry.geometryObjects)
    covered.insert(p.model.frames[g.parentFrame].name);
  for (auto name : {"link00", "link01", "link02", "link03", "link04", "link05",
                    "link06", c.tool_frame.c_str()})
    if (!covered.count(name))
      throw std::runtime_error(std::string("missing collision coverage: ") +
                               name);
  for (auto &pair : c.ignored_pairs)
    if (!covered.count(pair.first) || !covered.count(pair.second) ||
        pair.first == pair.second)
      throw std::runtime_error("invalid collision exclusion");
  for (size_t i = 0; i < p.geometry.geometryObjects.size(); ++i)
    for (size_t j = i + 1; j < p.geometry.geometryObjects.size(); ++j) {
      auto &a = p.geometry.geometryObjects[i];
      auto &b = p.geometry.geometryObjects[j];
      std::string an = p.model.frames[a.parentFrame].name,
                  bn = p.model.frames[b.parentFrame].name;
      bool skip = an == bn;
      for (auto &pair : c.ignored_pairs)
        skip |= (pair.first == an && pair.second == bn) ||
                (pair.first == bn && pair.second == an);
      if (!skip)
        p.geometry.addCollisionPair(pinocchio::CollisionPair(i, j));
    }
  if (p.geometry.collisionPairs.empty())
    throw std::runtime_error("no collision pairs enabled");
  p.boxes = c.fixed_boxes;
  std::set<std::string> box_names;
  for (auto &box : p.boxes)
    if (box.name.empty() || !box_names.insert(box.name).second ||
        covered.count(box.name) || !validPose(box.placement) ||
        !box.size.allFinite() || (box.size.array() <= 0).any())
      throw std::runtime_error("invalid fixed box");
  p.geometry_data.reset(new pinocchio::GeometryData(p.geometry));
  std::ifstream source(c.urdf, std::ios::binary);
  if (!source)
    throw std::runtime_error("cannot fingerprint model");
  std::ostringstream canonical;
  canonical.imbue(std::locale::classic());
  canonical << std::setprecision(17);
  canonical << source.rdbuf() << '\n'
            << c.tool_frame << '\n'
            << c.gravity.transpose() << '\n';
  for (const auto &pair : c.ignored_pairs)
    canonical << pair.first << '\n' << pair.second << '\n';
  for (const auto &box : c.fixed_boxes)
    canonical << box.name << '\n'
              << box.size.transpose() << '\n'
              << box.placement.matrix() << '\n';
  auto bytes = canonical.str();
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned length{};
  if (EVP_Digest(bytes.data(), bytes.size(), digest, &length, EVP_sha256(),
                 nullptr) != 1 ||
      length != 32)
    throw std::runtime_error("model fingerprint failed");
  std::ostringstream hex;
  for (unsigned i = 0; i < length; ++i)
    hex << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<unsigned>(digest[i]);
  p.revision = hex.str();
}
Model::~Model() = default;
Vec Model::lower() const { return impl_->model.lowerPositionLimit; }
Vec Model::upper() const { return impl_->model.upperPositionLimit; }
Vec Model::effort() const { return impl_->model.effortLimit; }
Vec Model::velocityLimit() const { return impl_->model.velocityLimit; }
double Model::radiusBound() const { return impl_->radius_bound; }
std::string Model::revision() const { return impl_->revision; }
bool Model::withinLimits(const Vec &q, double m) const {
  return q.allFinite() && (q.array() >= lower().array() + m - 1e-10).all() &&
         (q.array() <= upper().array() - m + 1e-10).all();
}
Pose Model::fk(const Vec &q) {
  auto &p = *impl_;
  pinocchio::framesForwardKinematics(p.model, *p.data, q);
  Pose out;
  out.matrix() = p.data->oMf[p.tool].toHomogeneousMatrix();
  return out;
}
Jacobian Model::jacobian(const Vec &q) {
  auto &p = *impl_;
  Jacobian J = Jacobian::Zero();
  pinocchio::computeJointJacobians(p.model, *p.data, q);
  pinocchio::updateFramePlacements(p.model, *p.data);
  pinocchio::getFrameJacobian(p.model, *p.data, p.tool,
                              pinocchio::LOCAL_WORLD_ALIGNED, J);
  return J;
}
Vec poseError(const Pose &a, const Pose &d) {
  Vec e;
  e.head<3>() = d.translation() - a.translation();
  e.tail<3>() =
      pinocchio::log3(Eigen::Matrix3d(d.linear() * a.linear().transpose()));
  return e;
}
Vec Model::differentialIk(const Vec &q, const Pose &t, double speed) {
  Vec e = poseError(fk(q), t);
  e.head<3>() *= 4;
  e.tail<3>() *= 3;
  if (e.head<3>().norm() > speed * 0.12)
    e.head<3>() *= speed * 0.12 / e.head<3>().norm();
  if (e.tail<3>().norm() > speed * 0.5)
    e.tail<3>() *= speed * 0.5 / e.tail<3>().norm();
  Jacobian J = jacobian(q);
  return J.transpose() *
         (J * J.transpose() + 0.002 * Jacobian::Identity()).ldlt().solve(e);
}
bool Model::ik(const Pose &target, const Vec &seed, Vec &result) {
  if (!validPose(target) || !withinLimits(seed))
    return false;
  Vec q = seed;
  for (int i = 0; i < 300; ++i) {
    Vec e = poseError(fk(q), target);
    if (e.head<3>().norm() < 1e-4 && e.tail<3>().norm() < 5e-4) {
      result = q;
      return true;
    }
    Jacobian J = jacobian(q);
    Vec step =
        J.transpose() *
        (J * J.transpose() + 0.0002 * Jacobian::Identity()).ldlt().solve(e);
    if (step.norm() > 0.15)
      step *= 0.15 / step.norm();
    q = (q + step).cwiseMax(lower()).cwiseMin(upper());
  }
  return false;
}
} // namespace z1
