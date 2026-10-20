#include "model_internal.hpp"
#include <coal/distance.h>
#include <coal/shape/geometric_shapes.h>
#include <limits>
#include <pinocchio/algorithm/geometry.hpp>
namespace z1 {
Clearance Model::clearance(const Vec &q) {
  auto &p = *impl_;
  pinocchio::updateGeometryPlacements(p.model, *p.data, p.geometry,
                                      *p.geometry_data, q);
  Clearance out{std::numeric_limits<double>::infinity(), ""};
  auto test = [&](const coal::CollisionGeometry *a, const pinocchio::SE3 &ap,
                  const coal::CollisionGeometry *b, const pinocchio::SE3 &bp,
                  const std::string &name) {
    coal::DistanceRequest request;
    request.enable_signed_distance = true;
    coal::DistanceResult result;
    coal::Transform3s at(ap.rotation(), ap.translation()),
        bt(bp.rotation(), bp.translation());
    double d = coal::distance(a, at, b, bt, request, result);
    if (!std::isfinite(d))
      throw std::runtime_error("nonfinite collision distance");
    if (d < out.distance)
      out = {d, name};
  };
  for (auto &pair : p.geometry.collisionPairs) {
    auto &a = p.geometry.geometryObjects[pair.first];
    auto &b = p.geometry.geometryObjects[pair.second];
    test(a.geometry.get(), p.geometry_data->oMg[pair.first], b.geometry.get(),
         p.geometry_data->oMg[pair.second], a.name + " / " + b.name);
  }
  for (auto &box : p.boxes) {
    coal::Box shape(box.size.x(), box.size.y(), box.size.z());
    pinocchio::SE3 transform(box.placement.linear(),
                             box.placement.translation());
    for (size_t i = 0; i < p.geometry.geometryObjects.size(); ++i) {
      auto &a = p.geometry.geometryObjects[i];
      test(a.geometry.get(), p.geometry_data->oMg[i], &shape, transform,
           a.name + " / " + box.name);
    }
  }
  return out;
}
} // namespace z1
