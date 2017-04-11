#include "drake/automotive/maliput/rndf/spline_lane.h"

#include <algorithm>
#include <tuple>
#include <cmath>
#include <limits>

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace rndf {


SplineLane::SplineLane(const api::LaneId& id, const api::Segment* segment,
  const std::vector<std::tuple<ignition::math::Vector3d,
    ignition::math::Vector3d>> &control_points,
  const api::RBounds& lane_bounds,
  const api::RBounds& driveable_bounds,
  const CubicPolynomial& elevation,
  const CubicPolynomial& superelevation):
    Lane(id,
      segment,
      lane_bounds,
      driveable_bounds,
      ComputeLength(control_points),
      elevation,
      superelevation) {
  spline_.Tension(0.0);
  spline_.AutoCalculate(true);
  std::cout << this->id().id << std::endl;
  for (const auto &point : control_points) {
    spline_.AddPoint(std::get<0>(point), std::get<1>(point));
  }
}

api::LanePosition SplineLane::DoToLanePosition(
  const api::GeoPosition&,
  api::GeoPosition*,
  double*) const {
    DRAKE_ABORT();
}

V2 SplineLane::xy_of_p(const double p) const {
  const auto point = spline_.Interpolate(p);
  return {point.X(), point.Y()};
}

V2 SplineLane::xy_dot_of_p(const double p) const {
  double _p = module_p(p);
  const auto& tangent = spline_.InterpolateTangent(_p);
  return {tangent.X(), tangent.Y()};
}

double SplineLane::heading_of_p(const double p) const {
  const auto tangent = xy_dot_of_p(p);
  return std::atan2(tangent.y(), tangent.x());
}

double SplineLane::heading_dot_of_p(const double p) const {
  const auto h_p = heading_of_p(p);
  const auto h_p_dp = heading_of_p(p + 0.001);
  return (h_p_dp - h_p) * 0.5;
}

double SplineLane::module_p(const double _p) const {
  double p = std::max(0.0, _p);
  p = std::min(1.0, p);
  return p;
}

double SplineLane::ComputeLength(
  const std::vector<std::tuple<ignition::math::Vector3d,
    ignition::math::Vector3d>> &points) {
  ignition::math::Spline spline;
  spline.Tension(0.0);
  spline.AutoCalculate(true);
  for (const auto &point : points) {
    spline.AddPoint(std::get<0>(point), std::get<1>(point));
  }
  return spline.ArcLength();
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake