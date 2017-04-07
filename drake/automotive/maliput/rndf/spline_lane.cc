#include "drake/automotive/maliput/rndf/spline_lane.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "drake/common/drake_assert.h"

#include <ignition/math.hh>

namespace drake {
namespace maliput {
namespace rndf {


SplineLane::SplineLane(const api::LaneId& id, const api::Segment* segment,
                      const std::vector<Point2> &control_points,
                      const api::RBounds& lane_bounds,
                      const api::RBounds& driveable_bounds,
                      const CubicPolynomial& elevation,
                      const CubicPolynomial& superelevation)
    : Lane(id,
          segment,
          lane_bounds, driveable_bounds,
          ComputeLength(Points22IgnitionVector3ds(control_points), nullptr),
          elevation,
          superelevation),
      control_points_(Points22IgnitionVector3ds(control_points)) {

  // Create the spline and calculate all the tangents.
  spline_.Tension(0.0);
  spline_.AutoCalculate(false);
  for (const auto &point : control_points_) {
    spline_.AddPoint(point);
  }
  spline_.RecalcTangents();
}



api::LanePosition SplineLane::DoToLanePosition(const api::GeoPosition&,
                                            api::GeoPosition*,
                                            double*) const {
  DRAKE_ABORT();  // TODO(maddog@tri.global) Implement me.
}

V2 SplineLane::xy_of_p(const double p) const {
  const auto point = spline_.Interpolate(p);
  return {point.X(), point.Y()};
}

V2 SplineLane::xy_dot_of_p(const double p) const {
  double p1 = module_p(p);
  double p2 = module_p(p + 0.001);

  const auto& point_p1 = spline_.Interpolate(p1);
  const auto& point_p2 = spline_.Interpolate(p2);
  const auto tangent = (point_p2 - point_p1) * 0.5;
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

Point2 SplineLane::IgnitionVector3d2Point2(
  const ignition::math::Vector3d &point) {
  return Point2{point.X(), point.Y()};
}
ignition::math::Vector3d Point22IgnitionVector3d(
  const Point2 &point) {
  return ignition::math::Vector3d(point.x(), point.y(), 0.0);
}

std::vector<ignition::math::Vector3d> SplineLane::Points22IgnitionVector3ds(
  const std::vector<Point2> &points) {
  std::vector<ignition::math::Vector3d> ignition_points;
  for(const auto& point : points) {
    ignition_points.push_back(
      ignition::math::Vector3d(point.x(), point.y(), 0.0));
  }
  return ignition_points;
}

std::vector<Point2> SplineLane::IgnitionVector3ds2Points(
  const std::vector<ignition::math::Vector3d> &points) {
  std::vector<Point2> ignition_points;
  for(const auto& point : points) {
    ignition_points.push_back(
      Point2{point.X(), point.Y()});
  }
  return ignition_points;
}

double SplineLane::ComputeLength(
  const std::vector<ignition::math::Vector3d> &points,
  std::vector<double> *lengths) {
  double length = 0.0;
  if (lengths == nullptr) {
    for (uint i = 0; i < points.size() - 1; i++) {
      length += (points[i].Distance(points[i+1]));
    }
  }
  else {
    for (uint i = 0; i < points.size() - 1; i++) {
      length += (points[i].Distance(points[i+1]));
      lengths->push_back(length);
    }
  }
  return length;
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake