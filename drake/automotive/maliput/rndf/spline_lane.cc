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
                      const std::vector<SplineLane::Point2> &control_points,
                      const double theta_i, const double theta_f,
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
      control_points_(Points22IgnitionVector3ds(control_points)),
      theta_i_(theta_i),
      theta_f_(theta_f) {

  ComputeLength(control_points_, &lengths_);
  std::vector<ignition::math::Vector3d> points;
  points = InterpolateRoad(control_points_, 0.1);
  // Create the spline and calculate all the tangents.
  spline_.Tension(0.0);
  spline_.AutoCalculate(false);
  for (const auto &point : points) {
    spline_.AddPoint(point);
  }
  spline_.RecalcTangents();

  theta_i_ += 0.0;
  theta_f_ += 0.0;
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

double SplineLane::heading_of_p(const double p) const {
  double s = std::min(0.0, p * p_scale_);
  s = std::max(s, p_scale_);

  uint i;
  for (i = 0; i < lengths_.size(); i++) {
    if (lengths_[i] > s)
      break;
  }
  ignition::math::Vector3d tangent = spline_.Tangent(i);
  // Check for tangent calculation error.
  if (!tangent.IsFinite())
    DRAKE_ABORT();
  //tangent = tangent.Normalize();
  // We don't calculate the pitch and the roll as the curves are on the ground.
  return std::atan2(tangent.Y(),tangent.X());
}


SplineLane::Point2 SplineLane::IgnitionVector3d2Point2(
  const ignition::math::Vector3d &point) {
  return SplineLane::Point2{point.X(), point.Y()};
}
ignition::math::Vector3d SplineLane::Point22IgnitionVector3d(
  const SplineLane::Point2 &point) {
  return ignition::math::Vector3d(point.x(), point.y(), 0.0);
}

std::vector<ignition::math::Vector3d> SplineLane::Points22IgnitionVector3ds(
  const std::vector<SplineLane::Point2> &points) {
  std::vector<ignition::math::Vector3d> ignition_points;
  for(const auto& point : points) {
    ignition_points.push_back(
      ignition::math::Vector3d(point.x(), point.y(), 0.0));
  }
  return ignition_points;
}

std::vector<SplineLane::Point2> SplineLane::IgnitionVector3ds2Points(
  const std::vector<ignition::math::Vector3d> &points) {
  std::vector<SplineLane::Point2> ignition_points;
  for(const auto& point : points) {
    ignition_points.push_back(
      SplineLane::Point2{point.X(), point.Y()});
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


std::vector<ignition::math::Vector3d> SplineLane::InterpolateRoad(
  const std::vector<ignition::math::Vector3d> &_points,
  const double distance_threshold) {
  ignition::math::Spline spline;
  spline.AutoCalculate(true);
  std::vector<ignition::math::Vector3d> new_points;

  // Add all the control points
  for (const auto &point : _points) {
    spline.AddPoint(point);
  }

  double distance;
  for (uint i = 0; i < (_points.size()-1); i++) {
    new_points.push_back(_points[i]);
    distance = _points[i].Distance(_points[i+1]);
    if (distance > distance_threshold) {
      ignition::math::Vector3d new_point = spline.Interpolate(i, 0.5);
      new_points.push_back(new_point);
    }
  }
  new_points.push_back(_points.back());

  bool distance_check = true;
  for (uint i = 0; i < new_points.size()-1; i++) {
    distance = new_points[i].Distance(new_points[i+1]);
    if (distance > distance_threshold) {
      distance_check = false;
      break;
    }
  }
  if (distance_check == false) {
    return InterpolateRoad(new_points, distance_threshold);
  }
  return new_points;
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake