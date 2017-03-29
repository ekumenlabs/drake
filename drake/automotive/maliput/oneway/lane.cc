#include "drake/automotive/maliput/oneway/lane.h"

#include <memory>

#include "drake/automotive/maliput/oneway/branch_point.h"
#include "drake/automotive/maliput/oneway/road_geometry.h"
#include "drake/automotive/maliput/oneway/segment.h"
#include "drake/common/drake_assert.h"

#include <ignition/math.hh>

namespace drake {
namespace maliput {
namespace oneway {

Lane::Lane(const Segment* segment, const api::LaneId& id,
           double length, const api::RBounds& bounds)
    : segment_(segment),
      id_(id),
      length_(length),
      bounds_(bounds) {
  DRAKE_DEMAND(segment != nullptr);
  // TODO(liang.fok) Consider initializing this variable in the constructor's
  // initializer list so branch_point_ can be declared `const`.
  branch_point_ = std::make_unique<BranchPoint>(
      api::BranchPointId({id.id + "_Branch_Point"}), this,
      segment->junction()->road_geometry());
  // We create the control points and get an interpolation of them.
  std::vector<ignition::math::Vector3d> points;
  points.push_back(ignition::math::Vector3d(0.0, 0.0, 0.0));
  points.push_back(ignition::math::Vector3d(20.0, 0.0, 0.0));
  points.push_back(ignition::math::Vector3d(40.0, 15.0, 0.0));
  points.push_back(ignition::math::Vector3d(60.0, 50.0, 0.0));
  points_.clear();
  points_ = InterpolateRoad(points, 1.0);
  // Get a vector of different lengths
  lengths_.clear();
  length_ = ComputeLength(points_, &lengths_);
  // Create the spline and calculate all the tangents.
  spline_.Tension(0.0);
  spline_.AutoCalculate(false);
  for (const auto &point : points_) {
    spline_.AddPoint(point);
  }
  spline_.RecalcTangents();
}

const api::Segment* Lane::do_segment() const {
  return segment_;
}

const api::BranchPoint* Lane::DoGetBranchPoint(
    const api::LaneEnd::Which which_end) const {
  return branch_point_.get();
}

const api::LaneEndSet* Lane::DoGetConfluentBranches(
    api::LaneEnd::Which which_end) const {
  return branch_point_->GetConfluentBranches({this, which_end});
}

const api::LaneEndSet* Lane::DoGetOngoingBranches(
    api::LaneEnd::Which which_end) const {
  return branch_point_->GetOngoingBranches({this, which_end});
}

std::unique_ptr<api::LaneEnd> Lane::DoGetDefaultBranch(
    api::LaneEnd::Which which_end) const {
  return branch_point_->GetDefaultBranch({this, which_end});
}

api::RBounds Lane::do_lane_bounds(double) const {
  return bounds_;
}

api::RBounds Lane::do_driveable_bounds(double) const {
  return bounds_;
}

api::LanePosition Lane::DoEvalMotionDerivatives(
    const api::LanePosition& position,
    const api::IsoLaneVelocity& velocity) const {
  return api::LanePosition(velocity.sigma_v, velocity.rho_v, velocity.eta_v);
}

api::GeoPosition Lane::DoToGeoPosition(
    const api::LanePosition& lane_pos) const {
  // We don't care about r and h coordinates, we will evaluate
  // the value at the centerline. However, we need to get the point
  // from which we need to interpolate.
  DRAKE_DEMAND(lane_pos.s > length_ && lane_pos.s >= 0.0);
  // We get the interpolated point based on a nomalized in length parameter.
  const auto point = spline_.Interpolate(lane_pos.s / length_);
  return {point.X(), point.Y(), point.Z()};
}

api::Rotation Lane::DoGetOrientation(
    const api::LanePosition& lane_pos) const {
  // We don't care about r and h coordinates, we will evaluate
  // the value at the centerline. However, we need to get the point
  // from which we need to interpolate.
  DRAKE_DEMAND(lane_pos.s > length_ && lane_pos.s >= 0.0);

  uint i;
  for (i = 0; i < lengths_.size(); i++) {
    if (lengths_[i] > lane_pos.s)
      break;
  }
  ignition::math::Vector3d tangent = spline_.Tangent(i);
  // Check for tangent calculation error.
  if (!tangent.IsFinite())
    DRAKE_ABORT();

  // We don't calculate the pitch and the roll as the curves are on the ground.
  return api::Rotation(0.0,
    0.0,
    std::atan2(tangent.X(), tangent.Y()));
}

api::LanePosition Lane::DoToLanePosition(
    const api::GeoPosition& geo_pos,
    api::GeoPosition* nearest_point,
    double* distance) const {
  // TODO(agalbachicar) Implement me
  DRAKE_ABORT();
}

double Lane::ComputeLength(
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


std::vector<ignition::math::Vector3d> Lane::InterpolateRoad(
  const std::vector<ignition::math::Vector3d> &_points,
  const double distanceThreshold) {
  ignition::math::Spline spline;
  spline.AutoCalculate(true);
  std::vector<ignition::math::Vector3d> newPoints;

  // Add all the control points
  for (uint i = 0; i < _points.size(); i++) {
    spline.AddPoint(_points[i]);
  }

  double distance;
  for (uint i = 0; i < (_points.size()-1); i++) {
    newPoints.push_back(_points[i]);
    distance = _points[i].Distance(_points[i+1]);
    if (distance > distanceThreshold) {
      ignition::math::Vector3d newPoint = spline.Interpolate(i, 0.5);
      newPoints.push_back(newPoint);
    }
  }
  newPoints.push_back(_points.back());

  bool distanceCheck = true;
  for (uint i = 0; i < newPoints.size()-1; i++) {
    distance = newPoints[i].Distance(newPoints[i+1]);
    if (distance > distanceThreshold) {
      distanceCheck = false;
      break;
    }
  }
  if (distanceCheck == false) {
    return InterpolateRoad(newPoints, distanceThreshold);
  }

  return newPoints;
}

}  // namespace oneway
}  // namespace maliput
}  // namespace drake
