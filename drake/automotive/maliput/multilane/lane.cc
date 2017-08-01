#include "drake/automotive/maliput/multilane/lane.h"

#include <iostream>

#include "drake/automotive/maliput/multilane/branch_point.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace multilane {

const api::Segment* Lane::do_segment() const { return segment_; }

const api::BranchPoint* Lane::DoGetBranchPoint(
    const api::LaneEnd::Which which_end) const {
  switch (which_end) {
    case api::LaneEnd::kStart:  { return start_bp_; }
    case api::LaneEnd::kFinish: { return end_bp_; }
  }
  DRAKE_ABORT();
}

const api::LaneEndSet* Lane::DoGetConfluentBranches(
    api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetConfluentBranches({this, which_end});
}

const api::LaneEndSet* Lane::DoGetOngoingBranches(
    api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetOngoingBranches({this, which_end});
}

std::unique_ptr<api::LaneEnd> Lane::DoGetDefaultBranch(
    api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetDefaultBranch({this, which_end});
}

double Lane::do_length() const {
  const Segment* segment = dynamic_cast<const Segment*>(segment_);
  DRAKE_THROW_UNLESS(segment != nullptr);
  return segment->TrajectoryLength();
}


api::GeoPosition Lane::DoToGeoPosition(
    const api::LanePosition& lane_pos) const {
  const Segment* segment = dynamic_cast<const Segment*>(segment_);
  DRAKE_THROW_UNLESS(segment != nullptr);
  return segment->DoToGeoPosition(lane_pos);
}


api::Rotation Lane::DoGetOrientation(const api::LanePosition& lane_pos) const {
  const Segment* segment = dynamic_cast<const Segment*>(segment_);
  DRAKE_THROW_UNLESS(segment != nullptr);
  return segment->DoGetOrientation(lane_pos);
}


api::LanePosition Lane::DoEvalMotionDerivatives(
    const api::LanePosition& position,
    const api::IsoLaneVelocity& velocity) const {
  const Segment* segment = dynamic_cast<const Segment*>(segment_);
  DRAKE_THROW_UNLESS(segment != nullptr);
  return segment->DoEvalMotionDerivatives(position, velocity);
}

api::LanePosition Lane::DoToLanePosition(const api::GeoPosition& geo_position,
                                         api::GeoPosition* nearest_position,
                                         double* distance) const {
  const Segment* segment = dynamic_cast<const Segment*>(segment_);
  DRAKE_THROW_UNLESS(segment != nullptr);
  const api::LanePosition lane_position = segment->DoToLanePosition(
      geo_position, driveable_bounds_, elevation_bounds_);
  const api::GeoPosition nearest = ToGeoPosition(lane_position);
  if (nearest_position != nullptr) {
    *nearest_position = nearest;
  }
  if (distance != nullptr) {
    *distance = (nearest.xyz() - geo_position.xyz()).norm();
  }
  return lane_position;
}


}  // namespace multilane
}  // namespace maliput
}  // namespace drake
