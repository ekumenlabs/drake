#include "drake/automotive/maliput/multilane/segment.h"

#include <iostream>

namespace drake {
namespace maliput {
namespace multilane {

const api::Junction* Segment::do_junction() const {
  return junction_;
}

Lane* Segment::NewLane(const api::LaneId& id) {
  DRAKE_DEMAND(static_cast<int>(lanes_.size()) < num_lanes_);
  // Computes driveable bounds and lane bounds for the lane.
  const double r_offset = r0_ + static_cast<double>(lanes_.size()) * r_spacing_;
  const api::RBounds lane_driveable_bounds(
      (r0_ - lane_width_ / 2.0) - r_offset,
      (r0_ - lane_width_ / 2.0 + width_) - r_offset);
  const api::RBounds lane_bounds(-lane_width_ / 2.0, lane_width_ / 2.0);
  DRAKE_DEMAND(road_curve_->IsValid(lane_driveable_bounds, elevation_bounds_,
                                    r_offset) == true);
  // Creates the lane and pushes it to the lanes vector.
  std::unique_ptr<Lane> lane_ = std::make_unique<Lane>(id, this, lane_bounds,
      lane_driveable_bounds, elevation_bounds_, road_curve_.get(), r_offset);
  lanes_.push_back(std::move(lane_));
  return lanes_.back().get();
}

const api::Lane* Segment::do_lane(int index) const {
  DRAKE_DEMAND(index >= 0 && index < static_cast<int>(lanes_.size()));
  return lanes_[index].get();
}


}  // namespace multilane
}  // namespace maliput
}  // namespace drake
