#include "drake/automotive/maliput/rndf/segment.h"

#include <utility>

#include "drake/automotive/maliput/rndf/spline_lane.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace rndf {

const api::Junction* Segment::do_junction() const {
  return junction_;
}

SplineLane* Segment::NewSplineLane(const api::LaneId& id,
                      const std::vector<std::tuple<ignition::math::Vector3d,
                        ignition::math::Vector3d>> &control_points,
                      const api::RBounds& lane_bounds,
                      const api::RBounds& driveable_bounds) {
  std::unique_ptr<SplineLane> lane = std::make_unique<SplineLane>(
      id, this,
      control_points,
      lane_bounds,
      driveable_bounds,
      lanes_.size());
  SplineLane* spline_lane = lane.get();
  lanes_.push_back(std::move(lane));
  return spline_lane;
}


const api::Lane* Segment::do_lane(int index) const {
  DRAKE_THROW_UNLESS(index >= 0 && index < static_cast<int>(lanes_.size()));
  return lanes_[index].get();
}


}  // namespace rndf
}  // namespace maliput
}  // namespace drake
