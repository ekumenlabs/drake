#include "drake/automotive/maliput/oneway/segment.h"

#include <string>
#include <utility>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/oneway/junction.h"
#include "drake/automotive/maliput/oneway/lane.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace oneway {

Segment::Segment(Junction* junction,
    double length,
    double lane_width)
    : id_({"Oneway_Segment_ID"}),
      junction_(junction) {
  // To better understand the semantics of the variables defined in this method,
  // see the class description.

  const api::RBounds bounds({-lane_width / 2, lane_width / 2});

  auto lane = std::make_unique<Lane>(
      this,
      api::LaneId({"Oneway_Lane_ID"}),
      length,
      bounds);
  lane_ = std::move(lane);
}

const api::Junction* Segment::do_junction() const {
  return junction_;
}

const api::Lane* Segment::do_lane(int index) const {
  DRAKE_DEMAND(index == 0);
  return lane_.get();
}

}  // namespace oneway
}  // namespace maliput
}  // namespace drake
