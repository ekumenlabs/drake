#include "drake/automotive/maliput/multilane/junction.h"

#include <utility>

namespace drake {
namespace maliput {
namespace multilane {

const api::RoadGeometry* Junction::do_road_geometry() const {
  return road_geometry_;
}


Segment* Junction::NewSegment(api::SegmentId id,
                              std::unique_ptr<SegmentGeometry> geometry) {
  segments_.push_back(std::make_unique<Segment>(id, this, std::move(geometry)));
  return segments_.back().get();
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
