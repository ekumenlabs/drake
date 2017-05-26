#include "drake/automotive/maliput/rndf/junction.h"

namespace drake {
namespace maliput {
namespace rndf {

const api::RoadGeometry* Junction::do_road_geometry() const {
  return road_geometry_;
}

Segment* Junction::NewSegment(api::SegmentId id) {
  segments_.push_back(std::make_unique<Segment>(id, this));
  return segments_.back().get();
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
