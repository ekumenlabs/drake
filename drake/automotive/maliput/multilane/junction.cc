#include "drake/automotive/maliput/multilane/junction.h"

#include <utility>

namespace drake {
namespace maliput {
namespace multilane {

const api::RoadGeometry* Junction::do_road_geometry() const {
  return road_geometry_;
}


Segment* Junction::NewSegment(api::SegmentId id,
                              std::unique_ptr<RoadCurve> road_curve,
                              int num_lanes,
                              double r0, double r_spacing, double width,
                              const api::HBounds& elevation_bounds) {
  segments_.push_back(std::make_unique<Segment>(id, this,
                                                std::move(road_curve),
                                                num_lanes, r0, r_spacing, width,
                                                elevation_bounds));
  return segments_.back().get();
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
