#include "drake/automotive/maliput/oneway/junction.h"

#include "drake/automotive/maliput/oneway/road_geometry.h"
#include "drake/automotive/maliput/oneway/segment.h"

namespace drake {
namespace maliput {
namespace oneway {

Junction::Junction(RoadGeometry* road_geometry,
    double length,
    double lane_width)
  : id_({"Dragway Junction"}),
    road_geometry_(road_geometry),
    segment_(this, length, lane_width) {
  DRAKE_DEMAND(road_geometry != nullptr);
}

const api::RoadGeometry* Junction::do_road_geometry() const {
  return road_geometry_;
}

}  // namespace oneway
}  // namespace maliput
}  // namespace drake
