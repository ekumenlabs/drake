#include "drake/automotive/maliput/oneway/road_geometry.h"

#include <memory>

#include "drake/automotive/maliput/oneway/branch_point.h"
#include "drake/automotive/maliput/oneway/junction.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

using std::make_unique;

namespace drake {
namespace maliput {
namespace oneway {

RoadGeometry::RoadGeometry(const api::RoadGeometryId& id,
               double length,
               double lane_width,
               double linear_tolerance,
               double angular_tolerance)
  : id_(id),
    linear_tolerance_(linear_tolerance),
    angular_tolerance_(angular_tolerance),
    junction_(this, length, lane_width) {
  DRAKE_DEMAND(length > 0);
  DRAKE_DEMAND(lane_width > 0);
  DRAKE_DEMAND(linear_tolerance >= 0);
  DRAKE_DEMAND(angular_tolerance >= 0);
}

const api::Junction* RoadGeometry::do_junction(int index) const {
  DRAKE_DEMAND(index < num_junctions());
  return &junction_;
}

int RoadGeometry::do_num_branch_points() const {
  // There is only one BranchPoint per lane. Thus, return the number of lanes.
  return junction_.segment(0)->num_lanes();
}

const api::BranchPoint* RoadGeometry::do_branch_point(int index) const {
  DRAKE_DEMAND(index < num_branch_points());
  // The same BranchPoint is at the start versus end of a Lane, thus it doesn't
  // matter whether the start or finish BranchPoint is returned.
  return junction_.segment(0)->lane(index)->GetBranchPoint(
      api::LaneEnd::kStart);
}

bool RoadGeometry::IsGeoPositionOnOneway(const api::GeoPosition& geo_pos)
    const {
  const api::Lane* lane = junction_.segment(0)->lane(0);
  DRAKE_ASSERT(lane != nullptr);
  const double length = lane->length();
  const api::RBounds lane_driveable_bounds = lane->driveable_bounds(0 /* s */);
  const double min_y = lane_driveable_bounds.r_min;
  const double max_y = lane_driveable_bounds.r_max;

  if (geo_pos.x < 0 || geo_pos.x > length ||
      geo_pos.y > max_y || geo_pos.y < min_y) {
    drake::log()->trace(
        "dragway::RoadGeometry::IsGeoPositionOnDragway(): The provided geo_pos "
        "({}, {}) is not on the dragway (length = {}, min_y = {}, max_y = {}).",
        geo_pos.x, geo_pos.y, length, min_y, max_y);
    return false;
  } else {
    return true;
  }
}

api::RoadPosition RoadGeometry::DoToRoadPosition(
    const api::GeoPosition& geo_pos,
    const api::RoadPosition* hint,
    api::GeoPosition* nearest_position,
    double* distance) const {
  api::LanePosition result_lane_position;
  result_lane_position.s = geo_pos.x;
  result_lane_position.h = geo_pos.z;
  result_lane_position.r = geo_pos.y;
  if (IsGeoPositionOnOneway(geo_pos)) {
    const api::Lane* lane = junction_.segment(0)->lane(0);
    DRAKE_ASSERT(lane != nullptr);
    if (nearest_position != nullptr) {
      nearest_position->x = geo_pos.x;
      nearest_position->y = geo_pos.y;
      nearest_position->z = geo_pos.z;
    }
    if (distance != nullptr) {
      *distance = 0;
    }
    return api::RoadPosition(lane, result_lane_position);
  } else {
    // TODO(liang.fok): Implement this!
    throw std::runtime_error(
        "dragway::RoadGeometry::DoToRoadPosition: The ability to determine the "
        "road position of a GeoPosition that's not on top of the dragway has "
        "yet to be implemented.");
  }
}

}  // namespace oneway
}  // namespace maliput
}  // namespace drake
