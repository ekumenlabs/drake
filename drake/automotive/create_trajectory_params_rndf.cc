#include "drake/automotive/create_trajectory_params_rndf.h"

namespace drake {
namespace automotive {

std::tuple<Curve2<double>, double, double> CreateTrajectoryParamsForRndf(
    const maliput::rndf::RoadGeometry& road_geometry,
    const std::vector<std::string> &lanes,
    double speed, double start_time) {
  std::vector<Curve2<double>::Point2> waypoints;

  const double step = 0.1;

  for (const auto& id : lanes) {
    const std::string segment_id = "s:" + id;
    const std::string lane_id = "l:" + id;

    for (int i = 0; i < road_geometry.num_junctions(); i++) {
      const auto junction = road_geometry.junction(i);

      for (int j = 0; j < junction->num_segments(); j++) {
        const auto segment = junction->segment(j);
        if (segment->id().id != segment_id) {
          continue;
        }

        const drake::maliput::rndf::Lane *lane =
          dynamic_cast<const drake::maliput::rndf::Lane *>(segment->lane(0));
        DRAKE_DEMAND(lane);

        double l = 0.0;
        while (l < lane->length()) {
          const auto geo_position = lane->ToGeoPosition(
            maliput::api::LanePosition(l, 0.0, 0.0));
          waypoints.push_back({geo_position.x, geo_position.y});
          l += step;
        }

        break;
      }
    }
  }

  Curve2<double> curve(waypoints);
  return std::make_tuple(curve, speed, start_time);
}

}  // namespace automotive
}  // namespace drake
