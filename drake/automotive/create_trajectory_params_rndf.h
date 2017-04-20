#pragma once

#include <algorithm>
#include <vector>

#include "drake/automotive/curve2.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"

namespace drake {
namespace automotive {

std::tuple<Curve2<double>, double, double> CreateTrajectoryParamsForRndf(
  const maliput::rndf::RoadGeometry& road_geometry,
  const std::vector<std::string> &lanes,
  double speed, double start_time);

}  // automotive
}  // drake
