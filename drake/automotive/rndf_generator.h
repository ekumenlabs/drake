#pragma once

#include <cmath>
#include <memory>
#include <vector>
#include <tuple>
#include <sstream>
#include <string>
#include <map>
#include <algorithm>

#include "ignition/math/Vector3.hh"
#include "ignition/math/SphericalCoordinates.hh"
#include "ignition/rndf/RNDF.hh"
#include "ignition/rndf/Segment.hh"
#include "ignition/rndf/Lane.hh"
#include "ignition/rndf/Waypoint.hh"
#include "ignition/rndf/UniqueId.hh"
#include "ignition/rndf/Exit.hh"

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/rndf/builder.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

struct RNDFRoadCharacteristics {
  /// Constructor for using default road geometries.
  RNDFRoadCharacteristics() = default;

  /// Constructor for custom road geometries.
  RNDFRoadCharacteristics(const double lw, const double dw)
      : lane_width(lw), driveable_width(dw) {}

  const double lane_width{4.};
  const double driveable_width{8.};

  const maliput::api::RBounds lane_bounds{-lane_width / 2., lane_width / 2.};
  const maliput::api::RBounds driveable_bounds{-driveable_width / 2.,
                                               driveable_width / 2.};
};

class RNDFTBuilder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RNDFTBuilder)

  /// Constructor for the example.  The user supplies @p rc, a
  /// RoadCharacteristics structure that aggregates the road boundary data.
  explicit RNDFTBuilder(const RNDFRoadCharacteristics& rc) : rc_(rc) {}

  /// Constructor for the example, using default RoadCharacteristics settings.
  RNDFTBuilder() : RNDFTBuilder(RNDFRoadCharacteristics{}) {}

  std::unique_ptr<const maliput::api::RoadGeometry> Build(
    const std::string &file_name);

 private:

  ignition::math::Vector3d ToGlobalCoordinates(
    const ignition::math::Vector3d &origin,
    const ignition::math::SphericalCoordinates &spherical_position) const;

  void BuildSegments(
    maliput::rndf::Builder &builder,
    const ignition::math::Vector3d &origin,
    std::vector<ignition::rndf::Segment> &segments) const;

  void BuildConnections(
    maliput::rndf::Builder &builder,
    std::vector<ignition::rndf::Segment> &segments) const;

  /// Tolerances for monolane's Builder.
  const double linear_tolerance_  = 0.01;
  const double angular_tolerance_ = 2.0 * M_PI;
  //const double angular_tolerance_ = 0.01 * M_PI;

  const RNDFRoadCharacteristics rc_;
};

}  // namespace automotive
}  // namespace drake