#pragma once

#include <cmath>
#include <memory>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/rndf/builder.h"

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

  /// Implements the onramp example.
  std::unique_ptr<const maliput::api::RoadGeometry> Build();

 private:
  /// Tolerances for monolane's Builder.
  const double linear_tolerance_  = 0.01;
  const double angular_tolerance_ = 2.0 * M_PI;//0.01 * M_PI;

  const RNDFRoadCharacteristics rc_;
};

class Waypoint {
 public:
  Waypoint (const uint segment_id,
    const uint lane_id,
    const uint id,
    const double latitude,
    const double longitude) :
      segment_id_(segment_id),
      lane_id_(lane_id),
      id_(id),
      latitude_(latitude),
      longitude_(longitude) {}

  Waypoint (const std::string &wp_str) {}

  std::string Id() const {
    return std::to_string(segment_id_) + "_" +
      std::to_string(lane_id) + "_" +
      std::to_string(id_);
  }

  std::pair<double, double> ToGlobalCoordinates(
    const double latitude_origin,
    const double longitude_origin) const {
    const auto origin =
      BuildSphericalCoordinates(latitude_origin, longitude_origin);
    const auto position = origin.positionTransform(
      BuildSphericalCoordinates(latitude_, longitude_),
      ignition::math::SphericalCoordinates::SPHERICAL,
      ignition::math::SphericalCoordinates::GLOBAL);
    return std::pair<double, double> (position.X(), position.Y());
  }

 private:
  ignition::math::SphericalCoordinates BuildSphericalCoordinates(
    const double latitude, const double longitude) {
    return ignition::math::SphericalCoordinates origin(
      ignition::math::SphericalCoordinates::EARTH_WGS84,
      ignition::math::Angle(latitude / 180.0 * M_PI),
      ignition::math::Angle(longitude / 180.0 * M_PI),
      0.0,
      ignition::math::Angle(0.0));
  }

  uint segment_id_;
  uint lane_id_;
  uint id_;
  double latitude_;
  double longitude_;
};

}  // namespace automotive
}  // namespace drake