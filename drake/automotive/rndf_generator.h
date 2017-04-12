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

  Waypoint (const std::string &wp_str) {
    const auto parts = SeparateIdsLatLong(wp_str);
    DRAKE_DEMAND(parts.size() == 2);
    const auto ids = ParseIds(parts[0]);
    const auto latlong = ParseLatLong(parts[1]);

    segment_id_ = std::get<0>(ids);
    lane_id_ = std::get<1>(ids);
    id_ = std::get<2>(ids);

    latitude_ = std::get<0>(latlong);
    longitude_ = std::get<1>(latlong);
  }

  std::string IdStr() const {
    return std::to_string(segment_id_) + "_" +
      std::to_string(lane_id_) + "_" +
      std::to_string(id_);
  }

  bool MatchId(const std::string &id) const {
    return IdStr() == id;
  }

  uint Id() const {
    return id_;
  }
  uint LaneId() const {
    return lane_id_;
  }
  uint SegmentId() const {
    return segment_id_;
  }

  std::string LatLongStr() const {
    return std::to_string(latitude_) + "_" +
      std::to_string(longitude_);
  }

  ignition::math::Vector3d ToGlobalCoordinates(
    const double latitude_origin,
    const double longitude_origin) const {
    const auto origin =
      BuildSphericalCoordinates(latitude_origin, longitude_origin);
    const auto position_spherical =
      BuildSphericalCoordinates(latitude_, longitude_);
    const auto position_vector =  ignition::math::Vector3d(
      position_spherical.LatitudeReference().Radian(),
      position_spherical.LongitudeReference().Radian(),
      position_spherical.ElevationReference());
    return origin.PositionTransform(
      position_vector,
      ignition::math::SphericalCoordinates::SPHERICAL,
      ignition::math::SphericalCoordinates::GLOBAL);
    /*const auto position = origin.PositionTransform(
      position_vector,
      ignition::math::SphericalCoordinates::SPHERICAL,
      ignition::math::SphericalCoordinates::GLOBAL);
    return std::make_tuple(position.X(), position.Y()); */
  }

  static std::vector<std::string> Split(
    const std::string &statement,
    const std::string &separator) {
      std::istringstream simple_car_name_stream(statement);
      std::vector<std::string> parts;
      std::string part;
      while (std::getline(simple_car_name_stream, part, separator.front())) {
        parts.push_back(part);
      }
      return parts;
  }

 private:
  static ignition::math::SphericalCoordinates BuildSphericalCoordinates(
    const double latitude, const double longitude) {
    return ignition::math::SphericalCoordinates(
      ignition::math::SphericalCoordinates::EARTH_WGS84,
      ignition::math::Angle(latitude / 180.0 * M_PI),
      ignition::math::Angle(longitude / 180.0 * M_PI),
      0.0,
      ignition::math::Angle(0.0));
  }

  static std::tuple<uint, uint, uint> ParseIds(
    const std::string &id_str) {
      const std::vector<std::string> &ids_string = Split(id_str, ".");
      DRAKE_DEMAND(ids_string.size() == 3);
      return std::make_tuple(
        std::stoul(ids_string[0], nullptr, 0),
        std::stoul(ids_string[1], nullptr, 0),
        std::stoul(ids_string[2], nullptr, 0));
  }

  static std::tuple<double, double> ParseLatLong(
    const std::string &lat_long_str) {
      const std::vector<std::string> &ids_string = Split(lat_long_str, " ");
      DRAKE_DEMAND(ids_string.size() == 2);
      return std::make_tuple(
        std::stod(ids_string[0]),
        std::stod(ids_string[1]));
  }

  static std::vector<std::string> SeparateIdsLatLong(
    const std::string &statement) {
    return Split(statement, "\t");
  }

  uint segment_id_;
  uint lane_id_;
  uint id_;
  double latitude_;
  double longitude_;
};

class RNDFTBuilder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RNDFTBuilder)

  /// Constructor for the example.  The user supplies @p rc, a
  /// RoadCharacteristics structure that aggregates the road boundary data.
  explicit RNDFTBuilder(const RNDFRoadCharacteristics& rc) : rc_(rc) {}

  /// Constructor for the example, using default RoadCharacteristics settings.
  RNDFTBuilder() : RNDFTBuilder(RNDFRoadCharacteristics{}) {}

  /// Implements a T connection example.
  std::unique_ptr<const maliput::api::RoadGeometry> Build();

  /// Implements a custom RNDF map
  std::unique_ptr<const maliput::api::RoadGeometry> Build(
    const std::string &road_waypoints,
    const std::string &connections);

 private:
  void BuildWaypointMap(
    const std::string &rndf_description,
    std::map<uint, std::vector<Waypoint>> &waypoints_map);

  void BuildSegment(
    maliput::rndf::Builder &builder,
    const uint segment_id,
    const std::vector<Waypoint> &waypoints,
    const double latitude,
    const double longitude);

  void BuildConnection(
    maliput::rndf::Builder &builder,
    const Waypoint *exit,
    const Waypoint *entry);

  void BuildConnectionsTupleList(
    const std::string &connections,
    std::vector<std::tuple<std::string, std::string>> &conn_vector);

  const Waypoint* FindWaypointById(
    const std::map<uint, std::vector<Waypoint>> &waypoints_map,
    const std::string &wpId);

  /// Tolerances for monolane's Builder.
  const double linear_tolerance_  = 0.01;
  const double angular_tolerance_ = 2.0 * M_PI;//0.01 * M_PI;

  const RNDFRoadCharacteristics rc_;
};

}  // namespace automotive
}  // namespace drake