#include "drake/automotive/maliput/rndf/loader.h"
#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace maliput {
namespace rndf {

std::unique_ptr<const maliput::api::RoadGeometry>
Loader::LoadFile(const std::string &file_name) {
  builder = std::make_unique<maliput::rndf::Builder>(
    rc_.lane_bounds,
    rc_.driveable_bounds,
    linear_tolerance_,
    angular_tolerance_);

  std::unique_ptr<ignition::rndf::RNDF> rndfInfo =
    std::make_unique<ignition::rndf::RNDF>(file_name);
  DRAKE_DEMAND(rndfInfo->Valid());

  const auto &segments = rndfInfo->Segments();
  // I get the first waypoint location and build the map
  // from it.
  DRAKE_DEMAND(segments.size() > 0);
  DRAKE_DEMAND(segments[0].Lanes().size() > 0);
  DRAKE_DEMAND(segments[0].Lanes()[0].Waypoints().size() > 0);
  const auto &location = segments[0].Lanes()[0].Waypoints()[0].
    Location();
  const ignition::math::Vector3d origin(
    location.LatitudeReference().Degree(),
    location.LongitudeReference().Degree(),
    0.0);

  BuildSegments(origin, segments);
  BuildConnections(segments);

  return builder->Build({rndfInfo->Name()});
}

void Loader::BuildSegments(
  const ignition::math::Vector3d &origin,
  const std::vector<ignition::rndf::Segment> &segments) const {
  for (const auto &segment : segments) {
    for (const auto &lane : segment.Lanes()) {
      std::vector<ignition::math::Vector3d> waypoint_positions;
      for (auto &waypoint : lane.Waypoints()) {
        waypoint_positions.push_back(
          ToGlobalCoordinates(origin, waypoint.Location()));
      }
      builder->CreateLaneConnections(segment.Id(),
        lane.Id(),
        waypoint_positions);
    }
  }
}

void Loader::BuildConnections(
  const std::vector<ignition::rndf::Segment> &segments) const {
  for (const auto &segment : segments) {
    for (const auto &lane : segment.Lanes()) {
      for (const auto &exit : lane.Exits()) {
        const auto &exit_id = exit.ExitId();
        const auto &entry_id = exit.EntryId();
        builder->CreateConnection(
          rc_.lane_bounds,
          rc_.driveable_bounds,
          exit_id,
          entry_id);
      }
    }
  }
}

ignition::math::Vector3d Loader::ToGlobalCoordinates(
  const ignition::math::Vector3d &origin,
  const ignition::math::SphericalCoordinates &spherical_position) const {
  const auto build_spherical_coordinates = [] (
    const double latitude, const double longitude) {
      return ignition::math::SphericalCoordinates(
        ignition::math::SphericalCoordinates::EARTH_WGS84,
        ignition::math::Angle(latitude / 180.0 * M_PI),
        ignition::math::Angle(longitude / 180.0 * M_PI),
        0.0,
        ignition::math::Angle(0.0));
  };

  const auto &_origin = build_spherical_coordinates(
    origin.X(), origin.Y());

  const auto &_spherical_position = ignition::math::Vector3d(
      spherical_position.LatitudeReference().Radian(),
      spherical_position.LongitudeReference().Radian(),
      spherical_position.ElevationReference());
  return _origin.PositionTransform(
      _spherical_position,
      ignition::math::SphericalCoordinates::SPHERICAL,
      ignition::math::SphericalCoordinates::GLOBAL);
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
