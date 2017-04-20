#include "drake/automotive/rndf_generator.h"

#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace automotive {

namespace rndf = maliput::rndf;

std::unique_ptr<const maliput::api::RoadGeometry>
RNDFTBuilder::Build(const std::string &file_name) {
  std::unique_ptr<maliput::rndf::Builder> builder =
    std::make_unique<maliput::rndf::Builder>(
      rc_.lane_bounds,
      rc_.driveable_bounds,
      linear_tolerance_,
      angular_tolerance_);

  std::unique_ptr<ignition::rndf::RNDF> rndfInfo =
    std::make_unique<ignition::rndf::RNDF>(file_name);
  DRAKE_DEMAND(rndfInfo->Valid());

  std::vector<ignition::rndf::Segment> &segments =
    rndfInfo->Segments();

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

  BuildSegments(*builder, origin, segments);
  BuildConnections(*builder, segments);

  return builder->Build({rndfInfo->Name()});
}

void RNDFTBuilder::BuildSegments(
  maliput::rndf::Builder &builder,
  const ignition::math::Vector3d &origin,
  std::vector<ignition::rndf::Segment> &segments) const{
  for (auto &segment : segments) {
    std::vector<ignition::rndf::Lane> &lanes =
      segment.Lanes();
    for (auto &lane : lanes) {
      std::vector<ignition::math::Vector3d> waypoint_positions;
      std::vector<ignition::rndf::Waypoint> &waypoints =
        lane.Waypoints();
      for (auto &waypoint : waypoints) {
        waypoint_positions.push_back(
          ToGlobalCoordinates(origin, waypoint.Location()));
      }
      builder.CreateLaneConnections(segment.Id(),
        lane.Id(),
        waypoint_positions);
    }
  }
}

void RNDFTBuilder::BuildConnections(
  maliput::rndf::Builder &builder,
  std::vector<ignition::rndf::Segment> &segments) const {

  const auto unique_id_to_str = [] (
    const ignition::rndf::UniqueId &id) {
    return std::to_string(id.X()) + "_" +
      std::to_string(id.Y()) + "_" +
      std::to_string(id.Z());
  };
  for (auto &segment : segments) {
    std::vector<ignition::rndf::Lane> &lanes =
      segment.Lanes();
    for (auto &lane : lanes) {
      std::vector<ignition::math::Vector3d> waypoint_positions;
      std::vector<ignition::rndf::Exit> &exits =
        lane.Exits();
      for (auto &exit : exits) {
        const auto &exit_id = exit.ExitId();
        const auto &entry_id = exit.EntryId();
        builder.CreateLaneToLaneConnection(
          unique_id_to_str(exit_id),
          unique_id_to_str(entry_id));
      }
    }
  }
}

ignition::math::Vector3d RNDFTBuilder::ToGlobalCoordinates(
  const ignition::math::Vector3d &origin,
  const ignition::math::SphericalCoordinates &spherical_position) const{
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


}  // namespace automotive
}  // namespace drake
