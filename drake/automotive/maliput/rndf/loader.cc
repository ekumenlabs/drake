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
  DRAKE_THROW_UNLESS(rndfInfo->Valid());

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

  // We first build all segments so waypoints are populated in Builder class and
  // then we move to the lane connections which make use of the UniqueIds of the
  // waypoints to reference them.
  BuildSegments(origin, segments);
  BuildConnections(segments);

  return builder->Build({rndfInfo->Name()});
}

void Loader::BuildSegments(
  const ignition::math::Vector3d &origin,
  const std::vector<ignition::rndf::Segment> &segments) const {
  auto build_lane_bounds = [this](const double width) {
    if (width == 0.0) {
      return std::make_tuple(
        this->rc_.lane_bounds, this->rc_.driveable_bounds);
    }
    return std::make_tuple(
      api::RBounds(-width / 2.0, width / 2.0),
      api::RBounds(-width / 2.0, width / 2.0));
  };
  // We iterate over the segments, lanes and waypoints to build the lanes.
  for (const auto &segment : segments) {
    std::vector<ConnectedLane> segment_lanes;
    for (const auto &lane : segment.Lanes()) {
      ConnectedLane connected_lane;
      for (auto &waypoint : lane.Waypoints()) {
        connected_lane.waypoints.push_back(DirectedWaypoint(
          ignition::rndf::UniqueId(segment.Id(), lane.Id(), waypoint.Id()),
          ToGlobalCoordinates(origin, waypoint.Location()),
          waypoint.IsEntry(),
          waypoint.IsExit()));
      }
      auto lane_bounds = build_lane_bounds(lane.Width());
      connected_lane.lane_bounds = std::get<0>(lane_bounds);
      connected_lane.driveable_bounds = std::get<1>(lane_bounds);
      segment_lanes.push_back(connected_lane);
    }
    builder->CreateSegmentConnections(segment.Id(), &segment_lanes);
  }
}

void Loader::BuildConnections(
  const std::vector<ignition::rndf::Segment> &segments) const {
  auto build_connection_bounds = [segments, this](const double exit_width,
    const ignition::rndf::UniqueId &entry_id) {
    for (const auto &segment : segments) {
      if (segment.Id() != entry_id.X())
        continue;
      for (const auto &lane : segment.Lanes()) {
        if (lane.Id() != entry_id.Y())
          continue;
        const double entry_width = lane.Width();
        double width = std::min(exit_width, entry_width);
        if (width == 0.0) {
          return std::make_tuple(
            this->rc_.lane_bounds, this->rc_.driveable_bounds);
        } else {
          return std::make_tuple(
            api::RBounds(-width / 2.0, width / 2.0),
            api::RBounds(-width / 2.0, width / 2.0));
        }
      }
      DRAKE_ABORT_MSG(
        (entry_id.String() + std::string("was not found.")).c_str());
    }
    DRAKE_ABORT_MSG(
      (entry_id.String() + std::string("was not found.")).c_str());
  };
  // We iterate over the segments looking for each segment connection. We get
  // the exit and entry id from them and the builder uses it to build a
  // connection.
  for (const auto &segment : segments) {
    for (const auto &lane : segment.Lanes()) {
      for (const auto &exit : lane.Exits()) {
        const auto &exit_id = exit.ExitId();
        const auto &entry_id = exit.EntryId();
        // We define the bounds for the connection
        const auto bounds = build_connection_bounds(lane.Width(), entry_id);
        // We set a default value for the width
        builder->CreateConnection(
          std::get<0>(bounds),
          std::get<1>(bounds),
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
