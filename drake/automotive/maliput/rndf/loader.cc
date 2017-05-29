#include "drake/automotive/maliput/rndf/loader.h"
#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace maliput {
namespace rndf {

std::unique_ptr<const maliput::api::RoadGeometry>
Loader::LoadFile(const std::string& file_name) {
  builder = std::make_unique<maliput::rndf::Builder>(linear_tolerance_,
    angular_tolerance_);
  std::unique_ptr<ignition::rndf::RNDF> rndfInfo =
    std::make_unique<ignition::rndf::RNDF>(file_name);
  DRAKE_THROW_UNLESS(rndfInfo->Valid());

  const std::vector<ignition::rndf::Segment>& segments = rndfInfo->Segments();
  // I get the first waypoint location and build the map
  // from it.
  DRAKE_DEMAND(segments.size() > 0);
  DRAKE_DEMAND(segments[0].Lanes().size() > 0);
  DRAKE_DEMAND(segments[0].Lanes()[0].Waypoints().size() > 0);
  const ignition::math::SphericalCoordinates &location =
    segments[0].Lanes()[0].Waypoints()[0].Location();
  const ignition::math::Vector3d origin(location.LatitudeReference().Degree(),
    location.LongitudeReference().Degree(), 0.0);

  BuildBoundingBox(origin, segments);
  // We first build all segments so waypoints are populated in Builder class and
  // then we move to the lane connections which make use of the UniqueIds of the
  // waypoints to reference them.
  BuildSegments(origin, segments);
  BuildZoneLanes(origin, rndfInfo->Zones());
  BuildConnections(segments, rndfInfo->Zones());

  return builder->Build({rndfInfo->Name()});
}

void Loader::BuildBoundingBox(
  const ignition::math::Vector3d& origin,
  const std::vector<ignition::rndf::Segment>& segments) const {
  std::vector<DirectedWaypoint> wps;
  for (const auto& segment : segments) {
    for (const auto& lane : segment.Lanes()) {
      for (const auto& waypoint : lane.Waypoints()) {
        wps.push_back(
          DirectedWaypoint(ignition::rndf::UniqueId(),
            ToGlobalCoordinates(origin, waypoint.Location())));
      }
    }
  }
  const std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>
    bounding_box = DirectedWaypoint::CalculateBoundingBox(wps);
  builder->SetBoundingBox(bounding_box);
}

void Loader::BuildSegments(
  const ignition::math::Vector3d& origin,
  const std::vector<ignition::rndf::Segment>& segments) const {
  // We iterate over the segments, lanes and waypoints to build the lanes.
  for (const auto& segment : segments) {
    std::vector<Connection> segment_lanes;
    for (const auto& lane : segment.Lanes()) {
      Connection connected_lane;
      for (const auto& waypoint : lane.Waypoints()) {
        connected_lane.waypoints().push_back(DirectedWaypoint(
            ignition::rndf::UniqueId(segment.Id(), lane.Id(), waypoint.Id()),
            ToGlobalCoordinates(origin, waypoint.Location()),
            waypoint.IsEntry(), waypoint.IsExit()));
      }
      if (lane.Width() == 0.0) {
        connected_lane.set_width(rc_.default_width_);
      } else {
        connected_lane.set_width(lane.Width());
      }
      segment_lanes.push_back(connected_lane);
    }
    builder->CreateSegmentConnections(segment.Id(), &segment_lanes);
  }
}

void Loader::BuildConnections(
  const std::vector<ignition::rndf::Segment>& segments,
  const std::vector<ignition::rndf::Zone>& zones) const {
  auto build_connection_bounds = [segments, zones, this](
      const double exit_width, const ignition::rndf::UniqueId& entry_id) {
    for (const auto& segment : segments) {
      if (segment.Id() != entry_id.X())
        continue;
      for (const auto& lane : segment.Lanes()) {
        if (lane.Id() != entry_id.Y())
          continue;
        const double entry_width = lane.Width();
        const double width = std::min(exit_width, entry_width);
        if (width == 0.0) {
          return this->rc_.default_width_;
        } else {
          return width;
        }
      }
      DRAKE_ABORT_MSG(
        (entry_id.String() + std::string("was not found.")).c_str());
    }
    if (exit_width == 0.0) {
      return this->rc_.default_width_;
    }
    return exit_width;
  };
  // We iterate over the segments looking for each segment connection. We get
  // the exit and entry id from them and the builder uses it to build a
  // connection.
  for (const auto& segment : segments) {
    for (const auto& lane : segment.Lanes()) {
      for (const auto& exit : lane.Exits()) {
        const ignition::rndf::UniqueId& exit_id = exit.ExitId();
        const ignition::rndf::UniqueId& entry_id = exit.EntryId();
        // We define the bounds for the connection
        const double width = build_connection_bounds(lane.Width(), entry_id);
        // We set a default value for the width
        builder->CreateConnection(width, exit_id, entry_id);
      }
    }
  }

  for (const auto& zone : zones) {
    const ignition::rndf::Perimeter& perimeter = zone.Perimeter();
    std::vector<DirectedWaypoint> perimeter_waypoints;
    for (const auto& exit : perimeter.Exits()) {
      const ignition::rndf::UniqueId& exit_id = exit.ExitId();
      const ignition::rndf::UniqueId& entry_id = exit.EntryId();
      // We define the bounds for the connection
      const double width =
        build_connection_bounds(this->rc_.default_width_, entry_id);
      // We set a default value for the width
      builder->CreateConnection(width, exit_id, entry_id);
    }
  }
}

void Loader::BuildZoneLanes(
  const ignition::math::Vector3d& origin,
  const std::vector<ignition::rndf::Zone>& zones) const {
  for (const auto& zone : zones) {
    const ignition::rndf::Perimeter& perimeter = zone.Perimeter();
    std::vector<DirectedWaypoint> perimeter_waypoints;
    for (const auto& waypoint : perimeter.Points()) {
      perimeter_waypoints.push_back(DirectedWaypoint(
        ignition::rndf::UniqueId(zone.Id(), 0, waypoint.Id()),
        ToGlobalCoordinates(origin, waypoint.Location()), waypoint.IsEntry(),
        waypoint.IsExit()));
    }
    builder->CreateConnectionsForZones(this->rc_.default_width_,
      &perimeter_waypoints);
  }
}

ignition::math::Vector3d Loader::ToGlobalCoordinates(
  const ignition::math::Vector3d& origin,
  const ignition::math::SphericalCoordinates& spherical_position) const {
  const auto build_spherical_coordinates = [](const double latitude,
    const double longitude) {
    return ignition::math::SphericalCoordinates(
        ignition::math::SphericalCoordinates::EARTH_WGS84,
        ignition::math::Angle(latitude / 180.0 * M_PI),
        ignition::math::Angle(longitude / 180.0 * M_PI), 0.0,
        ignition::math::Angle(0.0));
  };

  const ignition::math::SphericalCoordinates _origin =
    build_spherical_coordinates(origin.X(), origin.Y());
  const ignition::math::Vector3d _spherical_position(
    spherical_position.LatitudeReference().Radian(),
    spherical_position.LongitudeReference().Radian(),
    spherical_position.ElevationReference());
  return _origin.PositionTransform(
    _spherical_position, ignition::math::SphericalCoordinates::SPHERICAL,
    ignition::math::SphericalCoordinates::GLOBAL);
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
