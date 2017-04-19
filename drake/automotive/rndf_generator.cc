#include "drake/automotive/rndf_generator.h"

#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace automotive {

namespace rndf = maliput::rndf;

std::unique_ptr<const maliput::api::RoadGeometry>
RNDFTBuilder::Build() {
  std::unique_ptr<maliput::rndf::Builder> builder =
    std::make_unique<maliput::rndf::Builder>(
      rc_.lane_bounds,
      rc_.driveable_bounds,
      linear_tolerance_,
      angular_tolerance_);

  std::map<uint, std::vector<Waypoint>> waypoints_map;

  {
    std::vector<Waypoint> wps;
    wps.push_back(Waypoint(1u, 1u, 1u, -34.584502, -58.444782));
    wps.push_back(Waypoint(1u, 1u, 2u, -34.583823, -58.446346));
    wps.push_back(Waypoint(1u, 1u, 3u, -34.583287, -58.447599));
    waypoints_map[1] = wps;
  }

  {
    std::vector<Waypoint> wps;
    wps.push_back(Waypoint(2u, 1u, 1u, -34.583726, -58.446402));
    wps.push_back(Waypoint(2u, 1u, 2u, -34.583104, -58.446062));
    waypoints_map[2] = wps;
  }


  for(const auto &kv: waypoints_map) {
    BuildSegment(*builder, kv.first, kv.second, -34.584502, -58.444782);
  }

  // Generate the connections
  std::string exitId = "1_1_2";
  std::string entryId = "2_1_1";
  builder->CreateLaneToLaneConnection(exitId, entryId);



  /*
  {
    std::vector<Waypoint> wps;
    wps.push_back(Waypoint(1u, 1u, 1u, -34.587094, -58.462872));
    wps.push_back(Waypoint(1u, 1u, 2u, -34.586617, -58.462350));
    wps.push_back(Waypoint(1u, 1u, 3u, -34.586426, -58.462119));
    wps.push_back(Waypoint(1u, 1u, 4u, -34.586152, -58.461719));
    wps.push_back(Waypoint(1u, 1u, 5u, -34.585940, -58.461293));
    wps.push_back(Waypoint(1u, 1u, 6u, -34.585790, -58.460813));
    wps.push_back(Waypoint(1u, 1u, 7u, -34.585786, -58.460733));
    wps.push_back(Waypoint(1u, 1u, 8u, -34.585693, -58.460336));
    wps.push_back(Waypoint(1u, 1u, 9u, -34.585680, -58.459735));
    wps.push_back(Waypoint(1u, 1u, 10u, -34.585797, -58.459445));
    waypoints_map[1] = wps;
  }
  for(const auto &kv: waypoints_map) {
    BuildSegment(*builder, kv.first, kv.second, -34.587094, -58.462872);
  }
  */

  return builder->Build({"RNDF-T-example"});
}

std::unique_ptr<const maliput::api::RoadGeometry>
RNDFTBuilder::Build(
  const std::string &road_waypoints,
  const std::string &connections) {

  std::unique_ptr<maliput::rndf::Builder> builder =
    std::make_unique<maliput::rndf::Builder>(
      rc_.lane_bounds,
      rc_.driveable_bounds,
      linear_tolerance_,
      angular_tolerance_);

  std::map<uint, std::vector<Waypoint>> waypoints_map;
  BuildWaypointMap(road_waypoints, waypoints_map);

  // Print the map to see if everything is OK
  /*
  for(const auto &kv : waypoints_map) {
    std::cout << kv.first << std::endl;
    for (const auto &wp : kv.second) {
      std::cout << "\t" << wp.IdStr() << std::endl;
    }
  }
  */

  // Build the segments
  for(const auto &kv: waypoints_map) {
    BuildSegment(*builder, kv.first, kv.second, 10.0, 65.0);
  }

  // Generate the connections
  std::vector<std::tuple<std::string, std::string>> conn_vector;
  BuildConnectionsTupleList(connections, conn_vector);

  // Build the connections
  for (const auto &connection : conn_vector) {
    std::string exitId = std::get<0>(connection);
    std::replace(exitId.begin(), exitId.end(), '.', '_');
    std::string entryId = std::get<1>(connection);
    std::replace(entryId.begin(), entryId.end(), '.', '_');

    builder->CreateLaneToLaneConnection(exitId, entryId);
  }

  return builder->Build({"SimpleCity"});
}


void RNDFTBuilder::BuildWaypointMap(
  const std::string &road_waypoints,
  std::map<uint, std::vector<Waypoint>> &waypoints_map) {
  std::vector<std::string> lines =
    Waypoint::Split(road_waypoints, "\n");

  for (const auto &line : lines) {
    const Waypoint wp(line);
    if (waypoints_map.find(wp.SegmentId()) == waypoints_map.end()) {
      std::vector<Waypoint> wps;
      wps.push_back(wp);
      waypoints_map[wp.SegmentId()] = wps;
    } else {
      waypoints_map[wp.SegmentId()].push_back(wp);
    }
  }
}

void RNDFTBuilder::BuildConnectionsTupleList(
  const std::string &connections,
  std::vector<std::tuple<std::string, std::string>> &conn_vector) {
  std::vector<std::string> conn_str_vector =
    Waypoint::Split(connections, "\n");
  for (const auto &connection : conn_str_vector) {
    std::vector<std::string> wpIds =
      Waypoint::Split(connection, " ");
    DRAKE_DEMAND(wpIds.size() == 2);
    conn_vector.push_back(std::make_tuple(wpIds[0], wpIds[1]));
  }
}

void RNDFTBuilder::BuildSegment(
  maliput::rndf::Builder &builder,
  const uint segment_id,
  const std::vector<Waypoint> &waypoints,
  const double latitude,
  const double longitude) {
  std::vector<ignition::math::Vector3d> endpoints;
  // We convert all the waypoints locations in spherical to global
  for (const auto &waypoint : waypoints) {
    endpoints.push_back(waypoint.ToGlobalCoordinates(latitude, longitude));
  }
  builder.CreateLaneConnections(segment_id,
    waypoints.front().LaneId(),
    endpoints);
}

const Waypoint* RNDFTBuilder::FindWaypointById(
  const std::map<uint, std::vector<Waypoint>> &waypoints_map,
  const std::string &wpId) {
  for(const auto &kv : waypoints_map) {
    for (const auto &wp : kv.second) {
      if(wp.MatchId(wpId))
        return &wp;
    }
  }
  return nullptr;
}
//-----------------------------------------------------------

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

  const ignition::math::Vector3d origin(10.0, 65.0, 0.0);

  std::vector<ignition::rndf::Segment> &segments =
    rndfInfo->Segments();

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
