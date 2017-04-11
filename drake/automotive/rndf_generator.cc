#include "drake/automotive/rndf_generator.h"

#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace automotive {

namespace rndf = maliput::rndf;

std::unique_ptr<const maliput::api::RoadGeometry>
RNDFTBuilder::Build() {
  std::unique_ptr<maliput::rndf::Builder> rb(
      new maliput::rndf::Builder(rc_.lane_bounds, rc_.driveable_bounds,
                                     linear_tolerance_, angular_tolerance_));
  const rndf::EndpointZ kFlatZ{0., 0., 0., 0.};
  std::vector<rndf::Endpoint> endpoints;

  // |
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(0.0, 0.0, 0.0),
    kFlatZ));
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(50.0, 0.0, 0.0),
    kFlatZ));
  rb->Connect("s1l1", endpoints);
  endpoints.clear();

  // |
  // |
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(50.0, 0.0, 0.0),
    kFlatZ));
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(100.0, 0.0, 0.0),
    kFlatZ));
  rb->Connect("s2l1", endpoints);
  endpoints.clear();

  // |
  //  -
  // |
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(50.0, 0.0, 0.0),
    kFlatZ));
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(55, 1.34, 30),
    kFlatZ));
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(58.66, 5.0, 60),
    kFlatZ));
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(60.0, 10.0, 90),
    kFlatZ));
  rb->Connect("s1l1tos3l1", endpoints);
  endpoints.clear();

  // |
  //  - --
  // |
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(60.0, 10.0, 90.0),
    kFlatZ));
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(60.0, 60.0, 90.0),
    kFlatZ));
  rb->Connect("s3l1", endpoints);
  endpoints.clear();

  return rb->Build({"rndf-T-example"});
}

std::unique_ptr<const maliput::api::RoadGeometry>
RNDFTBuilder::Build(
  const std::string &road_waypoints,
  const std::string &connections) {
  std::map<uint, std::vector<Waypoint>> waypoints_map;
  BuildWaypointMap(road_waypoints, waypoints_map);

  // Print the map to see if everything is OK
  for(const auto &kv : waypoints_map) {
    std::cout << kv.first << std::endl;
    for (const auto &wp : kv.second) {
      std::cout << "\t" << wp.IdStr() << std::endl;
    }
  }
  // Create the builder
  std::unique_ptr<maliput::rndf::Builder> builder(
    new maliput::rndf::Builder(rc_.lane_bounds,
      rc_.driveable_bounds,
      linear_tolerance_,
      angular_tolerance_));

  // Build the segments
  for(const auto &kv: waypoints_map) {
    BuildSegment(*builder, kv.first, kv.second);
  }
/*
  // Generate the connections
  std::vector<std::tuple<std::string, std::string>> conn_vector;
  BuildConnectionsTupleList(connections, conn_vector);

  // Build the connections
  for (const auto &connection : conn_vector) {
    std::string exitId = std::get<0>(connection);
    std::replace(exitId.begin(), exitId.end(), '.', '_');
    const Waypoint *exit = FindWaypointById(
      waypoints_map,
      exitId);
    DRAKE_DEMAND(exit != nullptr);

    std::string entryId = std::get<1>(connection);
    std::replace(entryId.begin(), entryId.end(), '.', '_');
    const Waypoint *entry = FindWaypointById(
      waypoints_map,
      entryId);
    DRAKE_DEMAND(entry != nullptr);

    BuildConnection(*builder, exit, entry);
  }
*/
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
  const std::vector<Waypoint> &waypoints) {
  std::vector<ignition::math::Vector3d> endpoints;
  // We convert all the waypoints locations in spherical to global
  for (const auto &waypoint : waypoints) {
    endpoints.push_back(waypoint.ToGlobalCoordinates(10., 65.));
  }
  const auto &base_name = std::to_string(segment_id) + "_" +
    std::to_string(waypoints.front().LaneId()) + "_";
  builder.CreateLaneConnections(base_name, endpoints);
}
/*
void RNDFTBuilder::BuildConnection(
  maliput::rndf::Builder &builder,
  const Waypoint *exit,
  const Waypoint *entry) {

  const rndf::EndpointZ kFlatZ{0., 0., 0., 0.};
  std::vector<rndf::Endpoint> endpoints;

  const auto &position_exit = exit->ToGlobalCoordinates(10., 65.);
  const auto &position_entry = entry->ToGlobalCoordinates(10., 65.);
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(std::get<0>(position_exit), std::get<1>(position_exit), 0.0),
    kFlatZ));
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(std::get<0>(position_entry), std::get<1>(position_entry), 0.0),
    kFlatZ));
  builder.Connect(
    exit->IdStr() + "-" + entry->IdStr(),
    endpoints);
}
*/

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



}  // namespace automotive
}  // namespace drake
