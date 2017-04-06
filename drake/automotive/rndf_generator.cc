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
RNDFTBuilder::Build(const std::string &rndf_description) {
  std::map<uint, std::vector<Waypoint>> waypoints_map;
  BuildWaypointMap(rndf_description, waypoints_map);

  for(const auto &kv : waypoints_map) {
    std::cout << kv.first << std::endl;
    for (const auto &wp : kv.second) {
      std::cout << "\t" << wp.IdStr() << std::endl;
    }
  }

  std::unique_ptr<maliput::rndf::Builder> builder(
    new maliput::rndf::Builder(rc_.lane_bounds,
      rc_.driveable_bounds,
      linear_tolerance_,
      angular_tolerance_));

  for(const auto &kv: waypoints_map) {
    BuildSegment(*builder, kv.first, kv.second);
  }

  return builder->Build({"SimpleCity"});
}

void RNDFTBuilder::BuildWaypointMap(
  const std::string &rndf_description,
  std::map<uint, std::vector<Waypoint>> &waypoints_map) {
  std::vector<std::string> lines =
    Waypoint::Split(rndf_description, "\n");

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

void RNDFTBuilder::BuildSegment(
  maliput::rndf::Builder &builder,
  const uint segment_id,
  const std::vector<Waypoint> &waypoints) {

  const rndf::EndpointZ kFlatZ{0., 0., 0., 0.};
  std::vector<rndf::Endpoint> endpoints;

  for (uint i = 0; i < (waypoints.size() - 1); i++) {
    const auto &position_1 = waypoints[i].ToGlobalCoordinates(10., 65.);
    const auto &position_2 = waypoints[i + 1].ToGlobalCoordinates(10., 65.);
    endpoints.push_back(rndf::Endpoint(
      rndf::EndpointXy(std::get<0>(position_1), std::get<1>(position_1), 0.0),
      kFlatZ));
    endpoints.push_back(rndf::Endpoint(
      rndf::EndpointXy(std::get<0>(position_2), std::get<1>(position_2), 0.0),
      kFlatZ));
    builder.Connect(
      waypoints[i].IdStr() + "-" + waypoints[i + 1].IdStr(),
      endpoints);
    endpoints.clear();
  }
}



}  // namespace automotive
}  // namespace drake
