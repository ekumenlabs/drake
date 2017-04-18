#include "drake/automotive/maliput/rndf/builder.h"

#include <cmath>
#include <utility>

#include "drake/automotive/maliput/rndf/branch_point.h"
#include "drake/automotive/maliput/rndf/spline_lane.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace maliput {
namespace rndf {

Builder::Builder(const api::RBounds& lane_bounds,
                 const api::RBounds& driveable_bounds,
                 const double linear_tolerance,
                 const double angular_tolerance)
    : lane_bounds_(lane_bounds),
      driveable_bounds_(driveable_bounds),
      linear_tolerance_(linear_tolerance),
      angular_tolerance_(angular_tolerance) {
  DRAKE_DEMAND(lane_bounds_.r_min >= driveable_bounds_.r_min);
  DRAKE_DEMAND(lane_bounds_.r_max <= driveable_bounds_.r_max);
}

const Connection* Builder::Connect(
      const std::string& id,
      const std::vector<Endpoint> &points) {
  connections_.push_back(std::make_unique<Connection>(
    id, points));
  return connections_.back().get();
}

void Builder::CreateLaneConnections(
  const uint segment_id,
  const uint lane_id,
  const std::vector<ignition::math::Vector3d> &points) {
  DRAKE_DEMAND(points.size() >= 2);
  // Build the base name
  const auto &base_name = BuildName(segment_id, lane_id);
  // We first get the initial and final tangents from the
  // heading of the initial and last point.
  const auto &initial_tangent = (points[1] - points[0]).Normalize() /** 0.5*/;
  const auto &end_tangent = (points[points.size() - 2] - points.back()).Normalize() /** 0.5*/;

  // We generate the spline
  ignition::math::Spline spline;
  spline.AutoCalculate(true);
  spline.Tension(SPLINE_TENSION);

  spline.AddPoint(points.front(), initial_tangent);
  for(uint i = 1; i < points.size() - 1; i++) {
    spline.AddPoint(points[i]);
  }
  spline.AddPoint(points.back(), end_tangent);

  // We move the spline to connections
  for(uint i = 0; i < (points.size() - 1); i++) {
    // Get the points with their respective tangent.

    const auto init_pose =
      std::make_tuple(spline.Point(i), spline.Tangent(i));
    const auto end_pose =
      std::make_tuple(spline.Point(i + 1), spline.Tangent(i + 1));
    // Generate the name for the new connection
    const auto &name = base_name + std::to_string(i + 1) +
      "-" + base_name + std::to_string(i + 2);
    // Add the waypoints to the map so as to use them later
    // for connections
    waypoints[BuildName(segment_id, lane_id, i + 1)] = init_pose;
    waypoints[BuildName(segment_id, lane_id, i + 2)] = end_pose;

    // Convert those points into endpoints
    std::vector<Endpoint> endpoints;
    endpoints.push_back(ConvertIntoEndpoint(init_pose));
    endpoints.push_back(ConvertIntoEndpoint(end_pose));
    // Create a connection
    Connect(name, endpoints);
  }
}

void Builder::CreateLaneToLaneConnection(
  const std::string &exit_id,
  const std::string &entry_id) {
  auto exit_pose = waypoints.find(exit_id);
  auto entry_pose = waypoints.find(entry_id);
  // Checks
  DRAKE_DEMAND(exit_pose != waypoints.end());
  DRAKE_DEMAND(entry_pose != waypoints.end());
  // Convert those poses into endpoints
  std::vector<Endpoint> endpoints;
  endpoints.push_back(ConvertIntoEndpoint(exit_pose->second));
  endpoints.push_back(ConvertIntoEndpoint(entry_pose->second));
  // Generate the spline
  Connect(exit_id + "-" + entry_id, endpoints);
}


void Builder::SetDefaultBranch(
    const Connection* in, const api::LaneEnd::Which in_end,
    const Connection* out, const api::LaneEnd::Which out_end) {
  default_branches_.push_back({in, in_end, out, out_end});
}

Group* Builder::MakeGroup(const std::string& id) {
  groups_.push_back(std::make_unique<Group>(id));
  return groups_.back().get();
}


Group* Builder::MakeGroup(const std::string& id,
                          const std::vector<const Connection*>& connections) {
  groups_.push_back(std::make_unique<Group>(id, connections));
  return groups_.back().get();
}

namespace {
// Determine the heading (in xy-plane) along the centerline when
// travelling towards/into the lane, from the specified end.
double HeadingIntoLane(const api::Lane* const lane,
                       const api::LaneEnd::Which end) {
  switch (end) {
    case api::LaneEnd::kStart: {
      return lane->GetOrientation({0., 0., 0.}).yaw;
    }
    case api::LaneEnd::kFinish: {
      return lane->GetOrientation({lane->length(), 0., 0.}).yaw + M_PI;
    }
    default: { DRAKE_ABORT(); }
  }
}
}  // namespace



BranchPoint* Builder::FindOrCreateBranchPoint(
    const Endpoint& point,
    RoadGeometry* road_geometry,
    std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* bp_map) const {
  auto ibp = bp_map->find(point);
  if (ibp != bp_map->end()) {
    return ibp->second;
  }
  // TODO(maddog@tri.global) Generate a more meaningful id (user-specified?)
  BranchPoint* bp = road_geometry->NewBranchPoint(
      {"bp:" + std::to_string(road_geometry->num_branch_points())});
  auto result = bp_map->emplace(point, bp);
  DRAKE_DEMAND(result.second);
  return bp;
}


void Builder::AttachBranchPoint(
    const Endpoint& point, Lane* const lane, const api::LaneEnd::Which end,
    RoadGeometry* road_geometry,
    std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* bp_map) const {
  BranchPoint* bp = FindOrCreateBranchPoint(point, road_geometry, bp_map);
  // Tell the lane about its branch-point.
  switch (end) {
    case api::LaneEnd::kStart: {
      lane->SetStartBp(bp);
      break;
    }
    case api::LaneEnd::kFinish: {
      lane->SetEndBp(bp);
      break;
    }
    default: { DRAKE_ABORT(); }
  }
  // Now, tell the branch-point about the lane.
  //
  // Is this the first lane-end added to the branch-point?
  // If so, just stick it on A-Side.
  // (NB: We just test size of A-Side, since A-Side is always populated first.)
  if (bp->GetASide()->size() == 0) {
    bp->AddABranch({lane, end});
    return;
  }
  // Otherwise, assess if this new lane-end is parallel or anti-parallel to
  // the first lane-end.  Parallel: go to same, A-side; anti-parallel:
  // other, B-side.  Do this by examining the dot-product of the heading
  // vectors (rather than goofing around with cyclic angle arithmetic).
  const double new_h = HeadingIntoLane(lane, end);
  const api::LaneEnd old_le = bp->GetASide()->get(0);
  const double old_h = HeadingIntoLane(old_le.lane, old_le.end);
  if (((std::cos(new_h) * std::cos(old_h)) +
       (std::sin(new_h) * std::sin(old_h))) > 0.) {
    bp->AddABranch({lane, end});
  } else {
    bp->AddBBranch({lane, end});
  }
}


Lane* Builder::BuildConnection(
    const Connection* const conn,
    Junction* const junction,
    RoadGeometry* const road_geometry,
    std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* const bp_map) const {
  Segment* segment = junction->NewSegment({std::string("s:") + conn->id()});
  Lane* lane{};
  api::LaneId lane_id{std::string("l:") + conn->id()};

  switch (conn->type()) {
    case Connection::kSpline: {
      std::vector<std::tuple<
        ignition::math::Vector3d,
        ignition::math::Vector3d>> points_tangents;
      for (const auto& endpoint : conn->points()) {
        const auto &point =
          ignition::math::Vector3d(endpoint.xy().x(), endpoint.xy().y(), 0.);
        const auto &tangent =
          ignition::math::Vector3d(1., std::tan(endpoint.xy().heading()), 0.).Normalize() *
          endpoint.xy().heading_mod();
        points_tangents.push_back(std::make_tuple(point, tangent));
      }
      lane = segment->NewSplineLane(lane_id,
        points_tangents,
        lane_bounds_,
        driveable_bounds_);
      break;
    }
    default: {
      DRAKE_ABORT();
    }
  }

  AttachBranchPoint(
      conn->start(), lane, api::LaneEnd::kStart, road_geometry, bp_map);
  AttachBranchPoint(
      conn->end(), lane, api::LaneEnd::kFinish, road_geometry, bp_map);
  return lane;
}


std::unique_ptr<const api::RoadGeometry> Builder::Build(
    const api::RoadGeometryId& id) const {
  auto road_geometry = std::make_unique<RoadGeometry>(
      id, linear_tolerance_, angular_tolerance_);
  std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder> bp_map(
      (EndpointFuzzyOrder(linear_tolerance_)));
  std::map<const Connection*, Lane*> lane_map;
  std::set<const Connection*> remaining_connections;

  for (const std::unique_ptr<Connection>& connection : connections_) {
    remaining_connections.insert(connection.get());
  }

  for (const std::unique_ptr<Group>& group : groups_) {
    Junction* junction =
        road_geometry->NewJunction({std::string("j:") + group->id()});
    drake::log()->debug("junction: {}", junction->id().id);
    for (auto& connection : group->connections()) {
      drake::log()->debug("connection: {}", connection->id());
      // Remove connection from remaining_connections, and ensure that it
      // was indeed in there.
      DRAKE_DEMAND(remaining_connections.erase(connection) == 1);
      lane_map[connection] = BuildConnection(
          connection, junction, road_geometry.get(), &bp_map);
    }
  }

  for (const Connection* const connection : remaining_connections) {
    Junction* junction =
        road_geometry->NewJunction({std::string("j:") + connection->id()});
    drake::log()->debug("junction: {}", junction->id().id);
    drake::log()->debug("connection: {}", connection->id());
    lane_map[connection] =
        BuildConnection(connection, junction, road_geometry.get(), &bp_map);
  }

  for (const DefaultBranch& def : default_branches_) {
    Lane* in_lane = lane_map[def.in];
    Lane* out_lane = lane_map[def.out];
    DRAKE_DEMAND((def.in_end == api::LaneEnd::kStart) ||
                 (def.in_end == api::LaneEnd::kFinish));
    ((def.in_end == api::LaneEnd::kStart) ?
     in_lane->start_bp() : in_lane->end_bp())
        ->SetDefault({in_lane, def.in_end},
                     {out_lane, def.out_end});
  }

  // Make sure we didn't screw up!
  std::vector<std::string> failures = road_geometry->CheckInvariants();
  for (const auto& s : failures) {
    drake::log()->error(s);
  }
  DRAKE_DEMAND(failures.size() == 0);

  return std::move(road_geometry);
}

std::string Builder::BuildName(const uint segment_id,
  const uint lane_id) const {
  return std::to_string(segment_id) + "_" +
    std::to_string(lane_id) + "_";
}

std::string Builder::BuildName(const uint segment_id,
  const uint lane_id,
  const uint waypoint_id) const {
  return std::to_string(segment_id) + "_" +
    std::to_string(lane_id) + "_" +
    std::to_string(waypoint_id);
}

Endpoint Builder::ConvertIntoEndpoint(
  const std::tuple<ignition::math::Vector3d,
    ignition::math::Vector3d> &pose) {
  const auto &point = std::get<0>(pose);
  const auto &tangent = std::get<1>(pose);
  return Endpoint(
    EndpointXy(point.X(),
      point.Y(),
      std::atan2(tangent.Y(), tangent.X()),
      tangent.Length()),
    EndpointZ());
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
