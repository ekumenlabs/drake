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

void Builder::CreateLane(
  const api::RBounds& lane_bounds,
  const api::RBounds& driveable_bounds,
  const std::vector<DirectedWaypoint> &control_points) {
  DRAKE_DEMAND(control_points.size() > 1);
  const auto &start_id = control_points.front().Id();
  const auto &end_id = control_points.back().Id();
  const std::string name =
    BuildName(start_id.X(), start_id.Y(), start_id.Z()) +
    "-" +
    BuildName(end_id.X(), end_id.Y(), end_id.Z());
  connections_.push_back(std::make_unique<Connection>(name, control_points));
  DRAKE_DEMAND(connections_.back() != nullptr);
}

void Builder::CreateConnection(
  const api::RBounds& lane_bounds,
  const api::RBounds& driveable_bounds,
  const ignition::rndf::UniqueId &exit,
  const ignition::rndf::UniqueId &entry) {
  const auto &exit_it = directed_waypoints_.find(exit.String());
  const auto &entry_it = directed_waypoints_.find(entry.String());
  DRAKE_DEMAND(exit_it != directed_waypoints_.end());
  DRAKE_DEMAND(entry_it != directed_waypoints_.end());

  std::vector<DirectedWaypoint> control_points = {
    exit_it->second,
    entry_it->second
  };
  CreateLane(lane_bounds_, driveable_bounds_, control_points);
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
  const auto &initial_tangent = (points[1] - points[0]);
  const auto &end_tangent = points[points.size() - 2] - points.back();
  // We generate the spline
  ignition::math::Spline spline;
  spline.AutoCalculate(true);
  spline.Tension(0.0);

  spline.AddPoint(points.front(), initial_tangent);
  for(uint i = 1; i < points.size() - 1; i++) {
    spline.AddPoint(points[i]);
  }
  spline.AddPoint(points.back(), end_tangent);

  for (double s = 0; s <= 1.0; s += 0.01) {
    /*std::cout << "t: " << s << " | " << spline.Interpolate(s) << " | " <<
      spline.InterpolateTangent(s) << std::endl;*/
  }
  std::cout << "------------------" << std::endl;

  // We move the spline to connections
  for(uint i = 0; i < (points.size() - 1); i++) {
    // Get the points with their respective tangent.
    const auto init_pose =
      std::make_tuple(spline.Point(i),
        spline.Tangent(i));
    const auto end_pose =
      std::make_tuple(spline.Point(i + 1),
        spline.Tangent(i + 1));
    // Generate the name for the new connection
    const auto &name = base_name + std::to_string(i + 1) +
      "-" + base_name + std::to_string(i + 2);

    // Add the waypoints to the map so as to use them later
    // for connections
    waypoints[BuildName(segment_id, lane_id, i + 1)] = init_pose;
    waypoints[BuildName(segment_id, lane_id, i + 2)] = end_pose;
    /*
    std::cout << BuildName(segment_id, lane_id, i + 1) << " " << spline.Tangent(i) << std::endl;
    std::cout << BuildName(segment_id, lane_id, i + 2) << " " << spline.Tangent(i + 1) << std::endl;
    */
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

  std::cout << "Connection: " << exit_id + "-" + entry_id << std::endl;
  std::cout << "\t" << std::get<1>(exit_pose->second) << std::endl;
  std::cout << "\t" << std::get<1>(entry_pose->second) << std::endl;
  // Convert those poses into endpoints
  std::vector<Endpoint> endpoints;
  endpoints.push_back(ConvertIntoEndpoint(exit_pose->second));
  endpoints.push_back(ConvertIntoEndpoint(entry_pose->second));
  // Generate the spline
  Connect(exit_id + "-" + entry_id, endpoints);
}


void Builder::CreateLaneConnections(
  const uint segment_id,
  const uint lane_id,
  const std::vector<ignition::math::Vector3d> &points) {
  DRAKE_DEMAND(points.size() >= 2);
  // We generate the spline
  ignition::math::Spline spline;
  spline.AutoCalculate(true);
  spline.Tension(SplineLane::Tension());

  for (const auto &point : points) {
    spline.AddPoint(ignition::math::Vector3d(point.X(), point.Y(), 0.));
  }

  // We move the spline to connections
  for (uint i = 0; i < (points.size() - 1); i++) {
    // Create the directed waypoints
    const DirectedWaypoint init_wp(
      ignition::rndf::UniqueId(segment_id, lane_id, i + 1),
      spline.Point(i),
      spline.Tangent(i));
    const DirectedWaypoint end_wp(
      ignition::rndf::UniqueId(segment_id, lane_id, i + 2),
      spline.Point(i + 1),
      spline.Tangent(i + 1));
    // Add the waypoints to the map so as to use them later
    // for connections
    directed_waypoints_[init_wp.Id().String()] = init_wp;
    directed_waypoints_[end_wp.Id().String()] = end_wp;

    // Create the vector and build the name of the road
    const std::vector<DirectedWaypoint> wps = {init_wp, end_wp};
    CreateLane(lane_bounds_, driveable_bounds_, wps);
  }
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

std::unique_ptr<const api::RoadGeometry> Builder::Build(
    const api::RoadGeometryId& id) {
  std::unique_ptr<std::map<std::string, BranchPoint *>> branch_point_map =
    std::make_unique<std::map<std::string, BranchPoint *>>();
  auto road_geometry = std::make_unique<RoadGeometry>(
    id,
    linear_tolerance_,
    angular_tolerance_);

  // Build a lane for each connection and create their respective branch points
  for (const auto &connection : connections_) {
    // Build a junction
    Junction* junction =
      road_geometry->NewJunction(
        {std::string("j:") + connection->id()});
    DRAKE_DEMAND(junction != nullptr);
    // Create the lane
    drake::maliput::rndf::Lane *lane =
      BuildConnection(junction, connection.get());
    DRAKE_DEMAND(lane != nullptr);
    // Build the branch points of necessary for the lane
    BuildOrUpdateBranchpoints(connection.get(),
      lane,
      branch_point_map.get(),
      road_geometry.get());
  }
  // Make sure we didn't screw up!
  std::vector<std::string> failures = road_geometry->CheckInvariants();
  for (const auto& s : failures) {
    drake::log()->error(s);
  }
  DRAKE_DEMAND(failures.size() == 0);

  return std::move(road_geometry);
}

void Builder::AttachLaneEndToBranchPoint(
    Lane* lane,
    const api::LaneEnd::Which end,
    BranchPoint *branch_point) {
  DRAKE_DEMAND(lane != nullptr);
  DRAKE_DEMAND(branch_point != nullptr);
  // Tell the lane about its branch-point.
  switch (end) {
    case api::LaneEnd::kStart: {
      lane->SetStartBp(branch_point);
      break;
    }
    case api::LaneEnd::kFinish: {
      lane->SetEndBp(branch_point);
      break;
    }
    default: {
      DRAKE_ABORT();
    }
  }
  // Now, tell the branch-point about the lane.
  //
  // Is this the first lane-end added to the branch-point?
  // If so, just stick it on A-Side.
  // (NB: We just test size of A-Side, since A-Side is always populated first.)
  if (branch_point->GetASide()->size() == 0) {
    branch_point->AddABranch({lane, end});
    return;
  }
  // Otherwise, assess if this new lane-end is parallel or anti-parallel to
  // the first lane-end.  Parallel: go to same, A-side; anti-parallel:
  // other, B-side.  Do this by examining the dot-product of the heading
  // vectors (rather than goofing around with cyclic angle arithmetic).
  const double new_h = HeadingIntoLane(lane, end);
  const api::LaneEnd old_le = branch_point->GetASide()->get(0);
  const double old_h = HeadingIntoLane(old_le.lane, old_le.end);
  if (((std::cos(new_h) * std::cos(old_h)) +
       (std::sin(new_h) * std::sin(old_h))) > 0.) {
    branch_point->AddABranch({lane, end});
  } else {
    branch_point->AddBBranch({lane, end});
  }
}

Lane* Builder::BuildConnection(
  Junction *junction,
  const Connection *connection) {
  DRAKE_DEMAND(junction != nullptr);
  // Create a new segment and assign a lane to it
  Segment* segment = junction->NewSegment(
    {std::string("s:") + connection->id()});
  Lane* lane{};
  api::LaneId lane_id{std::string("l:") + connection->id()};

  switch (connection->type()) {
    case Connection::kSpline: {
      std::vector<std::tuple<
        ignition::math::Vector3d,
        ignition::math::Vector3d>> points_tangents;
      for (const auto& directed_waypoint : connection->waypoints()) {
        points_tangents.push_back(std::make_tuple(
          directed_waypoint.Position(), directed_waypoint.Tangent()));
      }
      lane = segment->NewSplineLane(lane_id,
        points_tangents,
        lane_bounds_,
        driveable_bounds_);
      break;
    }
    case Connection::kSpline: {
      std::vector<std::tuple<
        ignition::math::Vector3d,
        ignition::math::Vector3d>> points_tangents;
      for (const auto& endpoint : conn->points()) {
        const auto &point = ignition::math::Vector3d(endpoint.xy().x(), endpoint.xy().y(), 0.);
        const auto &tangent = ignition::math::Vector3d(1., std::tan(endpoint.xy().heading()), 0.) *
          endpoint.xy().heading_mod();
        points_tangents.push_back(std::make_tuple(point, tangent));
      }
      lane = segment->NewSplineLane(lane_id,
        points_tangents,
        lane_bounds_,
        driveable_bounds_,
        CubicPolynomial(),
        CubicPolynomial());
      break;
    }
    default: {
      DRAKE_ABORT();
    }
  }
  return lane;
}

void Builder::BuildOrUpdateBranchpoints(
  Connection *connection,
  Lane *lane,
  std::map<std::string, BranchPoint*> *branch_point_map,
  RoadGeometry *road_geometry) {
  DRAKE_DEMAND(connection != nullptr);
  DRAKE_DEMAND(lane != nullptr);
  DRAKE_DEMAND(branch_point_map != nullptr);
  DRAKE_DEMAND(road_geometry != nullptr);
  // First we care about the start of the branch point
  BranchPoint* bp{nullptr};
  auto it = branch_point_map->find(connection->start().Id().String());
  if (it == branch_point_map->end()) {
    bp = road_geometry->NewBranchPoint(
      {"bp:" + std::to_string(road_geometry->num_branch_points())});
    DRAKE_DEMAND(bp != nullptr);
    (*branch_point_map)[connection->start().Id().String()] = bp;
  } else {
    bp = it->second;
  }
  AttachLaneEndToBranchPoint(lane, api::LaneEnd::kStart, bp);
  // Now, it's the turn of the end lane end
  bp = nullptr;
  it = branch_point_map->find(connection->end().Id().String());
  if (it == branch_point_map->end()) {
    bp = road_geometry->NewBranchPoint(
      {"bp:" + std::to_string(road_geometry->num_branch_points())});
    DRAKE_DEMAND(bp != nullptr);
    (*branch_point_map)[connection->end().Id().String()] = bp;
  } else {
    bp = it->second;
  }
  AttachLaneEndToBranchPoint(lane, api::LaneEnd::kFinish, bp);
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
  return Endpoint(
    EndpointXy(std::get<0>(pose).X(),
      std::get<0>(pose).Y(),
      std::atan2(std::get<1>(pose).Y(), std::get<1>(pose).X()),
      std::get<1>(pose).Length()),
    EndpointZ());
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
