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
  const std::string &base_name,
  const std::vector<ignition::math::Vector3d> &points) {
  DRAKE_DEMAND(points.size() >= 2);
  // We first get the initial and final tangents from the
  // heading of the initial and last point.
  const auto &initial_tangent =
    (points[1] - points[0]).Normalize();
  const auto &end_tangent =
    (points[points.size() - 2] - points.back()).Normalize();
  // We generate the spline
  ignition::math::Spline spline;
  spline.Tension(0.0);
  spline.AutoCalculate(true);
  for(uint i = 0; i < points.size(); i++) {
    if (i == 0)
      spline.AddPoint(points[i], initial_tangent);
    else if (i == (points.size() - 1))
      spline.AddPoint(points[i], end_tangent);
    else
      spline.AddPoint(points[i]);
  }
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
    // Convert those points into endpoints
    std::vector<Endpoint> endpoints;
    endpoints.push_back(Endpoint(
      EndpointXy(std::get<0>(init_pose).X(),
        std::get<0>(init_pose).Y(),
        std::atan2(std::get<1>(init_pose).Y(), std::get<1>(init_pose).X())),
      EndpointZ()));
    endpoints.push_back(Endpoint(
      EndpointXy(std::get<0>(end_pose).X(),
        std::get<0>(end_pose).Y(),
        std::atan2(std::get<1>(end_pose).Y(), std::get<1>(end_pose).X())),
      EndpointZ()));
    // Create a connection
    Connect(name, endpoints);
  }
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

      std::vector<Point2> points;
      for (const auto& point : conn->points()) {
        points.push_back(Point2{point.xy().x(), point.xy().y()});
      }
      lane = segment->NewSplineLane(lane_id,
                                  points,
                                  lane_bounds_, driveable_bounds_,
                                  CubicPolynomial(), CubicPolynomial());
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


}  // namespace rndf
}  // namespace maliput
}  // namespace drake
