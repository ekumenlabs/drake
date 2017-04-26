#pragma once

#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <vector>
#include <iostream>

#include "ignition/math/Vector3.hh"
#include "ignition/math/Spline.hh"
#include "ignition/rndf/UniqueId.hh"

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/rndf/junction.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {

namespace rndf {
class RoadGeometry;

class DirectedWaypoint {
 public:
  DirectedWaypoint(const DirectedWaypoint &directed_waypoint) :
    position_(directed_waypoint.position_),
    tangent_(directed_waypoint.tangent_) {
    id_.SetX(directed_waypoint.id_.X());
    id_.SetY(directed_waypoint.id_.Y());
    id_.SetZ(directed_waypoint.id_.Z());
  }
  DirectedWaypoint& operator=(const DirectedWaypoint &directed_waypoint) {
    position_ = directed_waypoint.position_;
    tangent_ = directed_waypoint.tangent_;
    id_.SetX(directed_waypoint.id_.X());
    id_.SetY(directed_waypoint.id_.Y());
    id_.SetZ(directed_waypoint.id_.Z());
    return *this;
  }
  DirectedWaypoint(DirectedWaypoint &&directed_waypoint) :
    position_(directed_waypoint.position_),
    tangent_(directed_waypoint.tangent_)  {
    id_.SetX(directed_waypoint.id_.X());
    id_.SetY(directed_waypoint.id_.Y());
    id_.SetZ(directed_waypoint.id_.Z());
  }
  DirectedWaypoint& operator=(DirectedWaypoint &&directed_waypoint) {
    if (this == &directed_waypoint) {
      position_ = directed_waypoint.position_;
      tangent_ = directed_waypoint.tangent_;
      id_.SetX(directed_waypoint.id_.X());
      id_.SetY(directed_waypoint.id_.Y());
      id_.SetZ(directed_waypoint.id_.Z());
    }
    return *this;
  }
  static void DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE() {
    (void) static_cast<DirectedWaypoint& (DirectedWaypoint::*)
      (const DirectedWaypoint&)>(&DirectedWaypoint::operator=);
  }

  DirectedWaypoint(
    const ignition::rndf::UniqueId &id,
    const ignition::math::Vector3d &position,
    const ignition::math::Vector3d &tangent) :
      id_(id),
      position_(position),
      tangent_(tangent) {}

  DirectedWaypoint() {}

  const ignition::rndf::UniqueId& Id() const {
    return id_;
  }
  const ignition::math::Vector3d& Position() const {
    return position_;
  }
  const ignition::math::Vector3d& Tangent() const {
    return tangent_;
  }

 private:
  ignition::rndf::UniqueId id_;
  ignition::math::Vector3d position_;
  ignition::math::Vector3d tangent_;
};

/// Representation of a reference path connecting two endpoints.
///
/// Upon building the RoadGeometry, a Connection yields a Segment
/// bearing a single Lane with the specified reference path.  The
/// Segment will belong to its own Junction, unless the Connection was
/// grouped with other Connections into a Group.
///
/// Two connection geometries are supported: line and arc.  These
/// primitives determine the projection of the reference path onto the
/// (locally-flat) plane of the earth.  The out-of-plane shape of
/// the path will be determined by the EndpointZ (elevation) parameters
/// of the endpoints.
class Connection {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Connection)

  /// Possible connection geometries: splines
  enum Type { kSpline };

  /// Constructs a spline-segment connection joining @p points[0] to
  /// @p points[size-1].
  Connection(const std::string& id,
    const std::vector<DirectedWaypoint>& waypoints) :
      type_(kSpline),
      id_(id),
      start_(waypoints.front()),
      end_(waypoints.back()),
      waypoints_(waypoints) {
    DRAKE_THROW_UNLESS(waypoints_.size() >= 2);
  }

  /// Returns the geometric type of the path.
  Type type() const { return type_; }

  /// Returns the ID string.
  const std::string& id() const { return id_; }

  /// Returns the parameters of the start point.
  const DirectedWaypoint& start() const { return start_; }
  DirectedWaypoint& start() { return start_; }

  /// Returns the parameters of the endpoint.
  const DirectedWaypoint& end() const { return end_; }
  DirectedWaypoint& end() { return end_; }

  const std::vector<DirectedWaypoint> &waypoints() const {
    DRAKE_THROW_UNLESS(type_ == kSpline);
    return waypoints_;
  }

 private:
  Type type_{};
  std::string id_;
  DirectedWaypoint start_;
  DirectedWaypoint end_;
  std::vector<DirectedWaypoint> waypoints_;
};

// N.B. The Builder class overview documentation lives at the top of this file.
class Builder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Builder)

  /// Constructs a Builder which can be used to specify and assemble a
  /// rndf implementation of an api::RoadGeometry.
  ///
  /// The bounds @p lane_bounds and @p driveable_bounds are applied uniformly
  /// to the single lanes of every segment; @p lane_bounds must be a subset
  /// of @p driveable_bounds.  @p linear_tolerance and @p angular_tolerance
  /// specify the respective tolerances for the resulting RoadGeometry.
  Builder(const api::RBounds& lane_bounds,
          const api::RBounds& driveable_bounds,
          const double linear_tolerance,
          const double angular_tolerance);

  void CreateLaneConnections(
    const uint segment_id,
    const uint lane_id,
    const std::vector<ignition::math::Vector3d> &points);

  void CreateConnection(
    const api::RBounds& lane_bounds,
    const api::RBounds& driveable_bounds,
    const ignition::rndf::UniqueId &exit,
    const ignition::rndf::UniqueId &entry);

  /// Produces a RoadGeometry, with the ID @p id.
  std::unique_ptr<const api::RoadGeometry> Build(
      const api::RoadGeometryId& id);

 private:
  std::string BuildName(const uint segment_id,
    const uint lane_id) const;

  std::string BuildName(const uint segment_id,
    const uint lane_id,
    const uint waypoint_id) const;

  void CreateLane(
    const api::RBounds& lane_bounds,
    const api::RBounds& driveable_bounds,
    const std::vector<DirectedWaypoint> &control_points);

  void AttachLaneEndToBranchPoint(
    Lane* lane,
    const api::LaneEnd::Which end,
    BranchPoint *branch_point);

  void BuildOrUpdateBranchpoints(
    Connection *connection,
    Lane *lane,
    std::map<std::string, BranchPoint*> *branch_point_map,
    RoadGeometry *road_geometry);

  Lane* BuildConnection(
    Junction *junction,
    const Connection *connection);

  api::RBounds lane_bounds_;
  api::RBounds driveable_bounds_;
  double linear_tolerance_{};
  double angular_tolerance_{};
  std::vector<std::unique_ptr<Connection>> connections_;
  std::map<std::string, DirectedWaypoint> directed_waypoints_;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
