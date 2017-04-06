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
    position_ = directed_waypoint.position_;
    tangent_ = directed_waypoint.tangent_;
    id_.SetX(directed_waypoint.id_.X());
    id_.SetY(directed_waypoint.id_.Y());
    id_.SetZ(directed_waypoint.id_.Z());
    return *this;
  }
  static void DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE() {
    (void) static_cast<DirectedWaypoint& (DirectedWaypoint::*)
      (const DirectedWaypoint&)>(&DirectedWaypoint::operator=);
  }

<<<<<<< HEAD
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

=======
  /// Returns the subset of parameters pertaining to the xy ground-plane.
  const EndpointXy& xy() const { return xy_; }

  /// Returns the subset of parameters pertaining to out-of-ground-plane
  /// aspects.
  const EndpointZ& z() const { return z_; }

 private:
  EndpointXy xy_;
  EndpointZ z_;
};


>>>>>>> Removed unused code from the builders and from the loader.
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

<<<<<<< HEAD
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
    DRAKE_DEMAND(waypoints_.size() >= 2);
  }
=======
  /// Possible connection geometries:  line- or arc-segment.
  enum Type { kSpline };

>>>>>>> Removed unused code from the builders and from the loader.

  /// Constructs a spline-segment connection joining @p points[0] to @p points[size-1].
  Connection(const std::string& id,
      const std::vector<Endpoint>& points)
      : type_(kSpline), id_(id), start_(points.front()), end_(points.back()), points_(points) {
    DRAKE_DEMAND(points_.size() >= 2);
  }

  /// Returns the geometric type of the path.
  Type type() const { return type_; }

  /// Returns the ID string.
  const std::string& id() const { return id_; }

  /// Returns the parameters of the start point.
  const DirectedWaypoint& start() const { return start_; }
  DirectedWaypoint& start() { return start_; }

  /// Returns the parameters of the endpoint.
<<<<<<< HEAD
  const DirectedWaypoint& end() const { return end_; }
  DirectedWaypoint& end() { return end_; }

  const std::vector<DirectedWaypoint> &waypoints() const {
    DRAKE_DEMAND(type_ == kSpline);
    return waypoints_;
  }
=======
  const Endpoint& end() const { return end_; }

  // /// Returns the x-component of the arc center (for arc connections only).
  // double cx() const {
  //   DRAKE_DEMAND(type_ == kArc);
  //   return cx_;
  // }

  // /// Returns the y-component of the arc center (for arc connections only).
  // double cy() const {
  //   DRAKE_DEMAND(type_ == kArc);
  //   return cy_;
  // }

  // /// Returns the radius of the arc (for arc connections only).
  // double radius() const {
  //   DRAKE_DEMAND(type_ == kArc);
  //   return radius_;
  // }

  // /// Returns the angle of the arc (for arc connections only).
  // double d_theta() const {
  //   DRAKE_DEMAND(type_ == kArc);
  //   return d_theta_;
  // }
>>>>>>> Removed unused code from the builders and from the loader.

  const std::vector<Endpoint> &points() const {
    DRAKE_DEMAND(type_ == kSpline);
    return points_;
  }

 private:
  Type type_{};
  std::string id_;
<<<<<<< HEAD
  DirectedWaypoint start_;
  DirectedWaypoint end_;
  std::vector<DirectedWaypoint> waypoints_;
=======
  Endpoint start_;
  Endpoint end_;
  std::vector<Endpoint> points_;

  // // Bits specific to type_ == kArc:
  // double cx_{};
  // double cy_{};
  // double radius_{};
  // double d_theta_{};
};


/// A group of Connections.
///
/// Upon building the RoadGeometry, a Group yields a Junction containing the
/// corresponding Segments specified by all the Connections in the Group.
class Group {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Group)

  /// Constructs an empty Group with the specified @p id.
  explicit Group(const std::string& id) : id_(id) {}

  /// Constructs a Group with @p id, populated by @p connections.
  Group(const std::string& id,
        const std::vector<const Connection*>& connections)
      : id_(id), connections_(connections.begin(), connections.end()) {}

  /// Adds a Connection.
  void Add(const Connection* connection) {
    auto result = connections_.insert(connection);
    DRAKE_DEMAND(result.second);
  }

  /// Returns the ID string.
  const std::string& id() const { return id_; }

  /// Returns the grouped Connections.
  const std::set<const Connection*>& connections() const {
    return connections_;
  }

 private:
  std::string id_;
  std::set<const Connection*> connections_;
>>>>>>> Removed unused code from the builders and from the loader.
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

<<<<<<< HEAD
  void CreateLaneConnections(
    const uint segment_id,
    const uint lane_id,
    const std::vector<ignition::math::Vector3d> &points);

  void CreateConnection(
    const api::RBounds& lane_bounds,
    const api::RBounds& driveable_bounds,
    const ignition::rndf::UniqueId &exit,
    const ignition::rndf::UniqueId &entry);
=======
  const Connection* Connect(
      const std::string& id,
      const std::vector<Endpoint> &points);

  /// Sets the default branch for one end of a connection.
  ///
  /// The default branch for the @p in_end of connection @p in will set to be
  /// @p out_end of connection @p out.  The specified connections must
  /// actually be joined at the specified ends (i.e., the Endpoint's for
  /// those ends must be coincident and (anti)parallel within the tolerances
  /// for the Builder).
  void SetDefaultBranch(
      const Connection* in, const api::LaneEnd::Which in_end,
      const Connection* out, const api::LaneEnd::Which out_end);

  /// Creates a new empty connection group with ID string @p id.
  Group* MakeGroup(const std::string& id);

  /// Creates a new connection group with ID @p id, populated with the
  /// given @p connections.
  Group* MakeGroup(const std::string& id,
                   const std::vector<const Connection*>& connections);
>>>>>>> Removed unused code from the builders and from the loader.

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
