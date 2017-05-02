#pragma once

#include <algorithm>
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
    is_entry_ = directed_waypoint.is_entry_;
    is_exit_ = directed_waypoint.is_exit_;
  }
  DirectedWaypoint& operator=(const DirectedWaypoint &directed_waypoint) {
    position_ = directed_waypoint.position_;
    tangent_ = directed_waypoint.tangent_;
    id_.SetX(directed_waypoint.id_.X());
    id_.SetY(directed_waypoint.id_.Y());
    id_.SetZ(directed_waypoint.id_.Z());
    is_entry_ = directed_waypoint.is_entry_;
    is_exit_ = directed_waypoint.is_exit_;
    return *this;
  }
  DirectedWaypoint(DirectedWaypoint &&directed_waypoint) :
    position_(directed_waypoint.position_),
    tangent_(directed_waypoint.tangent_)  {
    id_.SetX(directed_waypoint.id_.X());
    id_.SetY(directed_waypoint.id_.Y());
    id_.SetZ(directed_waypoint.id_.Z());
    is_entry_ = directed_waypoint.is_entry_;
    is_exit_ = directed_waypoint.is_exit_;
    is_entry_ = directed_waypoint.is_entry_;
    is_exit_ = directed_waypoint.is_exit_;
  }
  DirectedWaypoint& operator=(DirectedWaypoint &&directed_waypoint) {
    if (this == &directed_waypoint) {
      position_ = directed_waypoint.position_;
      tangent_ = directed_waypoint.tangent_;
      id_.SetX(directed_waypoint.id_.X());
      id_.SetY(directed_waypoint.id_.Y());
      id_.SetZ(directed_waypoint.id_.Z());
      is_entry_ = directed_waypoint.is_entry_;
      is_exit_ = directed_waypoint.is_exit_;
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
    const bool is_entry = false,
    const bool is_exit = false,
    const ignition::math::Vector3d &tangent = ignition::math::Vector3d::Zero) :
      id_(id),
      position_(position),
      tangent_(tangent),
      is_entry_(is_entry),
      is_exit_(is_exit) {}

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
  ignition::math::Vector3d& Tangent() {
    return tangent_;
  }
  bool is_entry() const {
    return is_entry_;
  }
  bool is_exit() const {
    return is_exit_;
  }

 private:
  ignition::rndf::UniqueId id_;
  ignition::math::Vector3d position_;
  ignition::math::Vector3d tangent_;
  bool is_entry_;
  bool is_exit_;
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
    const std::vector<DirectedWaypoint>& waypoints,
    const api::RBounds &lane_bounds,
    const api::RBounds &driveable_bounds) :
      type_(kSpline),
      id_(id),
      start_(waypoints.front()),
      end_(waypoints.back()),
      waypoints_(waypoints),
      lane_bounds_(lane_bounds.r_min, lane_bounds.r_max),
      driveable_bounds_(driveable_bounds.r_min, driveable_bounds.r_max) {
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

  const api::RBounds& lane_bounds() const {
    return lane_bounds_;
  }

  const api::RBounds& driveable_bounds() const {
    return driveable_bounds_;
  }

 private:
  Type type_{};
  std::string id_;
  DirectedWaypoint start_;
  DirectedWaypoint end_;
  std::vector<DirectedWaypoint> waypoints_;
  api::RBounds lane_bounds_;
  api::RBounds driveable_bounds_;
};

/// This structure holds all the lane information needed at construction time.
struct ConnectedLane {
  std::vector<DirectedWaypoint> waypoints;
  api::RBounds lane_bounds;
  api::RBounds driveable_bounds;
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

  /// Loads a set of connections that build a RNDF lane. @p segment_id and
  /// @p lane_id make the id of the lane and @p points are used to set the
  /// extents in terms of s coordinate of the lane. @p lane_bounds and
  /// @p driveable_bounds are the necessary values for the @class Lane.
  void CreateLaneConnections(
    const uint segment_id,
    const uint lane_id,
    const std::vector<DirectedWaypoint> &points,
    const api::RBounds& lane_bounds,
    const api::RBounds& driveable_bounds);

  /// If you have a segment with multiple lanes, you should call this function
  /// so as to build the lanes. It will create an extra control point on a
  /// lane when a not waypoint appears on another. @p segment_id is the id of
  /// the segment and @p lanes is a pointer to a vector that contains all the
  /// information regarding the lanes.
  /// It may throw a std::runtime exception in case there is a bad pointer
  /// reference or any of the conditions of the internal API calls is not met.
  void CreateSegmentConnections(
    const uint segment_id,
    std::vector<ConnectedLane> *lanes);

  /// Loads a connections between two RNDF lanes based on a set of @p exit and
  /// @p entry ids that map to a specific waypoint location.
  /// @p lane_bounds and @p driveable_bounds are the necessary values for the
  /// @class Lane .
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

  /// Given a vector of @p waypoints, it will construct and
  /// @return ignition::math::Spline. If any of the id_ of the waypoints is not
  /// valid, it won't be taken into account.
  std::unique_ptr<ignition::math::Spline> CreateSpline(
    const std::vector<DirectedWaypoint> *waypoints);
  /// Given the waypoints of the lanes (@p lanes) and @p index which is the item
  /// from zero to pick on each vector and test the distance against all the
  /// other lanes. Basically, it will look for a waypoint which is first in the
  /// s direction and return a @return vector containing all the lane indixes
  /// that are at the same s length from the first one. In case all the lanes
  /// are at the same position, none of them is returned.
  std::vector<int> GetInitialLaneToProcess(
    std::vector<std::vector<DirectedWaypoint>> *lanes,
    const int index);
  /// It loads the tangents into each of the @p waypoints using
  /// @function CreateSpline API.
  void BuildTangentsForWaypoints(
    std::vector<DirectedWaypoint> *waypoints);
  /// It computes a vector that joins @p base to @p destiny and then computes
  /// the projection of it against @p base tangent. It @return a double with
  /// that value
  double ComputeDistance(
    const DirectedWaypoint &base, const DirectedWaypoint &destiny);
  /// It checks if we need to add a dummy or an interpolated waypoint on the
  /// @p lanes given the @p ids of the lanes that have for @p index a control
  /// point before the others.
  void AddWaypointIfNecessary(
    const std::vector<int> &ids,
    std::vector<std::vector<DirectedWaypoint>> *lanes,
    const int index);
  void CreateNewControlPointsForLanes(
    std::vector<std::vector<DirectedWaypoint>> *lanes);


  api::RBounds lane_bounds_;
  api::RBounds driveable_bounds_;
  double linear_tolerance_{};
  double angular_tolerance_{};
  std::vector<std::unique_ptr<Connection>> connections_;
  std::map<std::string, DirectedWaypoint> directed_waypoints_;
  static const double kWaypointDistancePhase;
  static const double kLinearStep;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
