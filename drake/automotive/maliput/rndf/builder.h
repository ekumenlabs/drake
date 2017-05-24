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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DirectedWaypoint)

  DirectedWaypoint() = default;

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

  static std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>
    CalculateBoundingBox(
      const std::vector<DirectedWaypoint> &directed_waypoints);

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

  /// Constructs a spline-segment connection joining @p points[0] to
  /// @p points[size-1].
  Connection(const std::string& id,
    const std::vector<DirectedWaypoint>& waypoints,
    const double width) :
      id_(id),
      waypoints_(waypoints),
      width_(width) {
    DRAKE_THROW_UNLESS(waypoints_.size() >= 2);
  }

  /// Returns the ID string.
  const std::string& id() const { return id_; }

  /// Returns the parameters of the start point.
  const DirectedWaypoint& start() const {
    return waypoints_.front();
  }
  DirectedWaypoint& start() {
    return waypoints_.front();
  }

  /// Returns the parameters of the endpoint.
  const DirectedWaypoint& end() const {
    return waypoints_.back();
  }
  DirectedWaypoint& end() {
    return waypoints_.back();
  }

  const std::vector<DirectedWaypoint> &waypoints() const {
    return waypoints_;
  }

  double width() const {
    return width_;
  }

 private:
  std::string id_;
  std::vector<DirectedWaypoint> waypoints_;
  double width_;
};

/// This structure holds all the lane information needed at construction time.
struct ConnectedLane {
  std::vector<DirectedWaypoint> waypoints;
  bool inverse_direction{false};
  double width;
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
  Builder(const double linear_tolerance,
          const double angular_tolerance) :
    linear_tolerance_(linear_tolerance),
    angular_tolerance_(angular_tolerance) {}

  void SetBoundingBox(
    const std::tuple<ignition::math::Vector3d,
      ignition::math::Vector3d> &bounding_box) {
    bounding_box_ = bounding_box;
  }

  /// If you have a segment with multiple lanes, you should call this function
  /// so as to build the lanes. It will create an extra control point on a
  /// lane when a not waypoint appears on another. @p segment_id is the id of
  /// the segment and @p lanes is a pointer to a vector that contains all the
  /// information regarding the lanes.
  /// It may throw a std::runtime exception in case there is a bad pointer
  /// reference or any of the conditions of the internal API calls is not met.
  /// Also if the vector does not contain at least one lane or if the lanes do
  /// not contain at least two waypoints an exception is thrown.
  void CreateSegmentConnections(
    const uint segment_id,
    std::vector<ConnectedLane> *lanes);

  void CreateConnectionsForZones(const double width,
    std::vector<DirectedWaypoint> *perimeter_waypoints);

  /// Loads a connections between two RNDF lanes based on a set of @p exit and
  /// @p entry ids that map to a specific waypoint location.
  /// @p lane_bounds and @p driveable_bounds are the necessary values for the
  /// @class Lane .
  void CreateConnection(
    const double width,
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

  std::string BuildName(const ignition::rndf::UniqueId &id) const;

  void CreateLane(
    const std::string &key_id,
    const double width,
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
    Segment *segment,
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
    std::vector<ConnectedLane> *lanes,
    const int index);

  /// It loads the tangents into each of the @p waypoints using
  /// @function CreateSpline API.
  void BuildTangentsForWaypoints(
    std::vector<DirectedWaypoint> *waypoints);

  /// It computes a vector that joins @p base to @p destiny and then computes
  /// the projection of it against @p base tangent. It @return a double with
  /// that value
  double ComputeDistance(
    const DirectedWaypoint &base, const DirectedWaypoint &destiny) const;

  /// It checks if we need to add a dummy or an interpolated waypoint on the
  /// @p lanes given the @p ids of the lanes that have for @p index a control
  /// point before the others.
  void AddWaypointIfNecessary(
    const std::vector<int> &ids,
    std::vector<ConnectedLane> *lanes,
    const int index);

  /// It is the base function that wraps all the process of adding waypoints
  /// when necessary for the @p lanes.
  /// It will @throw std::runtime_error if @p lanes vector is nullptr or if any
  /// of the called functions constraints are not met.
  void CreateNewControlPointsForLanes(
    std::vector<ConnectedLane> *lanes);

  /// This function checks the list of lanes their waypoints (@p lane_waypoints)
  /// and copies all the waypoints in @p index position from @p lane_ids
  /// respective lanes. Then, it orders them following the direction of each
  /// control point from right to left. It will @return lane_ids with the
  /// correct order of the ids. In case @p lane_ids is nullptr or its size is 0
  /// it will @throw std::runtime_error. If the size of @p lane_ids just one, it
  /// will return without doing anything.
  void OrderLaneIds(
    std::vector<ConnectedLane> *lanes,
    std::vector<int> *lane_ids,
    const int index);

  double CalculateMomentum(const ignition::math::Vector3d &point,
    const DirectedWaypoint &wp);
  double CalculateRNDFLaneMomentum(
    const ignition::math::Vector3d &base_point,
    const std::vector<DirectedWaypoint> &wps);
  void SetInvertedLanes(
  std::vector<ConnectedLane> *lanes);

  ignition::math::Vector3d ConstructPointForLane(
    const DirectedWaypoint &base, const DirectedWaypoint &other_lane_base) const;
  std::vector<int> GetStartingLaneIds(std::vector<ConnectedLane> *lanes,
    const bool start_check) const;
  void GroupLanesByDirection(const std::vector<ConnectedLane> *lanes,
    std::map<int, std::vector<ConnectedLane>> *segments) const;

  double linear_tolerance_{};
  double angular_tolerance_{};
  std::map<std::string, std::vector<std::unique_ptr<Connection>>> connections_;
  std::map<std::string, DirectedWaypoint> directed_waypoints_;
  std::tuple<ignition::math::Vector3d, ignition::math::Vector3d> bounding_box_;
  static const double kWaypointDistancePhase;
  static const double kLinearStep;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
