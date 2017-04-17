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

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/rndf/junction.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {

namespace rndf {
class RoadGeometry;

/// @class Builder
/// Convenient builder class which makes it easy to construct a rndf road
/// network.
///
/// rndf is a simple road-network implementation:
///  - single lane per segment;
///  - constant lane_bounds and driveable_bounds, same for all lanes;
///  - only linear and constant-curvature-arc primitives in XY-plane;
///  - cubic polynomials (parameterized on XY-arc-length) for elevation
///    and superelevation;
///  - superelevation (bank of road) rotates around the reference line (r = 0)
///    of the path.
///
/// The Builder class simplifies the assembly of rndf road network
/// components into a valid RoadGeometry.  In the Builder model, an Endpoint
/// specifies a point in world coordinates (along with a direction, slope,
/// and superelevation parameters).  A Connection is a path from an explicit
/// start Endpoint to an end Endpoint calculated via a linear or arc
/// displacement (ArcOffset).  A Group is a collection of Connections.
///
/// Builder::Build() constructs a RoadGeometry.  Each Connection yields a
/// Segment bearing a single Lane.  Each Group yields a Junction containing
/// the Segments associated with the grouped Connections; ungrouped
/// Connections each receive their own Junction.


/// XY-plane-only parameters for an endpoint of a connection, specified in
/// the world frame.
///
/// The three components are:
///  - x: x position
///  - y: y position
///  - heading: heading of reference path (radians, zero == x-direction)
class EndpointXy {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EndpointXy)

  // Constructs an EndpointXy with all zero parameters.
  EndpointXy() = default;

  EndpointXy(double x, double y, double heading, double heading_mod)
      : x_(x), y_(y), heading_(heading), heading_mod_(heading_mod) {}

  /// Returns an EndpointXy with reversed direction.
  EndpointXy reverse() const {
    return EndpointXy(x_, y_,
                      std::atan2(-std::sin(heading_), -std::cos(heading_)),
                      heading_mod_);
  }

  double x() const { return x_; }

  double y() const { return y_; }

  double heading() const { return heading_; }

  double heading_mod() const { return heading_mod_; }

 private:
  double x_{};
  double y_{};
  double heading_{};
  double heading_mod_{};
};


/// Out-of-plane parameters for an endpoint of a connection, specified in
/// the world frame.
///
/// The four components are:
///  - z: elevation
///  - z_dot: grade (rate of change of elevation with respect to
///           arc length of the reference path)
///  - theta: superelevation (rotation of road surface around r = 0 centerline;
///           when theta > 0, elevation at r > 0 is above elevation at r < 0)
///  - theta_dot: rate of change of superelevation with respect to arc length
///               of the reference path
class EndpointZ {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EndpointZ)

  // Constructs an EndpointZ with all zero parameters.
  EndpointZ() = default;

  EndpointZ(double z, double z_dot, double theta, double theta_dot)
      : z_(z), z_dot_(z_dot), theta_(theta), theta_dot_(theta_dot) {}

  /// Returns an EndpointZ with reversed direction.
  EndpointZ reverse() const {
    return EndpointZ(z_, -z_dot_, -theta_, -theta_dot_);
  }

  double z() const { return z_; }

  double z_dot() const { return z_dot_; }

  double theta() const { return theta_; }

  double theta_dot() const { return theta_dot_; }

 private:
  double z_{};
  double z_dot_{};

  double theta_{};
  double theta_dot_{};
};


/// Complete set of parameters for an endpoint of a connection,
/// specified in the world frame.  It comprises two subsets of parameters:
/// those pertaining only to the xy ground-plane, and those pertaining to
/// out-of-plane aspects of an endpoint.
class Endpoint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Endpoint)

  // Constructs an Endpoint with all zero parameters.
  Endpoint() = default;

  Endpoint(const EndpointXy& xy, const EndpointZ& z) : xy_(xy), z_(z) {}

  /// Returns an Endpoint with reversed direction.
  Endpoint reverse() const {
    return Endpoint(xy_.reverse(), z_.reverse());
  }

  /// Returns the subset of parameters pertaining to the xy ground-plane.
  const EndpointXy& xy() const { return xy_; }

  /// Returns the subset of parameters pertaining to out-of-ground-plane
  /// aspects.
  const EndpointZ& z() const { return z_; }

 private:
  EndpointXy xy_;
  EndpointZ z_;
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

  /// Possible connection geometries:  line- or arc-segment.
  enum Type { kSpline };


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
  const Endpoint& start() const { return start_; }

  /// Returns the parameters of the endpoint.
  const Endpoint& end() const { return end_; }

  const std::vector<Endpoint> &points() const {
    DRAKE_DEMAND(type_ == kSpline);
    return points_;
  }

 private:
  Type type_{};
  std::string id_;
  Endpoint start_;
  Endpoint end_;
  std::vector<Endpoint> points_;
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

  const Connection* Connect(
      const std::string& id,
      const std::vector<Endpoint> &points);

  void CreateLaneConnections(
    const uint segment_id,
    const uint lane_id,
    const std::vector<ignition::math::Vector3d> &points);

  void CreateLaneToLaneConnection(
    const std::string &exit_id,
    const std::string &entry_id);

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


  /// Produces a RoadGeometry, with the ID @p id.
  std::unique_ptr<const api::RoadGeometry> Build(
      const api::RoadGeometryId& id) const;

 private:
  // EndpointFuzzyOrder is an arbitrary strict complete ordering of Endpoints
  // useful for, e.g., std::map.  It provides a comparison operation that
  // treats two Endpoints within @p linear_tolerance of one another as
  // equivalent.
  //
  // This is used to match up the endpoints of Connections, to determine
  // how Connections are linked to one another.  Exact numeric equality
  // would not be robust given the use of floating-point values in Endpoints.
  class EndpointFuzzyOrder {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EndpointFuzzyOrder)

    explicit EndpointFuzzyOrder(const double linear_tolerance)
        : lin_tol_(linear_tolerance) {}

    bool operator()(const Endpoint& lhs, const Endpoint& rhs) const {
      switch (fuzzy_compare(rhs.xy().x(), lhs.xy().x())) {
        case -1: { return true; }
        case 1: { return false; }
        case 0: {
          switch (fuzzy_compare(rhs.xy().y(), lhs.xy().y())) {
            case -1: { return true; }
            case 1: { return false; }
            case 0: {
              switch (fuzzy_compare(rhs.z().z(), lhs.z().z())) {
                case -1: { return true; }
                case 1: { return false; }
                case 0: { return false; }
                default: { DRAKE_ABORT(); }
              }
            }
            default: { DRAKE_ABORT(); }
          }
        }
        default: { DRAKE_ABORT(); }
      }
    }

   private:
    int fuzzy_compare(const double a, const double b) const {
      if (a < (b - lin_tol_)) {
        return -1;
      } else if (a > (b + lin_tol_)) {
        return 1;
      } else {
        return 0;
      }
    }

    double lin_tol_{};
  };

  struct DefaultBranch {
    DefaultBranch() = default;

    DefaultBranch(
        const Connection* ain, const api::LaneEnd::Which ain_end,
        const Connection* aout, const api::LaneEnd::Which aout_end)
        : in(ain), in_end(ain_end), out(aout), out_end(aout_end) {}

    const Connection* in{};
    api::LaneEnd::Which in_end{};
    const Connection* out{};
    api::LaneEnd::Which out_end{};
  };

  Lane* BuildConnection(
      const Connection* const cnx,
      Junction* const junction,
      RoadGeometry* const rg,
      std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* const bp_map) const;

  BranchPoint* FindOrCreateBranchPoint(
      const Endpoint& point,
      RoadGeometry* rg,
      std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* const bp_map) const;

  void AttachBranchPoint(
      const Endpoint& point, Lane* const lane, const api::LaneEnd::Which end,
      RoadGeometry* rg,
      std::map<Endpoint, BranchPoint*, EndpointFuzzyOrder>* bp_map) const;

  std::string BuildName(const uint segment_id,
    const uint lane_id) const;

  std::string BuildName(const uint segment_id,
    const uint lane_id,
    const uint waypoint_id) const;

  Endpoint ConvertIntoEndpoint(
    const std::tuple<ignition::math::Vector3d,
      ignition::math::Vector3d> &pose);

  api::RBounds lane_bounds_;
  api::RBounds driveable_bounds_;
  double linear_tolerance_{};
  double angular_tolerance_{};
  std::vector<std::unique_ptr<Connection>> connections_;
  std::vector<DefaultBranch> default_branches_;
  std::map<
    std::string,
    std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>> waypoints;
  std::vector<std::unique_ptr<Group>> groups_;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
