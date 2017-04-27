#pragma once

#include <cmath>
#include <memory>
#include <vector>
#include <tuple>
#include <sstream>
#include <string>
#include <map>
#include <algorithm>

#include "ignition/math/Vector3.hh"
#include "ignition/math/SphericalCoordinates.hh"
#include "ignition/rndf/RNDF.hh"
#include "ignition/rndf/Segment.hh"
#include "ignition/rndf/Lane.hh"
#include "ignition/rndf/Waypoint.hh"
#include "ignition/rndf/UniqueId.hh"
#include "ignition/rndf/Exit.hh"

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/rndf/builder.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace rndf {

/// It has some basic and common characteristics of Maliput's API needed to
/// construct the road_geometry.

// TODO(@agalbachicar) Should use the characteristics of the RNDF parsing.
// See issue https://bitbucket.org/ekumen/terminus-simulation/issues/120
struct RoadCharacteristics {
  /// Constructor for using default road geometries.
  RoadCharacteristics() = default;

  /// Constructor for custom road geometries.
  RoadCharacteristics(const double lw, const double dw)
      : lane_width(lw), driveable_width(dw) {}

  const double lane_width{4.};
  const double driveable_width{8.};

  const api::RBounds lane_bounds{
    -lane_width / 2.,
    lane_width / 2.};
  const api::RBounds driveable_bounds{
    -driveable_width / 2.,
    driveable_width / 2.};
};

class Loader {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Loader)

  /// Constructor for the example.  The user supplies @p rc, a
  /// RoadCharacteristics structure that aggregates the road boundary data.
  explicit Loader(const RoadCharacteristics& rc) : rc_(rc) {}

  /// Constructor for the example, using default RoadCharacteristics settings.
  Loader() : Loader(RoadCharacteristics{}) {}

  /// It loads a file and calls ignition::rndf::RNDF class to parse it. From it,
  /// it will use the @class Builder class to build the @class RoadGeometry. The
  ///  @p file_name is used to locate a file.
  ///
  /// @throws It will throw an exception when there is any problem loading the
  ///         file_name provided.
  ///
  /// There are some internal checks to find at least one segment, lane and
  /// waypoints.
  ///
  /// @note We don't support RNDF Zones yet.
  /// @note The origin of the map will be the position of the waypoint labeled
  ///       as 1.1.1.
  std::unique_ptr<const maliput::api::RoadGeometry> LoadFile(
    const std::string &file_name);

 private:
  /// It's used to compute the global coordinates based on an @p origin vector
  /// (composed of latitude [degrees], longitude [degrees], elevation [meters]).
  /// @p spherical_position is the ignition::rndf::Waypoint::Location() result.
  /// @return A vector containing a vector
  ignition::math::Vector3d ToGlobalCoordinates(
    const ignition::math::Vector3d &origin,
    const ignition::math::SphericalCoordinates &spherical_position) const;

  /// It builds all the segments of RNDF road geometry. @p origin is used as a
  /// base location to convert all the spherical coordiantes to a global frame.
  /// @p segments it's a vector containing all the segments to build.
  void BuildSegments(
    const ignition::math::Vector3d &origin,
    const std::vector<ignition::rndf::Segment> &segments) const;

  /// It builds the connections from one lane to another once all the
  /// @p segments have already been finished through BuildSegments.
  void BuildConnections(
    const std::vector<ignition::rndf::Segment> &segments) const;

  /// Linear tolerance for RNDF @class RoadGeometry.
  const double linear_tolerance_  = 0.01;

  /// Angular tolerance for RNDF @class RoadGeometry.
  const double angular_tolerance_ = 0.01 * M_PI;

  const RoadCharacteristics rc_;

  std::unique_ptr<maliput::rndf::Builder> builder;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
