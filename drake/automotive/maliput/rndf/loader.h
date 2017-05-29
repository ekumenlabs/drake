#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <tuple>

#include "ignition/math/Vector3.hh"
#include "ignition/math/SphericalCoordinates.hh"
#include "ignition/rndf/RNDF.hh"
#include "ignition/rndf/Segment.hh"
#include "ignition/rndf/Lane.hh"
#include "ignition/rndf/Waypoint.hh"
#include "ignition/rndf/UniqueId.hh"
#include "ignition/rndf/Exit.hh"
#include "ignition/rndf/Zone.hh"
#include "ignition/rndf/Perimeter.hh"

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/rndf/builder.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace rndf {

/// It has some basic and common characteristics of Maliput's API needed to
/// construct the RoadGeometry
///
// TODO(@agalbachicar) Should use the characteristics of the RNDF parsing.
// See issue https://bitbucket.org/ekumen/terminus-simulation/issues/120
struct RoadCharacteristics {
  /// Constructor for using default road geometries.
  RoadCharacteristics() = default;

  /// Constructor for custom road geometries.
  ///
  /// @param default_width is the width used for those lanes whose width is not
  /// provided in the map.
  explicit RoadCharacteristics(const double default_width)
      : default_width_(default_width) {}

  const double default_width_{4.};
};

/// This is a wrapper that lets you load a RNDF map file and use the
/// ignition::rndf library to parse it and call Builder class to get the
/// RoadGeometry.
class Loader {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Loader)

  /// Constructor
  ///
  /// @param rc is the RoadCharacteristics object to work with.
  explicit Loader(const RoadCharacteristics& rc) : rc_(rc) {}

  /// Constructor. Using default RoadCharacteristics settings.
  Loader() : Loader(RoadCharacteristics{}) {}

  /// It loads a file and calls ignition::rndf::RNDF class to parse it. From it,
  /// it will use a Builder object to construct the RoadGeometry. The
  ///  @p file_name is used to locate a file.
  ///
  /// @param file_name is the path to the RNDF map file.
  /// @throws when there is any problem loading the file_name provided.
  ///
  /// There are some internal checks to find at least one segment, lane and
  /// waypoints.
  ///
  /// @note The origin of the map will be the position of the waypoint labeled
  ///       as 1.1.1.
  std::unique_ptr<const maliput::api::RoadGeometry> LoadFile(
    const std::string& file_name);

 private:
  // It's used to compute the global coordinates based on an origin vector
  // (composed of latitude [degrees], longitude [degrees], elevation [meters]).
  // spherical_position is the ignition::rndf::Waypoint::Location() result.
  // It returns a vector containing a vector
  ignition::math::Vector3d ToGlobalCoordinates(
    const ignition::math::Vector3d& origin,
    const ignition::math::SphericalCoordinates& spherical_position) const;

  // It builds all the segments of RNDF road geometry. origin is used as a
  // base location to convert all the spherical coordiantes to a global frame.
  // segments it's a vector containing all the segments to build.
  void BuildSegments(
    const ignition::math::Vector3d& origin,
    const std::vector<ignition::rndf::Segment>& segments) const;

  // It builds a vector of DirectedWaypoints from all the RNDF perimeter
  // waypoints that outline a zone. Then, it constructs with the Builder the
  // roads inside that zone.
  void BuildZoneLanes(
    const ignition::math::Vector3d& origin,
    const std::vector<ignition::rndf::Zone>& zones) const;

  /// It builds the connections from one lane to another once all the
  /// segments and zone connections have already been finished.
  void BuildConnections(
    const std::vector<ignition::rndf::Segment>& segments,
    const std::vector<ignition::rndf::Zone>& zones) const;

  // It builds the bounding box of the RNDF map getting the minimum coordinate
  // and the maximum from all the segment waypoint locations.
  void BuildBoundingBox(const ignition::math::Vector3d& origin,
    const std::vector<ignition::rndf::Segment>& segments) const;

  /// Linear tolerance for RNDF @class RoadGeometry.
  const double linear_tolerance_{0.01};

  /// Angular tolerance for RNDF @class RoadGeometry.
  const double angular_tolerance_{0.01 * M_PI};

  const RoadCharacteristics rc_;

  std::unique_ptr<maliput::rndf::Builder> builder;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
