#pragma once

#include <cmath>
#include <memory>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/rndf/builder.h"

namespace drake {
namespace automotive {

struct RNDFRoadCharacteristics {
  /// Constructor for using default road geometries.
  RNDFRoadCharacteristics() = default;

  /// Constructor for custom road geometries.
  RNDFRoadCharacteristics(const double lw, const double dw)
      : lane_width(lw), driveable_width(dw) {}

  const double lane_width{4.};
  const double driveable_width{8.};

  const maliput::api::RBounds lane_bounds{-lane_width / 2., lane_width / 2.};
  const maliput::api::RBounds driveable_bounds{-driveable_width / 2.,
                                               driveable_width / 2.};
};

class RNDFTBuilder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RNDFTBuilder)

  /// Constructor for the example.  The user supplies @p rc, a
  /// RoadCharacteristics structure that aggregates the road boundary data.
  explicit RNDFTBuilder(const RNDFRoadCharacteristics& rc) : rc_(rc) {}

  /// Constructor for the example, using default RoadCharacteristics settings.
  RNDFTBuilder() : RNDFTBuilder(RNDFRoadCharacteristics{}) {}

  /// Implements the onramp example.
  std::unique_ptr<const maliput::api::RoadGeometry> Build();

 private:
  /// Tolerances for monolane's Builder.
  const double linear_tolerance_  = 0.01;
  const double angular_tolerance_ = 0.01 * M_PI;

  const RNDFRoadCharacteristics rc_;
};

}  // namespace automotive
}  // namespace drake