#pragma once

#include <cmath>

#include "drake/automotive/maliput/rndf/lane.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

#include "ignition/math/Spline.hh"

#include "spline_helpers.h"

namespace drake {
namespace maliput {
namespace rndf {

typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> Point2;

class SplineLane : public Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SplineLane)


  SplineLane(const api::LaneId& id, const api::Segment* segment,
          const std::vector<std::tuple<ignition::math::Vector3d,
            ignition::math::Vector3d>> &control_points,
          const api::RBounds& lane_bounds,
          const api::RBounds& driveable_bounds,
          const CubicPolynomial& elevation,
          const CubicPolynomial& superelevation);

  ~SplineLane() override = default;

  static double ComputeLength(
    const std::vector<std::tuple<ignition::math::Vector3d,
    ignition::math::Vector3d>> &points);


 private:
  api::LanePosition DoToLanePosition(
      const api::GeoPosition& geo_pos,
      api::GeoPosition* nearest_point,
      double* distance) const override;

  V2 xy_of_p(const double p) const override;
  V2 xy_dot_of_p(const double p) const override;
  double heading_of_p(const double p) const override;
  double heading_dot_of_p(const double p) const override;

  double module_p(const double _p) const;

  void do_test() const;

  // ignition::math::Spline spline_;
  std::unique_ptr<ArcLengthParameterizedSpline> spline_;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake