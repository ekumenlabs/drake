#pragma once

#include <cmath>

#include "drake/automotive/maliput/rndf/lane.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

#include "ignition/math/Spline.hh"

#include "spline_helpers.h"
#include "parameters.h"

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
          const api::RBounds& driveable_bounds);

  ~SplineLane() override = default;

  static double ComputeLength(
    const std::vector<std::tuple<ignition::math::Vector3d,
    ignition::math::Vector3d>> &points);

 private:
  api::LanePosition DoToLanePosition(
      const api::GeoPosition& geo_pos,
      api::GeoPosition* nearest_point,
      double* distance) const override;
  api::GeoPosition DoToGeoPosition(
      const api::LanePosition& lane_pos) const override;
  api::Rotation DoGetOrientation(
      const api::LanePosition& lane_pos) const override;
  api::LanePosition DoEvalMotionDerivatives(
      const api::LanePosition& position,
      const api::IsoLaneVelocity& velocity) const override;

  V2 xy_of_s(const double s) const;
  V2 xy_dot_of_s(const double s) const;
  double heading_of_s(const double s) const;
  double heading_dot_of_s(const double s) const;
  Rot3 Rabg_of_s(const double s) const;
  V3 s_hat_of_srh(const double s, const double r, const double h,
                      const Rot3& Rabg) const;
  V3 W_prime_of_srh(const double s, const double r, const double h,
                        const Rot3& Rabg) const;

  double do_length() const override { return spline_->BaseSpline()->ArcLength(); }

  double module_p(const double _p) const;

  // ignition::math::Spline spline_;
  std::unique_ptr<ArcLengthParameterizedSpline> spline_;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake