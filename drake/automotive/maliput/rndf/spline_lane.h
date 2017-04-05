#pragma once

#include <cmath>

#include "drake/automotive/maliput/rndf/lane.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

#include <ignition/math.hh>

namespace drake {
namespace maliput {
namespace rndf {

typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> Point2;

class SplineLane : public Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SplineLane)


  SplineLane(const api::LaneId& id, const api::Segment* segment,
          const std::vector<Point2> &control_points,
          const double theta_i, const double theta_f,
          const api::RBounds& lane_bounds,
          const api::RBounds& driveable_bounds,
          const CubicPolynomial& elevation,
          const CubicPolynomial& superelevation);

  ~SplineLane() override = default;

  static Point2 IgnitionVector3d2Point2(
  	const ignition::math::Vector3d &point);

  static ignition::math::Vector3d Point22IgnitionVector3d(
    const Point2 &point);

  static std::vector<ignition::math::Vector3d> Points22IgnitionVector3ds(
    const std::vector<Point2> &points);

  static std::vector<Point2> IgnitionVector3ds2Points(
  	const std::vector<ignition::math::Vector3d> &points);

  static double ComputeLength(
    const std::vector<ignition::math::Vector3d> &points,
    std::vector<double> *lengths);

  static std::vector<ignition::math::Vector3d> InterpolateRoad(
    const std::vector<ignition::math::Vector3d> &_points,
    const double distance_threshold);

 private:
  api::LanePosition DoToLanePosition(
      const api::GeoPosition& geo_pos,
      api::GeoPosition* nearest_point,
      double* distance) const override;

  V2 xy_of_p(const double p) const override;
  V2 xy_dot_of_p(const double p) const override { return {0.0, 0.0}; };
  double heading_of_p(const double p) const override;
  double heading_dot_of_p(const double p) const override { return 0.0; };

  std::vector<ignition::math::Vector3d> control_points_;
  std::vector<double> lengths_;
  ignition::math::Spline spline_;
  double theta_i_;
  double theta_f_;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake