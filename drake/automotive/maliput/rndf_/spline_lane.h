#pragma once

#include <cmath>
#include <memory>
#include <tuple>
#include <vector>
#include <utility>

#include "drake/automotive/maliput/rndf/lane.h"
#include "drake/automotive/maliput/rndf/spline_helpers.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

#include "ignition/math/Spline.hh"


namespace drake {
namespace maliput {
namespace rndf {

/// Specialization of Lane with a spline segment as its reference curve
/// in the xy-plane (the ground plane) of the world frame.
class SplineLane : public Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SplineLane)

  /// Constructs a SplineLane, a lane specified by a spline segment defined in
  /// the xy-plane (the ground plane) of the world frame.
  ///
  /// @param control_points are a collection of points and the tangent value
  ///        where the interpolated curve will pass. They must be at least two.
  /// @param id,segment,lane_bounds,driveable_bounds
  ///        See documentation for the Lane base class.
  ///
  /// This implementation uses ignition::math::Spline and
  /// @class ArcLengthParameterizedSpline which is used as the inverse function
  /// that maps the s parameter of s,r,h frame to ignition's Spline t parameter.
  SplineLane(const api::LaneId& id, const api::Segment* segment,
          const std::vector<std::tuple<ignition::math::Vector3d,
            ignition::math::Vector3d>> &control_points,
          const api::RBounds& lane_bounds,
          const api::RBounds& driveable_bounds);

  ~SplineLane() override = default;

  /// It computes the length of a SplineLane based on the @p points set as
  /// control points. The first value are the points and the second is the
  /// value at that point.
  static double ComputeLength(
    const std::vector<std::tuple<ignition::math::Vector3d,
    ignition::math::Vector3d>> &points);

  /// It returns the tension of the curve, a bounded constant value between 0 to
  /// 1 which is a measure of the curvature of the interpolated spline. Given a
  /// bigger value of the tension, you'll get an interpolation more similar to a
  /// straight line.
  static double Tension() { return kTension; }

  /// It returns the error bound that the arc length interpolator will
  /// attempt to attain when aproximating the inverse function that maps
  /// the s coordinate of s,r,h frame into the t parameter that
  /// ignition::math::Spline uses to evaluate the function.
  static double SplineErrorBound() { return kSplineErrorBound; }

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

  /// It will provide the x,y coordinates based on the
  /// @class ArcLengthParameterizedSpline that will provide the interpolation
  /// image at a arc length @p from the beginning of the SplineLane.
  V2 xy_of_s(const double s) const;

  /// It will provide the x,y tangent values based on the
  /// @class ArcLengthParameterizedSpline that will provide the interpolation
  /// image at a arc length @p from the beginning of the SplineLane.
  V2 xy_dot_of_s(const double s) const;

  /// It will provide the angle of the tangent vector evaluated at an arc lenght
  /// @p s distance from the beginning of the SplineLane.
  double heading_of_s(const double s) const;

  /// It will provide the derivative of the angle evaluates at at an arc lenght
  /// @p s distance from the beginning of the SplineLane. Given that x, y and z
  /// components of ignition::math::Spline are independant from each other and a
  /// a function of @p s (given the inverse function approximation provided by
  /// @class ArcLengthParameterizedSpline) we can apply the chain rule and
  /// the obtain derivative of the heading.
  double heading_dot_of_s(const double s) const;

  /// It will provide a rotation vector in terms of Euler angles, with only
  /// yaw angle set as RNDF is defined without any change in elevation.
  /// @see heading_of_s for more information.
  Rot3 Rabg_of_s(const double s) const;

  // Returns the s-axis unit-vector, expressed in the world frame,
  // of the (s,r,h) LANE-space frame (with respect to the world frame).
  //
  // (@p Rabg must be the result of Rabg_of_s(s) --- passed in here to
  // avoid recomputing it.)
  V3 s_hat_of_srh(const double s, const double r, const double h,
                      const Rot3& Rabg) const;

  // Returns W' = ∂W/∂s, the partial differential of W with respect to s,
  // evaluated at @p s, @p r, @p h.
  //
  // (@p Rabg must be the result of Rabg_of_s(s) --- passed in here to
  // avoid recomputing it.)
  V3 W_prime_of_srh(const double s, const double r, const double h,
                        const Rot3& Rabg) const;

  /// It returns the length of the curve.
  double do_length() const override {
    return spline_->BaseSpline()->ArcLength();
  }

  std::unique_ptr<ArcLengthParameterizedSpline> spline_;

  static const double kSplineErrorBound;
  static const double kTension;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
