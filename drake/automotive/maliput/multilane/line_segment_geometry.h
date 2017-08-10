#pragma once

#include <cmath>
#include <utility>

#include "drake/automotive/maliput/multilane/segment_geometry.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"

namespace drake {
namespace maliput {
namespace multilane {

/// SegmentGeometry specification for a reference curve that describes a line.
class LineGeometry : public SegmentGeometry {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LineGeometry)

  /// Constructor. Computes a line from @p xy0 as the initial point of the line
  /// and @p dxy as the difference vector that connects the @p xy0 with the end
  /// point of the reference curve.
  /// @param xy0 A 2D vector that represents the first point of the lane.
  /// @param dxy A 2D difference vector between the last point and @p xy0.
  /// @param elevation CubicPolynomial object that represents the elevation
  /// polynomial. Note that coefficients should be scaled to match the path
  /// length integral of the reference curve.
  /// @param superelevation CubicPolynomial object that represents the
  /// superelevation polynomial. Note that coefficients should be scaled to
  /// match the path length integral of the reference curve.
  explicit LineGeometry(const Vector2<double>& xy0, const Vector2<double>& dxy,
                        const CubicPolynomial& elevation,
                        const CubicPolynomial& superelevation)
      : SegmentGeometry(elevation, superelevation),
        p0_(xy0),
        dp_(dxy),
        heading_(std::atan2(dxy.y(), dxy.x())) {}

  ~LineGeometry() override = default;

  /// Computes the interpolation of the position at @p p, which is a scale
  /// factor of the constructor's difference vector dxy.
  /// @param p Reference curve parameter.
  /// @return A 2D vector that represents the world position of the curve at
  /// @p p.
  Vector2<double> xy_of_p(double p) const override { return p0_ + p * dp_; }

  /// Computes the derivative of the position at @p p. As this class represents
  /// a line, it has a constant derivative given by the constructor's difference
  /// vector dxy.
  /// @param p Referece curve parameter.
  /// @return The constructor's difference vector dxy.
  Vector2<double> xy_dot_of_p(double p) const override {
    unused(p);
    return dp_;
  }

  /// Computes the heading of the tangent at @p p. Since the line has a constant
  /// heading for all the value of @p p, it is calculated in the constructor.
  /// @param p Reference curve parameter.
  /// @return The heading of the reference curve.
  double heading_of_p(double p) const override {
    unused(p);
    return heading_;
  }

  /// Since the line has a constant heading for all p values, the derivative of
  /// the heading with respect to p is zero.
  /// @param p Reference curve parameter.
  /// @return 0.0 for all values of @p p.
  double heading_dot_of_p(double p) const override {
    unused(p);
    return 0.;
  }

  /// Computes the path length of the reference curve along the interval of
  /// p = [0;1].
  /// @return The path length of the reference curve.
  double length() const override { return dp_.norm(); }

  /// Computes the coordinate frame conversion from world coordinates to
  /// composed curve frame (the same as api::LanePosition describes) for the
  /// line reference curve, elevation and superelevation polynomials
  /// combination.
  /// @param geo_coordinate A 3D vector in the world frame to be converted to
  /// the composed curve frame.
  /// @param lateral_bounds A pair that represents the lateral bounds of
  /// the surface mapping. The first item in the pair is the minimum value and
  /// the second is the maximum value in the lateral direction over the
  /// composed curve.
  /// @param height_bounds A pair that represents the height bounds of
  /// the surface mapping. The first item in the pair is the minimum value and
  /// the second is the maximum value in the vertical direction over the
  /// composed curve.
  /// @return A 3D vector that represents the coordinates with respect to the
  /// composed curve. The first dimension represents the path length coordinate,
  /// the second dimension is the lateral deviation from the composed curve and
  /// the third one is the vertical deviation from the composed curve too. The
  /// frame where this vector is defined is the same as api::LanePosition.
  /// @throws std::runtime_error When @p lateral_bounds first value is bigger
  /// than the second one.
  /// @throws std::runtime_error When @p height_bounds first value is bigger
  /// than the second one.
  Vector3<double> ToCurveFrame(
      const Vector3<double>& geo_coordinate,
      const std::pair<double, double>& lateral_bounds,
      const std::pair<double, double>& height_bounds) const override;

  /// As the reference curve is a line, the curvature radius is infinitum at any
  /// point in the range of p = [0;1] so no value of elevation or superelevation
  /// may derive in a geometry that intersects with itself.
  /// @param lateral_bounds A pair that represents the lateral bounds of
  /// the surface mapping. The first item in the pair is the minimum value and
  /// the second is the maximum value in the lateral direction over the
  /// composed curve.
  /// @param height_bounds A pair that represents the height bounds of
  /// the surface mapping. The first item in the pair is the minimum value and
  /// the second is the maximum value in the vertical direction over the
  /// composed curve.
  /// @return True.
  /// @throws std::runtime_error When @p lateral_bounds first value is bigger
  /// than the second one.
  /// @throws std::runtime_error When @p height_bounds first value is bigger
  /// than the second one.
  bool IsValid(
      const std::pair<double, double>& lateral_bounds,
      const std::pair<double, double>& height_bounds) const override {
    DRAKE_THROW_UNLESS(lateral_bounds.first < lateral_bounds.second);
    DRAKE_THROW_UNLESS(height_bounds.first < height_bounds.second);
    return true;
  }

 private:
  // The first point in world coordinates over the z=0 plane of the reference
  // curve.
  const Vector2<double> p0_{};
  // The difference vector that joins the end point of the reference curve with
  // the first one, p0_.
  const Vector2<double> dp_{};
  // The constant angle deviation of dp_ with respect to the x axis of the world
  // frame.
  const double heading_{};
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
