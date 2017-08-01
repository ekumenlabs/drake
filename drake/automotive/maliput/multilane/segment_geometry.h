#pragma once

#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"

namespace drake {
namespace maliput {
namespace multilane {

/// A cubic polynomial, f(p) = a + b*p + c*p^2 + d*p^3.
class CubicPolynomial {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CubicPolynomial)

  /// Default constructor, all zero coefficients.
  CubicPolynomial() : CubicPolynomial(0., 0., 0., 0.) {}

  /// Constructs a cubic polynomial given all four coefficients.
  CubicPolynomial(double a, double b, double c, double d)
      : a_(a), b_(b), c_(c), d_(d) {
    const double df = f_p(1.) - f_p(0.);
    s_1_ = std::sqrt(1. + (df * df));
  }

  // Returns the a coefficient.
  double a() const { return a_; }

  // Returns the b coefficient.
  double b() const { return b_; }

  // Returns the c coefficient.
  double c() const { return c_; }

  // Returns the d coefficient.
  double d() const { return d_; }

  /// Evaluates the polynomial f at @p p.
  double f_p(double p) const {
    return a_ + (b_ * p) + (c_ * p * p) + (d_ * p * p * p);
  }

  /// Evaluates the derivative df/dp at @p p.
  double f_dot_p(double p) const {
    return b_ + (2. * c_ * p) + (3. * d_ * p * p);
  }

  /// Evaluates the double-derivative d^2f/dp^2 at @p p.
  double f_ddot_p(double p) const { return (2. * c_) + (6. * d_ * p); }

  // TODO(maddog@tri.global)  s_p() and p_s() need to be replaced with a
  //                          properly integrated path-length parameterization.
  //                          For the moment, we are calculating the length by
  //                          approximating the curve with a single linear
  //                          segment from (0, f(0)) to (1, f(1)), which is
  //                          not entirely awful for relatively flat curves.
  /// Returns the path-length s along the curve (p, f(p)) from p = 0 to @p p.
  double s_p(double p) const { return s_1_ * p; }

  /// Returns the inverse of the path-length parameterization s_p(p).
  double p_s(double s) const { return s / s_1_; }

  // TODO(maddog@tri.global) Until s(p) is a properly integrated path-length
  //                         parameterization, we have a need to calculate the
  //                         derivative of the actual linear function
  //                         involved in our bogus path-length approximation.
  double fake_gprime(double p) const {
    unused(p);
    // return df;  which is...
    return f_p(1.) - f_p(0.);
  }

 private:
  double a_{};
  double b_{};
  double c_{};
  double d_{};
  double s_1_{};
};

/// Defines an interface for a surface in a Segment object. The surface is
/// defined by an elevation and superelevation CubicPolynomial objects and a
/// reference curve. This reference curve is a C1 function over the z=0 plane.
/// Its domain is constrained in [0;1] interval and it should map a ℝ^2 curve.
/// As per notation, p is the parameter of the reference curve, and function
/// interpolations and function derivatives as well as headings and heading
/// derivatives are expressed in world coordinates, which is the same frame as
/// api::GeoPosition.
/// By implementing the interface the reference curve is defined and a complete
/// description of the segment surface is provided.
class SegmentGeometry {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SegmentGeometry)

  /// Constructor.
  /// @param elevation CubicPolynomial object that represents the elevation
  /// polynomial. Note that coefficients should be scaled to match the path
  /// length integral of the reference curve.
  /// @param superelevation CubicPolynomial object that represents the
  /// superelevation polynomial. Note that coefficients should be scaled to
  /// match the path length integral of the reference curve.
  SegmentGeometry(const CubicPolynomial& elevation,
                  const CubicPolynomial& superelevation)
      : elevation_(elevation), superelevation_(superelevation) {}

  virtual ~SegmentGeometry() = default;

  const CubicPolynomial& elevation() const { return elevation_; }

  const CubicPolynomial& superelevation() const { return superelevation_; }

  /// Computes the composed curve path integral in the interval of p = [0; 1].
  /// @return The path length integral of the curve composed with the elevation
  /// polynomial.
  double trajectory_length() const {
    return elevation_.s_p(1.0) * length();
  }

  /// Computes the interpolation of the reference curve.
  /// @param p The reference curve parameter.
  /// @return The reference curve itself, F(p).
  virtual Vector2<double> xy_of_p(double p) const = 0;

  /// Computes the first derivative interpolation of the reference curve.
  /// @param p The reference curve parameter.
  /// @return The derivative of the curve with respect to p, at @p p, i.e.,
  /// F'(p0) = (dx/dp, dy/dp) at p0.
  virtual Vector2<double> xy_dot_of_p(double p) const = 0;

  /// Computes the heading interpolation of the reference curve.
  /// @param p The reference curve parameter.
  /// @return The heading of the curve at @p p, i.e.,
  /// the angle of the tangent vector (with respect to x-axis) in the
  /// increasing-p direction.
  virtual double heading_of_p(double p) const = 0;

  /// Computes the first derivative heading interpolation of the reference
  /// curve.
  /// @param p The reference curve parameter.
  /// @return The derivative of the heading with respect to p, i.e.,
  /// d_heading/dp evaluated at @p p.
  virtual double heading_dot_of_p(double p) const = 0;

  /// Computes the path length integral of the reference curve for the interval
  /// [0;1] of p.
  /// @return The path length integral of the reference curve.
  virtual double length() const = 0;

  /// Converts a @p geo_coordinate in the world frame to the composed curve
  /// frame, i.e., the superposition of the reference curve, elevation and
  /// superelevation polynomials. The resulting coordinates are saturated to
  /// @p lateral_bounds and @p height_bounds in the lateral and vertical
  /// directions over the composed curve trajectory. The path length coordinate
  /// is saturated in the interval [0; trajectory_length()].
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
  virtual Vector3<double> ToCurveFrame(
      const Vector3<double>& geo_coordinate,
      const std::pair<double, double>& lateral_bounds,
      const std::pair<double, double>& height_bounds) const = 0;

  /// Checks if the surface does not fold due to the combination of elevation
  /// and superelevation polynomials. @p lateral_bounds and @p heigh_bounds are
  /// considered over the composed curve to derive the critical points to check.
  /// @param lateral_bounds A pair that represents the lateral bounds of
  /// the surface mapping. The first item in the pair is the minimum value and
  /// the second is the maximum value in the lateral direction over the
  /// composed curve.
  /// @param height_bounds A pair that represents the height bounds of
  /// the surface mapping. The first item in the pair is the minimum value and
  /// the second is the maximum value in the vertical direction over the
  /// composed curve.
  /// @return True when the surface does not fold.
  virtual bool IsValid(
      const std::pair<double, double>& lateral_bounds,
      const std::pair<double, double>& height_bounds) const = 0;

 private:
  // A polynomial that represents the elevation change as a function of p.
  CubicPolynomial elevation_;
  // A polynomial that represents the superelevation angle change as a function
  // of p.
  CubicPolynomial superelevation_;
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
