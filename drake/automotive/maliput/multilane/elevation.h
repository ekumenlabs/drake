#pragma once

#include <cmath>
#include <memory>
#include <utility>

#include "drake/automotive/maliput/multilane/cubic_polynomial.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/unused.h"

namespace drake {
namespace maliput {
namespace multilane {

/// Wraps the math behind the composition of a reference elevation function and
/// a reference superelevation function with an offset r from where it is
/// defined. When the elevation and superelevation are defined over the same
/// reference path, there is no distortion of the height due to the lateral
/// distance but this is not true when there is a parallel reference path and
/// the superelevation polynomial is different from zero. For example,
/// considering the following cross section of a plane whose reference curve is
/// is a line over the z = 0 plane, reference_elevation function is zero along
/// the whole path but referece_superelevation function is a non-zero constant:
///
/// <pre>
///                  *--> h = 5 * R * tan(superelevation(p))
///               *-----> h = 4 * R * tan(superelevation(p))
///            *--------> h = 3 * R * tan(superelevation(p))
///         *-----------> h = 2 * R * tan(superelevation(p))
///      *--------------> h = R * tan(superelevation(p))
///   *_________________> h = 0 * tan(superelevation(p))
///   |  |  |  |  |  |
///   |  |  |  |  |  ---> r = 5 * R
///   |  |  |  |  ------> r = 4 * R
///   |  |  |  ---------> r = 3 * R
///   |  |  ------------> r = 2 * R
///   |  ---------------> r = R
///   ------------------> r = 0
///
/// </pre>
///
/// As it is depicted on the graph above, the height is a function of the
/// lateral distance form where the reference_elevation and
/// reference_superelevation functions are defined. This class will compute the
/// composed elevation with a constant lateral distance r. The composed function
/// will be:
///
/// composed_elevation(p) = reference_elevation(p) +
///                         r * tan(reference_superelevation(p))
template<typename T> class Elevation : public C2ScalarFunction<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Elevation)

  /// Constructor.
  /// Builds a composed elevation from the @p r lateral distance to the
  /// @p reference_elevation and @p reference_superelevation functions.
  /// @param r The lateral offset of this composed elevation with respect to
  /// where @p reference_elevation and @p reference_superelevation are defined.
  /// @param reference_elevation The function that defines the reference
  /// elevation profile.
  /// @param reference_superelevation The function that defines the reference
  /// superelevation profile.
  Elevation(T r,
            const CubicPolynomial<T>& reference_elevation,
            const CubicPolynomial<T>& reference_superelevation) :
      r_(r), reference_elevation_(reference_elevation),
      reference_superelevation_(reference_superelevation) {
    UpdateScale();
  }

  /// Destructor.
  ~Elevation() = default;

  /// Interpolates the composed elevation function at @p p parameter.
  /// Let CE(p) be the composed elevation function, RE(p) be the reference
  /// elevation function and SE(p) be the reference superelevation function.
  /// Consequently:
  ///
  /// CE(p) = RE(p) + r * tan(SE(p))
  /// @param p The parameter value of the function.
  /// @return The image of the function at @p p.
  T f_p(T p) const override {
    return reference_elevation_.f_p(p) +
           r_ * std::tan(reference_superelevation_.f_p(p));
  }

  /// Interpolates the composed elevation function's first derivative at @p p
  /// parameter.
  /// Let CE(p) be the composed elevation function, RE(p) be the reference
  /// elevation function and SE(p) be the reference superelevation function.
  /// Consequently:
  ///
  /// dCE(p) / dp = dRE(p) / dp + r * 1 / cos^(SE(p)) * dSE(p) / dp
  ///
  /// @param p The parameter value of the function.
  /// @return The image of the function's first derivative at @p p.
  T f_dot_p(T p) const override {
    const T superelev_cos{std::cos(reference_superelevation_.f_p(p))};
    return reference_elevation_.f_dot_p(p) +
           r_ * reference_superelevation_.f_dot_p(p) /
           (superelev_cos * superelev_cos);
  }

  /// Interpolates the function's second derivative at @p p parameter.
  /// Let CE(p) be the composed elevation function, RE(p) be the reference
  /// elevation function and SE(p) be the reference superelevation function.
  /// Consequently:
  ///
  /// d^2CE(p) / dp^2 = d^2RE(p) / dp^2 +
  ///                   r * cos(SE(p)) * [d^2SE(p) / dp^2 * cos(SE(p)) +
  ///                                     2 * (dSE(p) / dp)^2 * sin(SE(p))] /
  ///                   cos^4(SE(p))
  ///
  /// @param p The parameter value of the function.
  /// @return The image of the function's second derivative at @p p.
  T f_dot_dot_p(T p) const override {
    const T superelev_cos{std::cos(reference_superelevation_.f_p(p))};
    const T superelev_sin{std::sin(reference_superelevation_.f_p(p))};
    const T dot_superelev{reference_superelevation_.f_dot_p(p)};
    const T dot_dot_superelev{reference_superelevation_.f_dot_dot_p(p)};
    return reference_elevation_.f_dot_dot_p(p) +
           (dot_dot_superelev * superelev_cos +
             T{2.0} * dot_superelev * dot_superelev * superelev_sin) *
           r_ * superelev_cos / (std::pow(superelev_cos, T{4.0}));
  }

  /// Scales @p p by the distance between the points made by the composed
  /// function image and p parameter. Points are evaluated at p = 0 and
  /// at p = 1.0.
  /// @param p The parameter value to define the integration limit.
  /// @return The image of the function's second derivative at @p p.
  T s_p(T p) const override {
    return s_1_ * p;
  }

  /// Scales @p s by the inverse of the distance between the points made by the
  /// composed function image and p parameter. Points are evaluated at p = 0 and
  /// at p = 1.0.
  /// @param s The path length value.
  /// @return The inverse function image evaluated at @p s.
  T p_s(T s) const override {
    return s / s_1_;
  }

  const CubicPolynomial<T>& reference_elevation() const {
    return reference_elevation_;
  }

  const CubicPolynomial<T>& reference_superelevation() const {
    return reference_superelevation_;
  }

  T r() const { return r_; }

  /// Sets a new lateral distance of the composed elevation.
  /// A new scale factor is computed since the composed elevation is affected by
  /// the lateral distance.
  /// @param r The lateral offset of this composed elevation with respect to
  /// where @p reference_elevation and @p reference_superelevation are defined.
  void set_r(T r) {
    r_ = r;
    UpdateScale();
  }

 private:
  // Computes and updates the scale factor. The scale factor is equal to the
  // distance between the points made by the composed function image and p
  // parameter. Points are evaluated at p = 0 and at p = 1.0.
  void UpdateScale() {
    const T df = f_p(T(1.)) - f_p(T(0.));
    s_1_ = std::sqrt(T(1.) + (df * df));
  }

  // Lateral distance of the reference_elevation and reference_superelevation
  // functions.
  T r_{};
  // Scale factor.
  T s_1_{};
  // Reference elevation function.
  CubicPolynomial<T> reference_elevation_;
  // Reference superelevation function.
  CubicPolynomial<T> reference_superelevation_;
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
