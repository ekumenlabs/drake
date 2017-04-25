#pragma once

#include <memory>
#include <vector>
#include <utility>

#include "ignition/math/Spline.hh"
#include "ignition/math/Vector3.hh"

#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace rndf {

/// An interpolator for inverse arc length functions
/// of arbitrary ignition::math::Splines. Helpful for arc length
/// parameterization.
class InverseArcLengthInterpolator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseArcLengthInterpolator)

  /// Constructor that takes the number of segments to
  /// use for piecewise interpolation.
  /// @param[in] _num_of_segments a positive integer describing
  /// the amount of segments to be used for piecewise interpolation
  /// @throws whenever arguments constrains are not
  /// satisfied.
  explicit InverseArcLengthInterpolator(const int _num_of_segments);

  /// Fits this interpolator to match the inverse of
  /// the given spline arc length function @f$ s(t) @f$.
  /// @param[in] _spline to interpolate for
  void Fit(const ignition::math::Spline& _spline);

  /// Interpolates @f$ t(s) @f$, that is, the inverse arc length
  /// function fitted.
  /// @param[in] _mth a positive plus 0 integer describing
  /// the order of the function derivative to interpolate.
  /// @param[in] _s arc length to interpolate at, constrained.
  /// by the curve dimensions [0, arc_length].
  /// @return interpolated mth derivative @f$ t^m(s) @f$ .
  /// @throws whenever arguments constrains are not
  /// satisfied.
  double InterpolateMthDerivative(const int _mth,
                                  const double _s) const;

 private:
  std::vector<double> s_t_;  ///< Arc length function s(t) as a table.
  double dt_;  ///< Step value in the independent variable t.
};

/// An extension for ignition::math::Splines that reparameterizes
/// them by arc length.
class ArcLengthParameterizedSpline {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ArcLengthParameterizedSpline)

  /// Constructor that takes a spline and a number of segments
  /// for arc length parameterization approximations.
  /// @param[in] _spline to interpolate for.
  /// @param[in] _num_of_segments a positive integer describing
  /// the amount of segments to be used for piecewise interpolation.
  /// @throws whenever arguments constrains are not satisfied.
  explicit ArcLengthParameterizedSpline(
    std::unique_ptr<ignition::math::Spline> _spline,
    const int _num_of_segments);

  /// Interpolates @f$ Q(s) @f$, that is, the spline parameterized
  /// by arc length.
  /// @param[in] _mth a positive plus 0 integer describing
  /// the order of the function derivative to interpolate.
  /// @param[in] _s arc length to interpolate at, constrained.
  /// by the curve dimensions [0, arc_length].
  /// @return the mth derivative Q^m(s)
  /// @throws whenever arguments constrains are not satisfied.
  ignition::math::Vector3d InterpolateMthDerivative(
      const int _mth, const double _s) const;

  /// Returns a mutable pointer to the underlying spline
  inline ignition::math::Spline* BaseSpline() {
    return this->q_t_.get();
  }

 private:
  std::unique_ptr<ignition::math::Spline> q_t_;  ///< Parameterized curve Q(t).
  InverseArcLengthInterpolator t_s_;  ///< Inverse arc length function t(s).
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
