#include <algorithm>

#include "drake/automotive/maliput/rndf/spline_helpers.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace rndf {

InverseArcLengthInterpolator::InverseArcLengthInterpolator(
    const int _num_of_segments)
{
  // Make sure the number of linear segments
  // used for approximation is a positive integer.
  DRAKE_THROW_UNLESS(_num_of_segments > 0);
  // Add one extra space to hold the end point.
  this->s_t_.resize(_num_of_segments + 1);
  this->dt_ = 1.0 / _num_of_segments;
}

void InverseArcLengthInterpolator::Fit(const ignition::math::Spline &_spline)
{
  for (uint i = 0 ; i < this->s_t_.size() ; ++i) {
    this->s_t_[i] = _spline.ArcLength(this->dt_ * i);
  }
}

double InverseArcLengthInterpolator::InterpolateMthDerivative(
    const int _mth, const double _s) const
{
  double __s = std::max(this->s_t_.front(), _s);
  __s = std::min(this->s_t_.back(), __s);

  // Make sure that the derivative order is a positive integer.
  DRAKE_THROW_UNLESS(_mth >= 0);
  // Make sure that the arc length requested is not below
  // s(t) image interval low bound.
  DRAKE_THROW_UNLESS(this->s_t_.front() <= __s);
  // Make sure that the arc length requested is not below
  // s(t) image interval upper bound.
  DRAKE_THROW_UNLESS(this->s_t_.back() >= __s);

  // Search s(t) function tabulation for lower bound index
  auto it = std::lower_bound(this->s_t_.begin(), this->s_t_.end(), __s);
  if (it != this->s_t_.begin()) it--;
  int index = it - this->s_t_.begin();

  // Zero out any derivative of higher order than
  // 1, as this is a piecewise linear interpolant.
  if (_mth > 1) return 0.0;

  double ds = this->s_t_[index + 1] - this->s_t_[index];

  // Return t'(s) at __s.
  if (_mth == 1) return this->dt_ / ds;

  double t_0 = index * this->dt_;

  // Return t(s) at __s.
  return t_0 + (this->dt_ / ds) * (__s - this->s_t_[index]);
}

ArcLengthParameterizedSpline::ArcLengthParameterizedSpline(
    std::unique_ptr<ignition::math::Spline>& _spline, const int _num_of_segments)
    : q_t_(std::move(_spline)), t_s_(_num_of_segments)
{
  DRAKE_ASSERT(this->q_t_ != nullptr);
  // Fit inverse arc length interpolator to the
  // given spline arc length
  t_s_.Fit(*this->q_t_);
}

ignition::math::Vector3d
ArcLengthParameterizedSpline::InterpolateMthDerivative(
    const int _mth, const double _s) const
{
  if (_mth > 3) {
    // M > 3 => p = 0 (as this is a cubic interpolator)
    return ignition::math::Vector3d(0.0, 0.0, 0.0);
  }
  double t_s = this->t_s_.InterpolateMthDerivative(0, _s);
  if (_mth < 1) {
    // M = 0 => P(s) = Q(t(s))
    return this->q_t_->InterpolateMthDerivative(0, t_s);
  }
  double t_prime_s = this->t_s_.InterpolateMthDerivative(1, _s);
  ignition::math::Vector3d q_prime_t = this->q_t_->InterpolateMthDerivative(1, t_s);
  if (_mth < 2) {
    // M = 1 => P'(s) = Q'(t(s)) * t'(s)
    return q_prime_t * t_prime_s;
  }
  double t_prime_s_2 = t_prime_s * t_prime_s;
  double t_prime2_s = this->t_s_.InterpolateMthDerivative(2, _s);
  ignition::math::Vector3d q_prime2_t = this->q_t_->InterpolateMthDerivative(2, t_s);
  if (_mth < 3) {
    // M = 2 => P''(s) = Q''(t(s)) * t'(s)^2 + Q'(t(s)) * t''(s)
    return q_prime2_t * t_prime_s_2 + q_prime_t * t_prime2_s;
  }
  double t_prime_s_3 = t_prime_s_2 * t_prime_s;
  double t_prime3_s = this->t_s_.InterpolateMthDerivative(3, _s);
  ignition::math::Vector3d q_prime3_t = this->q_t_->InterpolateMthDerivative(3, t_s);
  // M = 3 => P'''(s) = Q'''(t(s)) * t'(s)^3
  ///                   + 3 * Q''(t(s)) * t'(s) * t''(s)
  ///                   + Q'(t(s)) * t'''(s)
  return (q_prime3_t * t_prime_s_3
          + 3 * q_prime2_t * t_prime_s * t_prime2_s
          + q_prime_t * t_prime3_s);
}


}  // namespace rndf
}  // namespace maliput
}  // namespace drake
