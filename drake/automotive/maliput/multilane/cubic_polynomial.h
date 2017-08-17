#pragma once

#include <cmath>

#include "drake/automotive/maliput/multilane/c2_scalar_function.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/unused.h"

namespace drake {
namespace maliput {
namespace multilane {

/// A cubic polynomial, f(p) = a + b*p + c*p^2 + d*p^3.
template<typename T> class CubicPolynomial : public C2ScalarFunction<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CubicPolynomial)

  /// Default constructor, all zero coefficients.
  CubicPolynomial() : CubicPolynomial(0., 0., 0., 0.) {}

  /// Constructs a cubic polynomial given all four coefficients.
  CubicPolynomial(T a, T b, T c, T d)
      : a_(a), b_(b), c_(c), d_(d) {
    const T df = f_p(T(1.)) - f_p(T(0.));
    s_1_ = std::sqrt(T(1.) + (df * df));
  }

  // Returns the a coefficient.
  T a() const { return a_; }

  // Returns the b coefficient.
  T b() const { return b_; }

  // Returns the c coefficient.
  T c() const { return c_; }

  // Returns the d coefficient.
  T d() const { return d_; }

  /// Evaluates the polynomial f at @p p.
  T f_p(T p) const override {
    return a_ + (b_ * p) + (c_ * p * p) + (d_ * p * p * p);
  }

  /// Evaluates the derivative df/dp at @p p.
  T f_dot_p(T p) const override {
    return b_ + (T(2.) * c_ * p) + (T(3.) * d_ * p * p);
  }

  /// Evaluates the double-derivative d^2f/dp^2 at @p p.
  T f_dot_dot_p(T p) const override {
    return (T(2.) * c_) + (T(6.) * d_ * p);
  }

  // TODO(maddog@tri.global)  s_p() and p_s() need to be replaced with a
  //                          properly integrated path-length parameterization.
  //                          For the moment, we are calculating the length by
  //                          approximating the curve with a single linear
  //                          segment from (0, f(0)) to (1, f(1)), which is
  //                          not entirely awful for relatively flat curves.
  /// Returns the path-length s along the curve (p, f(p)) from p = 0 to @p p.
  T s_p(T p) const override { return s_1_ * p; }

  /// Returns the inverse of the path-length parameterization s_p(p).
  T p_s(T s) const override { return s / s_1_; }

  // TODO(maddog@tri.global) Until s(p) is a properly integrated path-length
  //                         parameterization, we have a need to calculate the
  //                         derivative of the actual linear function
  //                         involved in our bogus path-length approximation.
  T fake_gprime(T p) const {
    unused(p);
    // return df;  which is...
    return f_p(T(1.)) - f_p(T(0.));
  }

  void scale(T scale_0, T scale_1) override {
    const T d_y{ (f_p(T(1.0)) - f_p(T(0.0))) * scale_0 };
    a_ = a_ * scale_0 / scale_1;
    // b_ = b_;
    c_ = c_ - T(3.0) * d_y / scale_0 + T(3.0) * d_y / scale_1,
    d_ = d_ + T(2.0) * d_y / scale_0 - T(2.0) * d_y / scale_1;
    const T df = f_p(T(1.)) - f_p(T(0.));
    s_1_ = std::sqrt(T(1.) + (df * df));
  }

 private:
  T a_{};
  T b_{};
  T c_{};
  T d_{};
  T s_1_{};
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
