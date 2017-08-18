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

template<typename T> class Elevation : public C2ScalarFunction<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Elevation)

  Elevation(T r,
            const CubicPolynomial<T>& reference_elevation,
            const CubicPolynomial<T>& reference_superelevation) :
      r_(r), reference_elevation_(reference_elevation),
      reference_superelevation_(reference_superelevation) {
    UpdateScale();
  }

  ~Elevation() = default;

  T f_p(T p) const override {
    return reference_elevation_.f_p(p) +
           r_ * std::tan(reference_superelevation_.f_p(p));
  }

  T f_dot_p(T p) const override {
    const T superelev_cos{std::cos(reference_superelevation_.f_p(p))};
    return reference_elevation_.f_dot_p(p) +
           r_ * reference_superelevation_.f_dot_p(p) /
           (superelev_cos * superelev_cos);
  }

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

  T s_p(T p) const override {
    return s_1_ * p;
  }

  T p_s(T s) const override {
    return s / s_1_;
  }

  void scale(T scale_0, T scale_1) override {
    reference_elevation_.scale(scale_0, scale_1);
    reference_superelevation_.scale(scale_0, scale_1);
    UpdateScale();
  }

  const CubicPolynomial<T>& reference_elevation() const {
    return reference_elevation_;
  }

  const CubicPolynomial<T>& reference_superelevation() const {
    return reference_superelevation_;
  }

  T r() const { return r_; }

  void set_r(T r) {
    r_ = r;
    UpdateScale();
  }

 private:
  void UpdateScale() {
    const T df = f_p(T(1.)) - f_p(T(0.));
    s_1_ = std::sqrt(T(1.) + (df * df));
  }

  T r_{};
  T s_1_{};
  CubicPolynomial<T> reference_elevation_;
  CubicPolynomial<T> reference_superelevation_;
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
