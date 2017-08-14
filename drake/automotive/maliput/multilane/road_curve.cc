#include "drake/automotive/maliput/multilane/road_curve.h"

namespace drake {
namespace maliput {
namespace multilane {

CubicPolynomial<double> RoadCurve::ScaleCubicPolynomial(
    const CubicPolynomial<double>& cubic_polynomial,
    double scale_0,
    double scale_1) const {
  const double a = cubic_polynomial.a();
  const double b = cubic_polynomial.b();
  const double c = cubic_polynomial.c();
  const double d = cubic_polynomial.d();
  const double d_y = (cubic_polynomial.f_p(1.0) -
                      cubic_polynomial.f_p(0.0)) * scale_0;

  return CubicPolynomial<double>(a * scale_0 / scale_1,
                                 b,
                                 c - 3.0 * d_y / scale_0 + 3.0 * d_y / scale_1,
                                 d + 2.0 * d_y / scale_0 - 2.0 * d_y / scale_1);
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
