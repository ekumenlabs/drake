/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/arc_road_curve.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/automotive/maliput/multilane/cubic_polynomial.h"
#include "drake/automotive/maliput/multilane/test_utilities/multilane_brute_force_integral.h"

namespace drake {
namespace maliput {
namespace multilane {

GTEST_TEST(BruteForceIntegral, Arc) {
  const Vector2<double> kCenter{10.0, 10.0};
  const double kRadius{10.0};
  const double kTheta0{M_PI / 4.0};
  const double kTheta1{3.0 * M_PI / 4.0};
  const double kDTheta{kTheta1 - kTheta0};
  const CubicPolynomial zp(0., 0., 0., 0.);
  const double kP0{0.};
  const double kP1{1.};
  const double kR{0.};
  const double kH{0.};

  const ArcRoadCurve rc(kCenter, kRadius, kTheta0, kDTheta, zp, zp);
  EXPECT_NEAR(test::PathLengthIntegral(rc, kP0, kP1, kR, kH, 0),
              std::sin(M_PI / 4.) * kRadius * 2., 1e-12);
  EXPECT_EQ(
      test::MinimumKOrder(rc, kP0, kP1, kR, kH, 0.01 * rc.s_from_p(1., 0.), 0),
      2);
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
