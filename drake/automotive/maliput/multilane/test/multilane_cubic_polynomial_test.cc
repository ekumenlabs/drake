/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/cubic_polynomial.h"
/* clang-format on */

#include <gtest/gtest.h>

namespace drake {
namespace maliput {
namespace multilane {

GTEST_TEST(MultilaneCubicPolynomial, GetterTest) {
  const double kA = 1.0;
  const double kB = 2.0;
  const double kC = 3.0;
  const double kD = 4.0;
  const CubicPolynomial<double> polynomial(kA, kB, kC, kD);
  EXPECT_DOUBLE_EQ(polynomial.a(), kA);
  EXPECT_DOUBLE_EQ(polynomial.b(), kB);
  EXPECT_DOUBLE_EQ(polynomial.c(), kC);
  EXPECT_DOUBLE_EQ(polynomial.d(), kD);
}

GTEST_TEST(MultilaneCubicPolynomial, InterpolationTest) {
  const double kA = 1.0;
  const double kB = 2.0;
  const double kC = 3.0;
  const double kD = 4.0;
  const CubicPolynomial<double> polynomial(kA, kB, kC, kD);
  const std::vector<double> kPVector {0.0, 0.1, 0.2, 0.5, 0.7, 1.0};
  for (const double p : kPVector) {
    EXPECT_DOUBLE_EQ(polynomial.f_p(p),
                     (kA + kB * p + kC * p * p + kD * p * p * p));
  }
}

GTEST_TEST(MultilaneCubicPolynomial, FirstDerivativeTest) {
  const double kA = 1.0;
  const double kB = 2.0;
  const double kC = 3.0;
  const double kD = 4.0;
  const CubicPolynomial<double> polynomial(kA, kB, kC, kD);
  const std::vector<double> kPVector {0.0, 0.1, 0.2, 0.5, 0.7, 1.0};
  for (const double p : kPVector) {
    EXPECT_DOUBLE_EQ(polynomial.f_dot_p(p),
                     (kB + 2.0 * kC * p + 3.0 * kD * p * p));
  }
}

GTEST_TEST(MultilaneCubicPolynomial, SecondDerivativeTest) {
  const double kA = 1.0;
  const double kB = 2.0;
  const double kC = 3.0;
  const double kD = 4.0;
  const CubicPolynomial<double> polynomial(kA, kB, kC, kD);
  const std::vector<double> kPVector {0.0, 0.1, 0.2, 0.5, 0.7, 1.0};
  for (const double p : kPVector) {
    EXPECT_DOUBLE_EQ(polynomial.f_dot_dot_p(p), (2.0 * kC + 6.0 * kD * p));
  }
}

GTEST_TEST(MultilaneCubicPolynomial, PathLengthTest) {
  const double kA = 1.0;
  const double kB = 2.0;
  const double kC = 3.0;
  const double kD = 4.0;
  const CubicPolynomial<double> polynomial(kA, kB, kC, kD);
  const std::vector<double> kPVector {0.0, 0.1, 0.2, 0.5, 0.7, 1.0};
  // Computes the scale factor to convert the polynomial's parameter to the
  // path length distance.
  const double kF_p_Init = polynomial.f_p(0.0);
  const double kF_p_End = polynomial.f_p(1.0);
  const double kPathScale = std::sqrt(
      1 + (kF_p_End - kF_p_Init) * (kF_p_End - kF_p_Init));
  for (const double p : kPVector) {
    EXPECT_DOUBLE_EQ(polynomial.s_p(p), (p * kPathScale));
  }
}

GTEST_TEST(MultilaneCubicPolynomial, InversePathLengthFunctionTest) {
  const double kA = 1.0;
  const double kB = 2.0;
  const double kC = 3.0;
  const double kD = 4.0;
  const CubicPolynomial<double> polynomial(kA, kB, kC, kD);
  // Computes the scale factor to convert the polynomial's parameter to the
  // path length distance.
  const double kF_p_Init = polynomial.f_p(0.0);
  const double kF_p_End = polynomial.f_p(1.0);
  const double kPathScale = std::sqrt(
      1 + (kF_p_End - kF_p_Init) * (kF_p_End - kF_p_Init));

  const std::vector<double> kPVector {0.0, 0.1, 0.2, 0.5, 0.7, 1.0};

  for (const double p : kPVector) {
    EXPECT_DOUBLE_EQ(polynomial.p_s(p * kPathScale), p);
  }
}
/*
GTEST_TEST(MultilaneCubicPolynomial, ScaleTest) {
  const double dX = 10.0;
  const double Y0 = 10.0;
  const double dY = 10.0;
  const double Y_dot_0 = 10.0;
  const double Y_dot_1 = 10.0;

  const double kA = Y0 / dX;
  const double kB = Y_dot_0;
  const double kC = 3.0 * dY / dX - 2.0 * Y_dot_0 - Y_dot_1;
  const double kD = Y_dot_0 + Y_dot_1 - 2.0 * dY / dX;
  CubicPolynomial<double> polynomial(kA, kB, kC, kD);
  polynomial.scale(dX, 2.0 * dX);
  EXPECT_DOUBLE_EQ(polynomial.a(), kA / 2.0);
  EXPECT_DOUBLE_EQ(polynomial.b(), kB);
  EXPECT_DOUBLE_EQ(polynomial.c(), kC - 3.0 * dY / dX + 3.0 * dY / (2.0 * dX));
  EXPECT_DOUBLE_EQ(polynomial.d(), kD + 2.0 * dY / dX - 2.0 * dY / (2.0 * dX));
  const double kF_p_Init = polynomial.f_p(0.0);
  const double kF_p_End = polynomial.f_p(1.0);
  const double kPathScale = std::sqrt(
      1 + (kF_p_End - kF_p_Init) * (kF_p_End - kF_p_Init));
  EXPECT_DOUBLE_EQ(polynomial.s_p(1.0), kPathScale);
}
*/
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
