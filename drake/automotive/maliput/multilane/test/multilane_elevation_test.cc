/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/elevation.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/automotive/maliput/multilane/cubic_polynomial.h"

namespace drake {
namespace maliput {
namespace multilane {

GTEST_TEST(MultilaneElevation, GetterAndSetterTest) {
  const double kReferenceElevationA = 1.0;
  const double kReferenceElevationB = 2.0;
  const double kReferenceElevationC = 3.0;
  const double kReferenceElevationD = 4.0;
  const CubicPolynomial<double> reference_elevation(
      kReferenceElevationA, kReferenceElevationB, kReferenceElevationC,
      kReferenceElevationD);
  const double kReferenceSuperelevationA = 5.0;
  const double kReferenceSuperelevationB = 6.0;
  const double kReferenceSuperelevationC = 7.0;
  const double kReferenceSuperelevationD = 8.0;
  const CubicPolynomial<double> reference_superelevation(
      kReferenceSuperelevationA, kReferenceSuperelevationB,
      kReferenceSuperelevationC, kReferenceSuperelevationD);
  const double kRadius = 9.0;
  Elevation<double> elevation(kRadius, reference_elevation,
                              reference_superelevation);
  EXPECT_DOUBLE_EQ(elevation.reference_elevation().a(),
                   kReferenceElevationA);
  EXPECT_DOUBLE_EQ(elevation.reference_elevation().b(),
                   kReferenceElevationB);
  EXPECT_DOUBLE_EQ(elevation.reference_elevation().c(),
                   kReferenceElevationC);
  EXPECT_DOUBLE_EQ(elevation.reference_elevation().d(),
                   kReferenceElevationD);
  EXPECT_DOUBLE_EQ(elevation.reference_superelevation().a(),
                   kReferenceSuperelevationA);
  EXPECT_DOUBLE_EQ(elevation.reference_superelevation().b(),
                   kReferenceSuperelevationB);
  EXPECT_DOUBLE_EQ(elevation.reference_superelevation().c(),
                   kReferenceSuperelevationC);
  EXPECT_DOUBLE_EQ(elevation.reference_superelevation().d(),
                   kReferenceSuperelevationD);
  EXPECT_DOUBLE_EQ(elevation.r(), kRadius);
  elevation.set_r(2.0 * kRadius);
  EXPECT_DOUBLE_EQ(elevation.r(), 2.0 * kRadius);
}

GTEST_TEST(MultilaneElevation, FlatElevationConstantSuperelevationTest) {
  // Flat reference elevation.
  const CubicPolynomial<double> reference_elevation(0.0, 0.0, 0.0, 0.0);
  // Constant superelevation angle.
  const double kSuperelevationOffset = M_PI / 4.0;
  const CubicPolynomial<double> reference_superelevation(
      kSuperelevationOffset, 0.0, 0.0, 0.0);
  const double kRadius = 10.0;
  Elevation<double> elevation(kRadius, reference_elevation,
                              reference_superelevation);
  const double kElevationOffset = kRadius * std::tan(kSuperelevationOffset);
  const std::vector<double> kPVector {0.0, 0.1, 0.2, 0.5, 0.7, 1.0};
  for (const double p : kPVector) {
    EXPECT_DOUBLE_EQ(elevation.f_p(p), kElevationOffset);
  }
  for (const double p : kPVector) {
    EXPECT_DOUBLE_EQ(elevation.f_dot_p(p), 0.0);
  }
  for (const double p : kPVector) {
    EXPECT_DOUBLE_EQ(elevation.f_dot_dot_p(p), 0.0);
  }
  // Since there is no change in the image of the composed function at the
  // beginning (p = 0.0) and at the end (p = 1.0), the scale factor is just 1.0.
  for (const double p : kPVector) {
    EXPECT_DOUBLE_EQ(elevation.s_p(p), p);
  }
  for (const double p : kPVector) {
    EXPECT_DOUBLE_EQ(elevation.p_s(p), p);
  }
}

GTEST_TEST(MultilaneElevation, ComposedElevationTest) {
  // Linear and increasing elevation.
  const double kElevationSlope = 10.0;
  const double kElevationOffset = 10.0;
  const CubicPolynomial<double> reference_elevation(
      kElevationOffset, kElevationSlope, 0.0, 0.0);
  // Constant superelevation angle.
  const double kSuperelevationSlope = M_PI / 200.0;
  const CubicPolynomial<double> reference_superelevation(
      0.0, kSuperelevationSlope, 0.0, 0.0);
  const double kRadius = -5.0;
  Elevation<double> elevation(kRadius, reference_elevation,
                              reference_superelevation);
  const std::vector<double> kPVector {0.0, 0.1, 0.2, 0.5, 0.7, 1.0};
  for (const double p : kPVector) {
    const double reference_elevation_f_p =
        kElevationOffset + kElevationSlope * p;
    const double superelevation_correction =
        kRadius * std::tan(kSuperelevationSlope * p);
    EXPECT_DOUBLE_EQ(elevation.f_p(p),
                     reference_elevation_f_p + superelevation_correction);
  }
}

GTEST_TEST(MultilaneElevation, ScaleTest) {
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

  const double kNewScale = 2.0 * dX;
  const double kRadius = 0.0;
  Elevation<double> elevation(kRadius, polynomial, polynomial);
  polynomial.scale(dX, kNewScale);
  elevation.scale(dX, kNewScale);

  EXPECT_DOUBLE_EQ(elevation.reference_elevation().a(), polynomial.a());
  EXPECT_DOUBLE_EQ(elevation.reference_elevation().b(), polynomial.b());
  EXPECT_DOUBLE_EQ(elevation.reference_elevation().c(), polynomial.c());
  EXPECT_DOUBLE_EQ(elevation.reference_elevation().d(), polynomial.d());
  EXPECT_DOUBLE_EQ(elevation.reference_superelevation().a(), polynomial.a());
  EXPECT_DOUBLE_EQ(elevation.reference_superelevation().b(), polynomial.b());
  EXPECT_DOUBLE_EQ(elevation.reference_superelevation().c(), polynomial.c());
  EXPECT_DOUBLE_EQ(elevation.reference_superelevation().d(), polynomial.d());

  const double kF_p_Init = polynomial.f_p(0.0);
  const double kF_p_End = polynomial.f_p(1.0);
  const double kPathScale = std::sqrt(
      1 + (kF_p_End - kF_p_Init) * (kF_p_End - kF_p_Init));
  EXPECT_DOUBLE_EQ(elevation.s_p(1.0), kPathScale);
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake

