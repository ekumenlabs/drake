/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/line_segment_geometry.h"
/* clang-format on */

#include <utility>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/multilane/segment_geometry.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace maliput {
namespace multilane {

GTEST_TEST(MultilaneLineGeometry, LineGeometryTest) {
  const Vector2<double> kOrigin(10.0, 10.0);
  const Vector2<double> kDirection(10.0, 10.0);
  const CubicPolynomial zp;
  const double kHeading = std::atan2(kDirection.y(), kDirection.x());
  const double kHeadingDerivative = 0.0;
  const double kVeryExact = 1e-12;

  const LineGeometry flat_line_geometry(kOrigin, kDirection, zp, zp);
  // Checks the length.
  EXPECT_NEAR(flat_line_geometry.length(),
              std::sqrt(kDirection.x() * kDirection.x() +
                        kDirection.y() * kDirection.y()),
              kVeryExact);
  // Check the interpolation of p at different p values.
  EXPECT_TRUE(
      CompareMatrices(flat_line_geometry.xy_of_p(0.0), kOrigin, kVeryExact));
  EXPECT_TRUE(CompareMatrices(flat_line_geometry.xy_of_p(0.5),
                              kOrigin + 0.5 * kDirection, kVeryExact));
  EXPECT_TRUE(CompareMatrices(flat_line_geometry.xy_of_p(1.0),
                              kOrigin + kDirection, kVeryExact));
  // Check the derivative of p at different p values.
  EXPECT_TRUE(CompareMatrices(flat_line_geometry.xy_dot_of_p(0.0), kDirection,
                              kVeryExact));
  EXPECT_TRUE(CompareMatrices(flat_line_geometry.xy_dot_of_p(0.5), kDirection,
                              kVeryExact));
  EXPECT_TRUE(CompareMatrices(flat_line_geometry.xy_dot_of_p(1.0), kDirection,
                              kVeryExact));
  // Check the heading at different p values.
  EXPECT_NEAR(flat_line_geometry.heading_of_p(0.0), kHeading, kVeryExact);
  EXPECT_NEAR(flat_line_geometry.heading_of_p(0.5), kHeading, kVeryExact);
  EXPECT_NEAR(flat_line_geometry.heading_of_p(1.0), kHeading, kVeryExact);
  // Check the heading derivative of p at different p values.
  EXPECT_NEAR(flat_line_geometry.heading_dot_of_p(0.0), kHeadingDerivative,
              kVeryExact);
  EXPECT_NEAR(flat_line_geometry.heading_dot_of_p(0.5), kHeadingDerivative,
              kVeryExact);
  EXPECT_NEAR(flat_line_geometry.heading_dot_of_p(1.0), kHeadingDerivative,
              kVeryExact);
}

GTEST_TEST(MultilaneLineGeometry, IsValidTest) {
  const Vector2<double> kOrigin(10.0, 10.0);
  const Vector2<double> kDirection(10.0, 10.0);
  const CubicPolynomial zp;
  const std::pair<double, double> lateral_bounds = std::make_pair(-10.0, 10.0);
  const std::pair<double, double> elevation_bounds = std::make_pair(0.0, 10.0);

  const LineGeometry flat_line_geometry(kOrigin, kDirection, zp, zp);
  EXPECT_THROW(
      flat_line_geometry.IsValid(std::make_pair(0.0, -10.0), elevation_bounds),
      std::runtime_error);
  EXPECT_THROW(
      flat_line_geometry.IsValid(lateral_bounds, std::make_pair(0.0, -10.0)),
      std::runtime_error);
  EXPECT_NO_THROW(flat_line_geometry.IsValid(lateral_bounds, elevation_bounds));
  EXPECT_TRUE(flat_line_geometry.IsValid(lateral_bounds, elevation_bounds));
}

GTEST_TEST(MultilaneLineGeometry, ToCurveFrameTest) {
  const Vector2<double> kOrigin(10.0, 10.0);
  const Vector2<double> kDirection(10.0, 10.0);
  const double kVeryExact = 1e-12;
  const CubicPolynomial zp;
  const std::pair<double, double> lateral_bounds = std::make_pair(-10.0, 10.0);
  const std::pair<double, double> elevation_bounds = std::make_pair(0.0, 10.0);

  const LineGeometry flat_line_geometry(kOrigin, kDirection, zp, zp);
  // Checks that it throws when lateral_bounds or elevation_bounds are wrong.
  EXPECT_THROW(flat_line_geometry.ToCurveFrame(Vector3<double>(10.0, 10.0, 0.0),
                                               std::make_pair(0.0, -10.0),
                                               elevation_bounds),
               std::runtime_error);
  EXPECT_THROW(flat_line_geometry.ToCurveFrame(Vector3<double>(10.0, 10.0, 0.0),
                                               lateral_bounds,
                                               std::make_pair(0.0, -10.0)),
               std::runtime_error);
  // Checks over the base line.
  EXPECT_TRUE(CompareMatrices(
      flat_line_geometry.ToCurveFrame(Vector3<double>(10.0, 10.0, 0.0),
                                      lateral_bounds, elevation_bounds),
      Vector3<double>(0.0, 0.0, 0.0), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      flat_line_geometry.ToCurveFrame(Vector3<double>(20.0, 20.0, 0.0),
                                      lateral_bounds, elevation_bounds),
      Vector3<double>(std::sqrt(2) * 10.0, 0.0, 0.0), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      flat_line_geometry.ToCurveFrame(Vector3<double>(15.0, 15.0, 0.0),
                                      lateral_bounds, elevation_bounds),
      Vector3<double>(std::sqrt(2) * 5.0, 0.0, 0.0), kVeryExact));
  // Check with lateral and vertical deviation.
  EXPECT_TRUE(CompareMatrices(
      flat_line_geometry.ToCurveFrame(Vector3<double>(11.0, 12.0, 5.0),
                                      lateral_bounds, elevation_bounds),
      Vector3<double>(2.12132034355964, 0.707106781186547, 5.0), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      flat_line_geometry.ToCurveFrame(Vector3<double>(11.0, 10.0, 7.0),
                                      lateral_bounds, elevation_bounds),
      Vector3<double>(0.707106781186547, -0.707106781186547, 7.0), kVeryExact));
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
