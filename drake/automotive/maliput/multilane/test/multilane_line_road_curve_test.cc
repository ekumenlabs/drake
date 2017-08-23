/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/line_road_curve.h"
/* clang-format on */

#include <utility>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace maliput {
namespace multilane {

// Checks line reference curve interpolations, derivatives, and lengths.
GTEST_TEST(MultilaneLineRoadCurve, LineRoadCurve) {
  const Vector2<double> kOrigin(10.0, 10.0);
  const Vector2<double> kDirection(10.0, 10.0);
  const CubicPolynomial<double> zp(0.0, 0.0, 0.0, 0.0);
  const Elevation<double> flat_elevation(0.0, zp, zp);
  const double kHeading = std::atan2(kDirection.y(), kDirection.x());
  const double kHeadingDerivative = 0.0;
  const double kVeryExact = 1e-12;

  const LineRoadCurve flat_line_geometry(kOrigin, kDirection, flat_elevation,
                                         zp);
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

// Checks that LineRoadCurve::IsValid() returns true.
GTEST_TEST(MultilaneLineRoadCurve, IsValidTest) {
  const Vector2<double> kOrigin(10.0, 10.0);
  const Vector2<double> kDirection(10.0, 10.0);
  const CubicPolynomial<double> zp(0.0, 0.0, 0.0, 0.0);
  const Elevation<double> flat_elevation(0.0, zp, zp);
  const api::RBounds lateral_bounds(-10.0, 10.0);
  const api::HBounds elevation_bounds(0.0, 10.0);
  const LineRoadCurve flat_line_geometry(kOrigin, kDirection, flat_elevation,
                                         zp);
  EXPECT_TRUE(flat_line_geometry.IsValid(lateral_bounds, elevation_bounds));
}

// Checks the validity of the surface for different lateral bounds and
// geometries.
GTEST_TEST(MultilaneLineRoadCurve, ToCurveFrameTest) {
  const Vector2<double> kOrigin(10.0, 10.0);
  const Vector2<double> kDirection(10.0, 10.0);
  const double kVeryExact = 1e-12;
  const CubicPolynomial<double> zp(0.0, 0.0, 0.0, 0.0);
  const Elevation<double> flat_elevation(0.0, zp, zp);
  const api::RBounds lateral_bounds(-10.0, 10.0);
  const api::HBounds elevation_bounds(0.0, 10.0);

  const LineRoadCurve flat_line_geometry(kOrigin, kDirection, flat_elevation,
                                         zp);
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

// Checks the Offset() computation of a LineRoadCurve object. Aerial view of the
// geometry is the following:
//
// <pre>
//
//            *             <---- Left LineRoadCurve offset.
//          *
//        *
//      *           *       <---- Base LineRoadCurve, elevation = 0.0.
//    *           *
//              *
//            *         *   <---- Right LineRoadCurve offset.
//          *         *
//                  *
//                *
//              *
// </pre>
//
// A cross section of the three curves will show the following:
//
// <pre>
//
//    x                   <---- Left LineRoadCurve offset.
//    | *
//    |   *
//    |     x             <---- Base LineRoadCurve, elevation = 0.0.
//    |       *
//    |         *
//    |___________x       <---- Right LineRoadCurve offset.
//
// </pre>
//
// Note that 'x' characters denote the curve cut, '*' characters show the
// surface that holds all the curves and with '|' and '_' axis are denoted.
GTEST_TEST(MultilaneLineRoadCurve, OffsetTest) {
  const double kVeryExact = 1e-12;
  const Vector2<double> kOrigin(10.0, 10.0);
  const Vector2<double> kDirection(10.0, 10.0);
  const CubicPolynomial<double> reference_elevation(0.0, 0.0, 0.0, 0.0);
  const double kSuperelevationOffset = M_PI / 4.0;
  const CubicPolynomial<double> reference_superelevation(
      kSuperelevationOffset, 0.0, 0.0, 0.0);
  const Elevation<double> composed_elevation(
      0.0, reference_elevation, reference_superelevation);
  const LineRoadCurve line_road(
      kOrigin, kDirection, composed_elevation, reference_superelevation);

  const std::vector<double> kPVector = {0.0, 0.1, 0.2, 0.5, 0.7, 1.0};
  for (const double p : kPVector) {
    EXPECT_DOUBLE_EQ(line_road.elevation().f_p(p), 0.0);
  }
  // Computes an offset of the line to the left.
  const double kOffsetDistance = 10.0;
  std::unique_ptr<RoadCurve> offset_line_road =
      line_road.Offset(kOffsetDistance);
  EXPECT_DOUBLE_EQ(offset_line_road->length(), line_road.length());
  EXPECT_DOUBLE_EQ(offset_line_road->trajectory_length(),
                   line_road.trajectory_length());
  const Vector2<double> kLeftDifferenceVector(
      -kOffsetDistance / std::sqrt(2.0), kOffsetDistance / std::sqrt(2.0));
  for (const double p : kPVector) {
    EXPECT_DOUBLE_EQ(offset_line_road->elevation().f_p(p), kOffsetDistance);
    const Vector2<double> difference_vector = offset_line_road->xy_of_p(p) -
                                              line_road.xy_of_p(p);
    EXPECT_TRUE(CompareMatrices(difference_vector, kLeftDifferenceVector,
                                kVeryExact));
  }
  // Computes an offset of the line to the right.
  offset_line_road = line_road.Offset(-kOffsetDistance);
  EXPECT_DOUBLE_EQ(offset_line_road->length(), line_road.length());
  EXPECT_DOUBLE_EQ(offset_line_road->trajectory_length(),
                   line_road.trajectory_length());
  const Vector2<double> kRightDifferenceVector(
      kOffsetDistance / std::sqrt(2.0), -kOffsetDistance / std::sqrt(2.0));
  for (const double p : kPVector) {
    EXPECT_DOUBLE_EQ(offset_line_road->elevation().f_p(p), -kOffsetDistance);
    const Vector2<double> difference_vector = offset_line_road->xy_of_p(p) -
                                              line_road.xy_of_p(p);
    EXPECT_TRUE(CompareMatrices(difference_vector, kRightDifferenceVector,
                                kVeryExact));
  }
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
