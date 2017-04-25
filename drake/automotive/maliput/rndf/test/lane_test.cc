#include <cmath>
#include <iostream>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/rndf/junction.h"
#include "drake/automotive/maliput/rndf/lane.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/automotive/maliput/rndf/segment.h"
#include "drake/automotive/maliput/rndf/spline_lane.h"

namespace drake {
namespace maliput {
namespace rndf {

const double kLinearTolerance = 1e-3;
const double kAngularTolerance = 1e-3;
const double kVeryExact = 1e-12;

#define EXPECT_GEO_NEAR(actual, expected, tolerance)         \
  do {                                                       \
    const api::GeoPosition _actual(actual);                  \
    const api::GeoPosition _expected expected;               \
    const double _tolerance = (tolerance);                   \
    EXPECT_NEAR(_actual.x, _expected.x, _tolerance);         \
    EXPECT_NEAR(_actual.y, _expected.y, _tolerance);         \
    EXPECT_NEAR(_actual.z, _expected.z, _tolerance);         \
  } while (0)

#define EXPECT_LANE_NEAR(actual, expected, tolerance)         \
  do {                                                        \
    const api::LanePosition _actual(actual);                  \
    const api::LanePosition _expected expected;               \
    const double _tolerance = (tolerance);                    \
    EXPECT_NEAR(_actual.s, _expected.s, _tolerance);          \
    EXPECT_NEAR(_actual.r, _expected.r, _tolerance);          \
    EXPECT_NEAR(_actual.h, _expected.h, _tolerance);          \
  } while (0)

#define EXPECT_ROT_NEAR(actual, expected, tolerance)                 \
  do {                                                               \
    const api::Rotation _actual(actual);                             \
    const api::Rotation _expected expected;                          \
    const double _tolerance = (tolerance);                           \
    EXPECT_NEAR(_actual.yaw, _expected.yaw, _tolerance);             \
    EXPECT_NEAR(_actual.pitch, _expected.pitch, _tolerance);         \
    EXPECT_NEAR(_actual.roll, _expected.roll, _tolerance);           \
  } while (0)

GTEST_TEST(RNDFLanesTest, FlatLineLane) {
  RoadGeometry rg({"FlatLineLane"}, kLinearTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  std::vector<
    std::tuple<ignition::math::Vector3d,ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(0.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(20.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  Lane *l1 = s1->NewSplineLane(
    {"l1"},
    control_points,
    {-5., 5.},
    {-10., 10.});

  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  EXPECT_EQ(l1->id().id, "l1");
  EXPECT_EQ(l1->segment(), s1);
  EXPECT_EQ(l1->index(), 0);
  EXPECT_EQ(l1->to_left(), nullptr);
  EXPECT_EQ(l1->to_right(), nullptr);

  EXPECT_NEAR(l1->length(), std::sqrt((20. * 20.) + (0. * 0.)), kLinearTolerance);

  EXPECT_NEAR(l1->lane_bounds(0.).r_min, -5., kVeryExact);
  EXPECT_NEAR(l1->lane_bounds(0.).r_max,  5., kVeryExact);
  EXPECT_NEAR(l1->driveable_bounds(0.).r_min, -10., kVeryExact);
  EXPECT_NEAR(l1->driveable_bounds(0.).r_max,  10., kVeryExact);

  // ToGeoPosition
  // Reference line
  // At the beginning, end and middle
  EXPECT_GEO_NEAR(l1->ToGeoPosition({0., 0., 0.}), (0., 0., 0.), kLinearTolerance);
  EXPECT_GEO_NEAR(l1->ToGeoPosition({20., 0., 0.}), (20., 0., 0.), kLinearTolerance);
  EXPECT_GEO_NEAR(l1->ToGeoPosition({10., 0., 0.}), (10., 0., 0.), kLinearTolerance);

  // A couple of meters away of the reference baseline
  EXPECT_GEO_NEAR(l1->ToGeoPosition({5., 2., 0.}), (5., 2., 0.), kLinearTolerance);
  EXPECT_GEO_NEAR(l1->ToGeoPosition({12., -2., 0.}), (12., -2., 0.), kLinearTolerance);
  EXPECT_GEO_NEAR(l1->ToGeoPosition({18., 1.234567, 0.}), (18., 1.234567, 0.), kLinearTolerance);

  // Outside the lane constraints
  EXPECT_GEO_NEAR(l1->ToGeoPosition({-1., 0., 0.}), (0., 0., 0.), kLinearTolerance);
  EXPECT_GEO_NEAR(l1->ToGeoPosition({21., 0., 0.}), (20., 0., 0.), kLinearTolerance);
  // TODO: Need to work on the lateral constraints!!
  // EXPECT_GEO_NEAR(l1->ToGeoPosition({5., 11., 0.}), (5., 10., 0.), kLinearTolerance);

  // Orientation
  EXPECT_ROT_NEAR(l1->GetOrientation({0., 0., 0.}), (0., 0., 0.), kAngularTolerance);
  EXPECT_ROT_NEAR(l1->GetOrientation({20., 0., 0.}), (0., 0., 0.), kAngularTolerance);
  EXPECT_ROT_NEAR(l1->GetOrientation({10., 2., 0.}), (0., 0., 0.), kAngularTolerance);
  EXPECT_ROT_NEAR(l1->GetOrientation({10., -2., 0.}), (0., 0., 0.), kAngularTolerance);

  // EvalMotionDerivatives
  EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}), (0., 0., 0.), kLinearTolerance);
  EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}), (1., 0., 0.), kLinearTolerance);
  EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}), (0., 1., 0.), kLinearTolerance);
  EXPECT_LANE_NEAR(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}), (0., 0., 1.), kLinearTolerance);
}

} // rndf
} // maliput
} // drake