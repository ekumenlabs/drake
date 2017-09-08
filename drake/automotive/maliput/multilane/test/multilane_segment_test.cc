/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/segment.h"
/* clang-format on */

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test/maliput_types_compare.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/multilane/arc_road_curve.h"
#include "drake/automotive/maliput/multilane/line_road_curve.h"
#include "drake/automotive/maliput/multilane/road_curve.h"

namespace drake {
namespace maliput {
namespace multilane {

// Checks how single lane segments create Lane and assign bounds with different
// road curve offsets.
GTEST_TEST(MulitilaneSegmentTest, SingleLaneTest) {
  const Vector2<double> kOrigin{10.0, 10.0};
  const Vector2<double> kDirection{10.0, 10.0};
  const CubicPolynomial zp;
  const api::RBounds lateral_bounds(-10.0, 10.0);
  const api::HBounds elevation_bounds(0., 10.);
  const std::vector<double> r0_vector{50., 10.0, 0.0, -10.0, -50.};
  const double kWidth = 20.;
  const double kRSpacing = 20.;
  const int kNumLanes = 1;

  for (double r0 : r0_vector) {
    std::unique_ptr<RoadCurve> road_curve =
        std::make_unique<LineRoadCurve>(kOrigin, kDirection, zp, zp);
    Segment segment(api::SegmentId("single_lane_segment"), nullptr,
                    std::move(road_curve), kNumLanes, kRSpacing, r0, kWidth,
                    elevation_bounds);
    EXPECT_EQ(segment.num_lanes(), 0);
    const Lane* lane = segment.NewLane(api::LaneId("lane"));
    EXPECT_NE(lane, nullptr);
    EXPECT_EQ(segment.num_lanes(), 1);
    EXPECT_EQ(segment.lane(0), lane);
    EXPECT_TRUE(
        api::test::IsRBoundsClose(lane->lane_bounds(0.), lateral_bounds, 0.));
    EXPECT_TRUE(api::test::IsRBoundsClose(lane->driveable_bounds(0.),
                                          lateral_bounds, 0.));
  }
}

// Checks how multi-lane segments create Lane and assign bounds with different
// road curve offsets.
GTEST_TEST(MulitilaneSegmentTest, MultipleLaneTest) {
  const Vector2<double> kOrigin{10.0, 10.0};
  const Vector2<double> kDirection{10.0, 10.0};
  const CubicPolynomial zp;
  const api::RBounds lateral_bounds(-5.0, 5.0);
  const api::HBounds elevation_bounds(0., 10.);
  const double kWidth = 30.;
  const double kRSpacing = 10.;
  const int kNumLanes = 3;
  const std::vector<double> r0_vector{50., 10.0, 0.0, -10.0, -50.};

  for (double r0 : r0_vector) {
    std::unique_ptr<RoadCurve> road_curve =
        std::make_unique<LineRoadCurve>(kOrigin, kDirection, zp, zp);
    Segment segment(api::SegmentId("multilane_segment"), nullptr,
                    std::move(road_curve), kNumLanes, r0, kRSpacing, kWidth,
                    elevation_bounds);
    EXPECT_EQ(segment.num_lanes(), 0);
    const Lane* l0 = segment.NewLane(api::LaneId("l0"));
    EXPECT_NE(l0, nullptr);
    EXPECT_EQ(segment.num_lanes(), 1);
    EXPECT_EQ(segment.lane(0), l0);
    EXPECT_TRUE(
        api::test::IsRBoundsClose(l0->lane_bounds(0.), lateral_bounds, 0.));
    EXPECT_TRUE(api::test::IsRBoundsClose(l0->driveable_bounds(0.),
                                          api::RBounds(-5., 25.), 0.));
    const Lane* l1 = segment.NewLane(api::LaneId("l1"));
    EXPECT_NE(l1, nullptr);
    EXPECT_EQ(segment.num_lanes(), 2);
    EXPECT_EQ(segment.lane(1), l1);
    EXPECT_TRUE(
        api::test::IsRBoundsClose(l1->lane_bounds(0.), lateral_bounds, 0.));
    EXPECT_TRUE(api::test::IsRBoundsClose(l1->driveable_bounds(0.),
                                          api::RBounds(-15., 15.), 0.));
    const Lane* l2 = segment.NewLane(api::LaneId("l2"));
    EXPECT_NE(l2, nullptr);
    EXPECT_EQ(segment.num_lanes(), 3);
    EXPECT_EQ(segment.lane(2), l2);
    EXPECT_TRUE(
        api::test::IsRBoundsClose(l2->lane_bounds(0.), lateral_bounds, 0.));
    EXPECT_TRUE(api::test::IsRBoundsClose(l2->driveable_bounds(0.),
                                          api::RBounds(-25., 5.), 0.));
  }
}

}
}
}
