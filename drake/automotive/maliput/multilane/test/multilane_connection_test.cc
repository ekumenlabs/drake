/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/builder.h"
/* clang-format on */

#include <cmath>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/multilane/test_utilities/multilane_types_compare.h"

namespace drake {
namespace maliput {
namespace multilane {

class MultilaneConnectionTest : public ::testing::Test {
 protected:
  const double kR0{10.};
  const int kNumLanes{3};
  const double kHeading{-M_PI / 4.};
  const EndpointZ kLowFlatZ{0., 0., 0., 0.};
  const Endpoint kStartEndpoint{{20., 30., kHeading}, kLowFlatZ};
  const double kZeroTolerance{0.};
  const double kLaneWidth{5.};
  const double kLeftShoulder{2.};
  const double kRightShoulder{2.};
};

TEST_F(MultilaneConnectionTest, ArcAccessors) {
  const std::string kId{"arc_connection"};
  const double kCenterX{40.};
  const double kCenterY{30.};
  const double kRadius{20. * std::sqrt(2.)};
  const double kDTheta{M_PI/2.};
  const Endpoint kEndEndpoint{{40., 30., kHeading + kDTheta}, kLowFlatZ};

  const Connection connection(kId, kStartEndpoint, kEndEndpoint, kNumLanes, kR0,
                              kLaneWidth, kLeftShoulder, kRightShoulder,
                              kCenterX, kCenterY, kRadius, kDTheta);
  EXPECT_EQ(connection.type(), Connection::Type::kArc);
  EXPECT_EQ(connection.id(), kId);
  EXPECT_TRUE(test::IsEndpointClose(
      connection.start(), kStartEndpoint, kZeroTolerance));
  EXPECT_TRUE(
      test::IsEndpointClose(connection.end(), kEndEndpoint, kZeroTolerance));
  EXPECT_EQ(connection.num_lanes(), kNumLanes);
  EXPECT_EQ(connection.r0(), kR0);
  EXPECT_EQ(connection.left_shoulder(), kLeftShoulder);
  EXPECT_EQ(connection.right_shoulder(), kRightShoulder);
  EXPECT_EQ(connection.r_min(), kR0 - kLaneWidth / 2. - kRightShoulder);
  EXPECT_EQ(connection.r_max(), kR0 + kLaneWidth *
                                      (static_cast<double>(kNumLanes - 1) + .5)
                                    + kLeftShoulder);
  EXPECT_EQ(connection.cx(), kCenterX);
  EXPECT_EQ(connection.cy(), kCenterY);
  EXPECT_EQ(connection.radius(), kRadius);
  EXPECT_EQ(connection.d_theta(), kDTheta);

  for (int i = 0; i < kNumLanes; i++) {
    EXPECT_EQ(
        connection.lane_offset(i), kR0 + kLaneWidth * static_cast<double>(i));
  }
}

TEST_F(MultilaneConnectionTest, LineAccessors) {
  const std::string kId{"line_connection"};
  const Endpoint kEndEndpoint{{-10., 0., kHeading}, kLowFlatZ};

  const Connection connection(kId, kStartEndpoint, kEndEndpoint, kNumLanes, kR0,
                              kLaneWidth, kLeftShoulder, kRightShoulder);
  EXPECT_EQ(connection.type(), Connection::Type::kLine);
  EXPECT_EQ(connection.id(), kId);
  EXPECT_TRUE(test::IsEndpointClose(
      connection.start(), kStartEndpoint, kZeroTolerance));
  EXPECT_TRUE(
      test::IsEndpointClose(connection.end(), kEndEndpoint, kZeroTolerance));
  EXPECT_EQ(connection.num_lanes(), kNumLanes);
  EXPECT_EQ(connection.r0(), kR0);
  EXPECT_EQ(connection.left_shoulder(), kLeftShoulder);
  EXPECT_EQ(connection.right_shoulder(), kRightShoulder);
  EXPECT_EQ(connection.r_min(), kR0 - kLaneWidth / 2. - kRightShoulder);
  EXPECT_EQ(connection.r_max(), kR0 + kLaneWidth *
                                      (static_cast<double>(kNumLanes - 1) + .5)
                                    + kLeftShoulder);
  for (int i = 0; i < kNumLanes; i++) {
    EXPECT_EQ(
        connection.lane_offset(i), kR0 + kLaneWidth * static_cast<double>(i));
  }
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
