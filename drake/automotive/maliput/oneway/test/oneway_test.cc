#include "gtest/gtest.h"

#include "drake/automotive/maliput/oneway/branch_point.h"
#include "drake/automotive/maliput/oneway/junction.h"
#include "drake/automotive/maliput/oneway/lane.h"
#include "drake/automotive/maliput/oneway/road_geometry.h"

namespace drake {
namespace maliput {
namespace oneway {
namespace {

// To understand the characteristics of the geometry, consult the
// oneway::Segment and oneway::Lane detailed class overview docs.
class MaliputOnewayLaneTest : public ::testing::Test {
 public:
  MaliputOnewayLaneTest()
      : length_(100.0),
        lane_width_(6.0) {}

  // Verifies the correctness of the provided `lane`.
  void VerifyLaneCorrectness(const api::Lane* lane) {
    // Tests Lane::length()
    EXPECT_EQ(lane->length(), length_);

    // Tests Lane::lane_bounds().
    for (double s = 0 ; s < length_; s += length_ / 100) {
      const api::RBounds lane_bounds = lane->lane_bounds(s);
      const api::RBounds driveable_bounds = lane->driveable_bounds(s);
      EXPECT_EQ(lane_bounds.r_min, -lane_width_ / 2);
      EXPECT_EQ(lane_bounds.r_max, lane_width_ / 2);
      EXPECT_EQ(driveable_bounds.r_min, -lane_width_ / 2);
      EXPECT_EQ(driveable_bounds.r_max, lane_width_ / 2);
    }

    // The following block of test code evaluates methods that take as input a
    // (s, r, h) lane position. Ideally, we want to check that the
    // method-under-test is correct for all combinations of (s, r, h). This,
    // unfortunately, it not possible since there are too many combinations of
    // (s, r, h). Instead we pick points in a grid that spans the (s, r, h)
    // state space and only check those points in the hopes that they are
    // representative of the entire state space.
    for (double s = 0; s < length_; s += length_ / 100) {
      for (double r = -lane_width_ / 2; r <= lane_width_ / 2;
           r += lane_width_ / 100) {
        for (double h = 0; h < 1; h += 0.1) {
          const api::LanePosition lane_position(s, r, h);

          // Tests Lane::ToGeoPosition().
          const api::GeoPosition geo_position =
              lane->ToGeoPosition(lane_position);
          EXPECT_DOUBLE_EQ(geo_position.x, s);
          EXPECT_DOUBLE_EQ(geo_position.y, r);
          EXPECT_DOUBLE_EQ(geo_position.z, h);

          // Tests Lane::GetOrientation().
          const api::Rotation rotation =
              lane->GetOrientation(lane_position);
          EXPECT_DOUBLE_EQ(rotation.roll, 0);
          EXPECT_DOUBLE_EQ(rotation.pitch, 0);
          EXPECT_DOUBLE_EQ(rotation.yaw, 0);

          // Tests Lane::EvalMotionDerivatives().
          //
          // The following translational velocities can be any value. We just
          // want to verify that the same values are returned from
          // Lane::EvalMotionDerivatives().
          const double kSigma_v = 1.1;
          const double kRho_v = 2.2;
          const double kEta_v = 3.3;

          const api::LanePosition motion_derivatives =
              lane->EvalMotionDerivatives(lane_position,
                  api::IsoLaneVelocity(kSigma_v, kRho_v, kEta_v));
          EXPECT_DOUBLE_EQ(motion_derivatives.s, kSigma_v);
          EXPECT_DOUBLE_EQ(motion_derivatives.r, kRho_v);
          EXPECT_DOUBLE_EQ(motion_derivatives.h, kEta_v);
        }
      }
    }
  }

  // Verifies that the branches within the provided `lane` are correct. The
  // provided `lane` must be in the provided `road_geometry`.
  void VerifyBranches(const api::Lane* lane,
      const RoadGeometry* road_geometry) const {
    // Verifies that the same BranchPoint covers both ends of the oneway lane.
    EXPECT_EQ(lane->GetBranchPoint(api::LaneEnd::kStart),
              lane->GetBranchPoint(api::LaneEnd::kFinish));

    const api::BranchPoint* branch_point =
        lane->GetBranchPoint(api::LaneEnd::kStart);
    EXPECT_NE(branch_point, nullptr);
    EXPECT_EQ(branch_point->id().id, lane->id().id + "_Branch_Point");
    EXPECT_EQ(branch_point->road_geometry(), road_geometry);

    // Verifies correctness of the confluent branches.
    {
      const api::LaneEndSet* lane_end_set_start =
          lane->GetConfluentBranches(api::LaneEnd::kStart);
      const api::LaneEndSet* lane_end_set_finish =
          lane->GetConfluentBranches(api::LaneEnd::kFinish);
      EXPECT_EQ(lane_end_set_start->size(), 1);
      EXPECT_EQ(lane_end_set_finish->size(), 1);

      const api::LaneEnd& lane_end_start =
        lane->GetConfluentBranches(api::LaneEnd::kStart)->get(0);
      EXPECT_EQ(lane_end_start.lane, lane);
      EXPECT_EQ(lane_end_start.end, api::LaneEnd::kStart);
      const api::LaneEnd& lane_end_finish =
        lane->GetConfluentBranches(api::LaneEnd::kFinish)->get(0);
      EXPECT_EQ(lane_end_finish.lane, lane);
      EXPECT_EQ(lane_end_finish.end, api::LaneEnd::kFinish);
    }

    // Verifies correctness of the ongoing branches.
    {
      const api::LaneEndSet* lane_end_set_start =
          lane->GetOngoingBranches(api::LaneEnd::kStart);
      const api::LaneEndSet* lane_end_set_finish =
          lane->GetOngoingBranches(api::LaneEnd::kFinish);
      EXPECT_EQ(lane_end_set_start->size(), 1);
      EXPECT_EQ(lane_end_set_finish->size(), 1);

      const api::LaneEnd& lane_end_start =
        lane->GetOngoingBranches(api::LaneEnd::kStart)->get(0);
      EXPECT_EQ(lane_end_start.lane, lane);
      EXPECT_EQ(lane_end_start.end, api::LaneEnd::kFinish);
      const api::LaneEnd& lane_end_finish =
        lane->GetOngoingBranches(api::LaneEnd::kFinish)->get(0);
      EXPECT_EQ(lane_end_finish.lane, lane);
      EXPECT_EQ(lane_end_finish.end, api::LaneEnd::kStart);
    }

    // Verifies correctness of the default branches.
    {
      std::unique_ptr<api::LaneEnd> default_start_lane_end =
          lane->GetDefaultBranch(api::LaneEnd::kStart);
      EXPECT_NE(default_start_lane_end.get(), nullptr);
      EXPECT_EQ(default_start_lane_end->end, api::LaneEnd::kFinish);
      EXPECT_EQ(default_start_lane_end->lane, lane);

      std::unique_ptr<api::LaneEnd> default_finish_lane_end =
          lane->GetDefaultBranch(api::LaneEnd::kFinish);
      EXPECT_NE(default_finish_lane_end.get(), nullptr);
      EXPECT_EQ(default_finish_lane_end->end, api::LaneEnd::kStart);
      EXPECT_EQ(default_finish_lane_end->lane, lane);
    }
  }

  const double length_{};
  const double lane_width_{};
};

/*
 Tests a oneway. It is arranged as shown below in the world frame:

               x
               ^
      |<-------|------->|    driveable & lane r_max / r_min
      -------------------    s = length_
      |        ^        |
      |        :        |
      |        ^        |
      |        :        |
  y <----------o---------->  s = 0
               |
               V
 */
TEST_F(MaliputOnewayLaneTest, SingleLane) {
  const api::RoadGeometryId road_geometry_id({"OneLaneOnewayRoadGeometry"});

  RoadGeometry road_geometry(road_geometry_id, length_, lane_width_);

  const api::Junction* junction = road_geometry.junction(0);
  ASSERT_NE(junction, nullptr);
  const api::Segment* segment = junction->segment(0);
  ASSERT_NE(segment, nullptr);
  EXPECT_EQ(segment->lane(0)->length(), length_);
  EXPECT_EQ(segment->num_lanes(), 1);
  EXPECT_EQ(segment->id().id, "Oneway_Segment_ID");

  const api::Lane* lane = segment->lane(0);
  ASSERT_NE(lane, nullptr);
  EXPECT_EQ(lane->segment(), segment);

  VerifyLaneCorrectness(lane);
  VerifyBranches(lane, &road_geometry);
}

// Tests oneway::RoadGeometry::ToRoadPosition() using a two-lane oneway. This
// also verifies that oneway::RoadGeometry::IsGeoPositionOnOneway() does not
// incorrectly return false.
TEST_F(MaliputOnewayLaneTest, TestToRoadPositionOnRoad) {
  const api::RoadGeometryId road_geometry_id({"OnewayRoadGeometry"});

  RoadGeometry road_geometry(road_geometry_id, length_, lane_width_);

  // Spot checks geographic positions on lane with a
  // focus on edge cases.
  for (double x = 0; x <= length_; x += length_ / 2) {
    for (double y = -lane_width_ / 2; y <= lane_width_ / 2;
        y += lane_width_ / 2) {
      for (double z = 0; z <= 10; z += 5) {
        api::GeoPosition nearest_position;
        double distance;
        const api::RoadPosition road_position = road_geometry.ToRoadPosition(
            api::GeoPosition(x, y, z), nullptr /* hint */, &nearest_position,
            &distance);
        const api::Lane* expected_lane =
            road_geometry.junction(0)->segment(0)->lane(0);
        EXPECT_DOUBLE_EQ(nearest_position.x, x);
        EXPECT_DOUBLE_EQ(nearest_position.y, y);
        EXPECT_DOUBLE_EQ(nearest_position.z, z);
        EXPECT_DOUBLE_EQ(distance, 0);
        EXPECT_EQ(road_position.lane, expected_lane);
        EXPECT_DOUBLE_EQ(road_position.pos.s, x);
        EXPECT_DOUBLE_EQ(road_position.pos.r, y);
        EXPECT_DOUBLE_EQ(road_position.pos.h, z);
      }
    }
  }
}

}  // namespace
}  // namespace oneway
}  // namespace maliput
}  // namespace drake
