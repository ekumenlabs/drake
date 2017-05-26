#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include <ignition/math/Vector3.hh>

#include <ignition/rndf/UniqueId.hh>

#include "drake/common/drake_throw.h"
#include "drake/automotive/maliput/rndf/builder.h"

namespace drake {
namespace maliput {
namespace rndf {

// const double kLinearTolerance = 1e-2;

#define EXPECT_GEO_NEAR(actual, expected, tolerance)         \
do {                                                       \
  const api::GeoPosition _actual(actual);                  \
  const api::GeoPosition _expected(expected);               \
  const double _tolerance = (tolerance);                   \
  EXPECT_NEAR(_actual.x(), _expected.x(), _tolerance);         \
  EXPECT_NEAR(_actual.y(), _expected.y(), _tolerance);         \
  EXPECT_NEAR(_actual.z(), _expected.z(), _tolerance);         \
} while (0)

//   * 1.1.1
//    \
//     \
//      * 1.1.2
//     /
//    /
//   * 1.1.3
//    \
//     \
//      * 1.1.4
GTEST_TEST(RNDFBuilder, ZigZagLane) {
  const double width = 5.;

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    0.01,
    0.01 * M_PI);

  std::vector<DirectedWaypoint> waypoints;
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 1),
    ignition::math::Vector3d(0., 0.0, 0.0)));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 2),
    ignition::math::Vector3d(10.0, -10.0, 0.0)));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 3),
    ignition::math::Vector3d(0.0, -20.0, 0.0)));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 4),
    ignition::math::Vector3d(10.0, -30.0, 0.0)));
  Connection l1(std::to_string(1),
    waypoints,
    width);

  std::vector<Connection> connected_lanes = {l1};

  auto bounding_box = std::make_tuple<ignition::math::Vector3d,
    ignition::math::Vector3d> (ignition::math::Vector3d(0., -30., 0.),
      ignition::math::Vector3d(10., 0., 0.));
  builder->SetBoundingBox(bounding_box);
  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"ZigZagLane"});
  EXPECT_NE(road_geometry, nullptr);

  // Check the junctions, segments and lanes
  EXPECT_EQ(road_geometry->num_junctions(), 3);
  EXPECT_EQ(road_geometry->junction(0)->id().id, std::string("j:1-0-0"));
  EXPECT_EQ(road_geometry->junction(0)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id,
    std::string("s:1-0-0"));
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->id().id,
    std::string("l:1_1_1-1_1_2"));

  EXPECT_EQ(road_geometry->junction(1)->id().id, std::string("j:1-0-1"));
  EXPECT_EQ(road_geometry->junction(1)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id,
    std::string("s:1-0-1"));
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->id().id,
    std::string("l:1_1_2-1_1_3"));

  EXPECT_EQ(road_geometry->junction(2)->id().id, std::string("j:1-0-2"));
  EXPECT_EQ(road_geometry->junction(2)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().id,
    std::string("s:1-0-2"));
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->id().id,
    std::string("l:1_1_3-1_1_4"));

  // Check the branchpoints
  EXPECT_EQ(road_geometry->num_branch_points(), 4);
  EXPECT_EQ(road_geometry->branch_point(0)->id().id,
    std::string("bp:") + std::to_string(0));
  EXPECT_EQ(road_geometry->branch_point(0)->GetASide()->size(), 1);
  EXPECT_EQ(road_geometry->branch_point(0)->GetBSide()->size(), 0);

  EXPECT_EQ(road_geometry->branch_point(1)->id().id,
    std::string("bp:") + std::to_string(1));
  EXPECT_EQ(road_geometry->branch_point(1)->GetASide()->size(), 1);
  EXPECT_EQ(road_geometry->branch_point(1)->GetBSide()->size(), 1);

  EXPECT_EQ(road_geometry->branch_point(2)->id().id,
    std::string("bp:") + std::to_string(2));
  EXPECT_EQ(road_geometry->branch_point(2)->GetASide()->size(), 1);
  EXPECT_EQ(road_geometry->branch_point(2)->GetBSide()->size(), 1);

  EXPECT_EQ(road_geometry->branch_point(3)->id().id,
    std::string("bp:") + std::to_string(3));
  EXPECT_EQ(road_geometry->branch_point(3)->GetASide()->size(), 1);
  EXPECT_EQ(road_geometry->branch_point(3)->GetBSide()->size(), 0);

  // Check the brach point assigment regarding the lanes
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->
      GetBranchPoint(api::LaneEnd::kStart),
    road_geometry->branch_point(0));
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->
      GetBranchPoint(api::LaneEnd::kFinish),
    road_geometry->branch_point(1));

  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->
      GetBranchPoint(api::LaneEnd::kStart),
    road_geometry->branch_point(1));
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->
      GetBranchPoint(api::LaneEnd::kFinish),
    road_geometry->branch_point(2));

  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->
      GetBranchPoint(api::LaneEnd::kStart),
    road_geometry->branch_point(2));
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->
      GetBranchPoint(api::LaneEnd::kFinish),
    road_geometry->branch_point(3));
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
