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

const double kLinearTolerance = 1e-2;

#define EXPECT_GEO_NEAR(actual, expected, tolerance)         \
do {                                                       \
  const api::GeoPosition _actual(actual);                  \
  const api::GeoPosition _expected(expected);               \
  const double _tolerance = (tolerance);                   \
  EXPECT_NEAR(_actual.x(), _expected.x(), _tolerance);         \
  EXPECT_NEAR(_actual.y(), _expected.y(), _tolerance);         \
  EXPECT_NEAR(_actual.z(), _expected.z(), _tolerance);         \
} while (0)

#define EXPECT_RBOUNDS_EQ(actual, expected)                       \
  do {                                                            \
    const api::RBounds _actual(actual.r_min, actual.r_max);       \
    const api::RBounds _expected(expected.r_min, expected.r_max); \
    EXPECT_EQ(_actual.r_min, _expected.r_min);                    \
    EXPECT_EQ(_actual.r_max, _expected.r_max);                    \
  } while (0)

// Some constructor and operator overloading assertions on DirectedWaypoint
// class
GTEST_TEST(RNDFBuilder, DirectedWaypointClass) {
  std::unique_ptr<DirectedWaypoint> directed_waypoint;

  directed_waypoint = std::make_unique<DirectedWaypoint>();
  EXPECT_EQ(directed_waypoint->Id(), ignition::rndf::UniqueId());
  EXPECT_EQ(directed_waypoint->Position(), ignition::math::Vector3d::Zero);
  EXPECT_EQ(directed_waypoint->Tangent(), ignition::math::Vector3d::Zero);

  {
    std::unique_ptr<DirectedWaypoint> copy_directed_waypoint =
      std::make_unique<DirectedWaypoint>(*directed_waypoint);
    EXPECT_EQ(copy_directed_waypoint->Id(),
      ignition::rndf::UniqueId());
    EXPECT_EQ(copy_directed_waypoint->Position(),
      ignition::math::Vector3d::Zero);
    EXPECT_EQ(copy_directed_waypoint->Tangent(),
      ignition::math::Vector3d::Zero);
  }

  directed_waypoint = std::make_unique<DirectedWaypoint>(
    ignition::rndf::UniqueId(1, 2, 3),
    ignition::math::Vector3d(1, 2, 3),
    false,
    true,
    ignition::math::Vector3d(4, 5, 6));
  EXPECT_EQ(directed_waypoint->Id(), ignition::rndf::UniqueId(1, 2, 3));
  EXPECT_EQ(directed_waypoint->Position(), ignition::math::Vector3d(1, 2, 3));
  EXPECT_EQ(directed_waypoint->Tangent(), ignition::math::Vector3d(4, 5, 6));
  EXPECT_EQ(directed_waypoint->is_entry(), false);
  EXPECT_EQ(directed_waypoint->is_exit(), true);

  {
    DirectedWaypoint copy_directed_waypoint = *directed_waypoint;
    EXPECT_EQ(copy_directed_waypoint.Id(), ignition::rndf::UniqueId(1, 2, 3));
    EXPECT_EQ(copy_directed_waypoint.Position(),
      ignition::math::Vector3d(1, 2, 3));
    EXPECT_EQ(copy_directed_waypoint.Tangent(),
      ignition::math::Vector3d(4, 5, 6));
    EXPECT_EQ(copy_directed_waypoint.is_entry(), false);
    EXPECT_EQ(copy_directed_waypoint.is_exit(), true);
  }
}

// Constructor assertions and checks over the quantity of points for the
// Connetion class
GTEST_TEST(RNDFBuilder, ConnectionClass) {
  const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  const api::RBounds driveable_bounds(-10 / 2., 10. / 2.);
  std::vector<DirectedWaypoint> directed_waypoints;
  directed_waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 1),
    ignition::math::Vector3d(0.0, 0.0, 0.0),
    false, false,
    ignition::math::Vector3d(10.0, 0.0, 0.0)));
  directed_waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 2),
    ignition::math::Vector3d(20.0, 0.0, 0.0),
    false, false,
    ignition::math::Vector3d(10.0, 0.0, 0.0)));

  std::unique_ptr<Connection> connection;
  connection = std::make_unique<Connection>("1_1_1-1_1_2",
    directed_waypoints,
    lane_bounds,
    driveable_bounds);

  EXPECT_EQ(connection->type(), Connection::kSpline);
  EXPECT_EQ(connection->id(), "1_1_1-1_1_2");
  EXPECT_EQ(connection->start().Id(), ignition::rndf::UniqueId(1, 1, 1));
  EXPECT_EQ(connection->end().Id(), ignition::rndf::UniqueId(1, 1, 2));
  EXPECT_EQ(connection->waypoints().size(), 2);

  directed_waypoints.clear();
  EXPECT_THROW(
    std::make_unique<Connection>("1_1_1-1_1_2",
      directed_waypoints,
      lane_bounds,
      driveable_bounds),
    std::runtime_error);
}

// Builder constructor tests. Checks on the bounds and throws assertions
GTEST_TEST(RNDFBuilder, BuilderConstructor) {
  EXPECT_NO_THROW(std::make_unique<Builder>(
    api::RBounds(-5. / 2., 5. / 2.),
    api::RBounds(-10 / 2., 10. / 2.),
    0.01,
    0.01 * M_PI));

  EXPECT_THROW(std::make_unique<Builder>(
    api::RBounds(-15. / 2., 5. / 2.),
    api::RBounds(-10 / 2., 10. / 2.),
    0.01,
    0.01 * M_PI), std::runtime_error);

  EXPECT_THROW(std::make_unique<Builder>(
    api::RBounds(-5. / 2., 15. / 2.),
    api::RBounds(-10 / 2., 10. / 2.),
    0.01,
    0.01 * M_PI), std::runtime_error);
}

// Assertions on the checks when creating a lane using the builder.
// When we get the road_geometry, we check the created values and names.
GTEST_TEST(RNDFBuilder, BuilderLaneConnections) {
  const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  const api::RBounds driveable_bounds(-10 / 2., 10. / 2.);
  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

  EXPECT_THROW(
    builder->CreateLaneConnections(1, 1,
      std::vector<DirectedWaypoint>(),
      lane_bounds,
      driveable_bounds),
    std::runtime_error);

  std::vector<DirectedWaypoint> points = {
    DirectedWaypoint(
      ignition::rndf::UniqueId(1, 1, 1),
      ignition::math::Vector3d(0., 0., 0.))
  };
  EXPECT_THROW(
    builder->CreateLaneConnections(1, 1,
      points,
      lane_bounds,
      driveable_bounds),
    std::runtime_error);

  points.push_back(
    DirectedWaypoint(
      ignition::rndf::UniqueId(1, 1, 2),
      ignition::math::Vector3d(10., 0., 0.)));
  EXPECT_NO_THROW(builder->CreateLaneConnections(1, 1,
    points,
    lane_bounds,
    driveable_bounds));

  auto road_geometry = builder->Build({"One-Lane"});
  EXPECT_EQ(road_geometry->CheckInvariants().size(), 0);

  EXPECT_EQ(road_geometry->num_junctions(), 1);

  auto junction = road_geometry->junction(0);
  EXPECT_EQ(junction->num_segments(), 1);
  EXPECT_EQ(junction->id().id, "j:1_1_1-1_1_2");

  auto segment = junction->segment(0);
  EXPECT_EQ(segment->num_lanes(), 1);
  EXPECT_EQ(segment->id().id, "s:1_1_1-1_1_2");

  auto lane = segment->lane(0);
  EXPECT_EQ(lane->id().id, "l:1_1_1-1_1_2");
  EXPECT_RBOUNDS_EQ(lane->lane_bounds(0.), lane_bounds);
  EXPECT_RBOUNDS_EQ(lane->driveable_bounds(0.), driveable_bounds);
}

// We create a T connection and check its creation and correct invariants from
// the road geometry. It will look like:

//            2.1.1
//            |
//            v
//            2.1.2
//            |
//            v
//            2.1.3
//            \
//             v
// 1.1.1 ----->1.1.2------>1.1.3

// This simple example is useful for checking several Lane and BranchPoint
// functions.
GTEST_TEST(RNDFBuilder, BuilderConnections) {
  const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  const api::RBounds driveable_bounds(-10/ 2., 10. / 2.);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

  {
    std::vector<DirectedWaypoint> points = {
      DirectedWaypoint(
        ignition::rndf::UniqueId(1, 1, 1),
        ignition::math::Vector3d(0., 0., 0.)),
      DirectedWaypoint(
        ignition::rndf::UniqueId(1, 1, 2),
        ignition::math::Vector3d(10., 0., 0.),
        true),
      DirectedWaypoint(
        ignition::rndf::UniqueId(1, 1, 3),
        ignition::math::Vector3d(20., 0., 0.))
    };
    builder->CreateLaneConnections(1, 1, points, lane_bounds, driveable_bounds);
  }
  {
    std::vector<DirectedWaypoint> points = {
      DirectedWaypoint(
        ignition::rndf::UniqueId(2, 1, 1),
        ignition::math::Vector3d(5., 20., 0.)),
      DirectedWaypoint(
        ignition::rndf::UniqueId(2, 1, 2),
        ignition::math::Vector3d(5., 12.5, 0.)),
      DirectedWaypoint(
        ignition::rndf::UniqueId(2, 1, 3),
        ignition::math::Vector3d(5., 5., 0.),
        false,
        true)
    };
    builder->CreateLaneConnections(2, 1, points, lane_bounds, driveable_bounds);
  }

  EXPECT_THROW(builder->CreateConnection(lane_bounds,
    driveable_bounds,
    ignition::rndf::UniqueId(1, 1, 4),
    ignition::rndf::UniqueId(1, 1, 3)), std::runtime_error);
  EXPECT_THROW(builder->CreateConnection(lane_bounds,
    driveable_bounds,
    ignition::rndf::UniqueId(1, 1, 3),
    ignition::rndf::UniqueId(1, 1, 4)), std::runtime_error);

  EXPECT_NO_THROW(builder->CreateConnection(lane_bounds,
    driveable_bounds,
    ignition::rndf::UniqueId(2, 1, 3),
    ignition::rndf::UniqueId(1, 1, 2)));

  auto road_geometry = builder->Build({"ConnectionsChecker"});
  EXPECT_EQ(road_geometry->CheckInvariants().size(), 0);
}

GTEST_TEST(RNDFBuilder, BuildT) {
  const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  const api::RBounds driveable_bounds(-10/ 2., 10. / 2.);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

  {
    std::vector<DirectedWaypoint> points = {
      DirectedWaypoint(
        ignition::rndf::UniqueId(1, 1, 1),
        ignition::math::Vector3d(0., 0., 0.)),
      DirectedWaypoint(
        ignition::rndf::UniqueId(1, 1, 2),
        ignition::math::Vector3d(10., 0., 0.),
        true),
      DirectedWaypoint(
        ignition::rndf::UniqueId(1, 1, 3),
        ignition::math::Vector3d(20., 0., 0.))
    };
    builder->CreateLaneConnections(1, 1, points, lane_bounds, driveable_bounds);
  }
  {
    std::vector<DirectedWaypoint> points = {
      DirectedWaypoint(
        ignition::rndf::UniqueId(2, 1, 1),
        ignition::math::Vector3d(5., 20., 0.)),
      DirectedWaypoint(
        ignition::rndf::UniqueId(2, 1, 2),
        ignition::math::Vector3d(5., 12.5, 0.)),
      DirectedWaypoint(
        ignition::rndf::UniqueId(2, 1, 3),
        ignition::math::Vector3d(5., 5., 0.),
        false,
        true)
    };
    builder->CreateLaneConnections(2, 1, points, lane_bounds, driveable_bounds);
  }
  builder->CreateConnection(lane_bounds,
    driveable_bounds,
    ignition::rndf::UniqueId(2, 1, 3),
    ignition::rndf::UniqueId(1, 1, 2));

  auto road_geometry = builder->Build({"T"});
  EXPECT_NE(road_geometry, nullptr);


  EXPECT_EQ(road_geometry->num_junctions(), 4);
  {
    auto junction = road_geometry->junction(0);
    EXPECT_EQ(junction->num_segments(), 1);
    EXPECT_EQ(junction->id().id, "j:1_1_1-1_1_2");

    auto segment = junction->segment(0);
    EXPECT_EQ(segment->num_lanes(), 1);
    EXPECT_EQ(segment->id().id, "s:1_1_1-1_1_2");

    auto lane = segment->lane(0);
    EXPECT_EQ(lane->id().id, "l:1_1_1-1_1_2");
    EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kStart), nullptr);
    EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kStart)->size(), 1);
    EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kFinish), nullptr);
    EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kFinish)->size(), 2);
    EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kStart), nullptr);
    EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kStart)->size(), 0);
    EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kFinish), nullptr);
    EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kFinish)->size(), 1);

    auto *start_branch = lane->GetBranchPoint(api::LaneEnd::kStart);
    EXPECT_NE(start_branch, nullptr);
    EXPECT_NE(start_branch->GetASide(), nullptr);
    EXPECT_NE(start_branch->GetBSide(), nullptr);
    EXPECT_EQ(start_branch->GetASide()->size(), 1);
    EXPECT_EQ(start_branch->GetBSide()->size(), 0);
    EXPECT_EQ(start_branch->GetASide()->get(0).lane, lane);
    EXPECT_EQ(start_branch->GetASide()->get(0).end, api::LaneEnd::kStart);

    auto *end_branch = lane->GetBranchPoint(api::LaneEnd::kFinish);
    EXPECT_NE(end_branch, nullptr);
    EXPECT_NE(end_branch->GetASide(), nullptr);
    EXPECT_NE(end_branch->GetBSide(), nullptr);
    EXPECT_EQ(end_branch->GetASide()->size(), 2);
    EXPECT_EQ(end_branch->GetBSide()->size(), 1);
    EXPECT_EQ(end_branch->GetASide()->get(0).lane, lane);
    EXPECT_EQ(end_branch->GetASide()->get(0).end, api::LaneEnd::kFinish);
  }
  {
    auto junction = road_geometry->junction(1);
    EXPECT_EQ(junction->num_segments(), 1);
    EXPECT_EQ(junction->id().id, "j:1_1_2-1_1_3");

    auto segment = junction->segment(0);
    EXPECT_EQ(segment->num_lanes(), 1);
    EXPECT_EQ(segment->id().id, "s:1_1_2-1_1_3");

    auto lane = segment->lane(0);
    EXPECT_EQ(lane->id().id, "l:1_1_2-1_1_3");
    EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kStart), nullptr);
    EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kStart)->size(), 1);
    EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kFinish), nullptr);
    EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kFinish)->size(), 1);
    EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kStart), nullptr);
    EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kStart)->size(), 2);
    EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kFinish), nullptr);
    EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kFinish)->size(), 0);

    auto *start_branch = lane->GetBranchPoint(api::LaneEnd::kStart);
    EXPECT_NE(start_branch, nullptr);
    EXPECT_NE(start_branch->GetASide(), nullptr);
    EXPECT_NE(start_branch->GetBSide(), nullptr);
    EXPECT_EQ(start_branch->GetASide()->size(), 2);
    EXPECT_EQ(start_branch->GetBSide()->size(), 1);
    EXPECT_EQ(start_branch->GetBSide()->get(0).lane, lane);
    EXPECT_EQ(start_branch->GetBSide()->get(0).end, api::LaneEnd::kStart);

    auto *end_branch = lane->GetBranchPoint(api::LaneEnd::kFinish);
    EXPECT_NE(end_branch, nullptr);
    EXPECT_NE(end_branch->GetASide(), nullptr);
    EXPECT_NE(end_branch->GetBSide(), nullptr);
    EXPECT_EQ(end_branch->GetASide()->size(), 1);
    EXPECT_EQ(end_branch->GetBSide()->size(), 0);
    EXPECT_EQ(end_branch->GetASide()->get(0).lane, lane);
    EXPECT_EQ(end_branch->GetASide()->get(0).end, api::LaneEnd::kFinish);
  }
  {
    auto junction = road_geometry->junction(2);
    EXPECT_EQ(junction->num_segments(), 1);
    EXPECT_EQ(junction->id().id, "j:2_1_1-2_1_3");

    auto segment = junction->segment(0);
    EXPECT_EQ(segment->num_lanes(), 1);
    EXPECT_EQ(segment->id().id, "s:2_1_1-2_1_3");

    auto lane = segment->lane(0);
    EXPECT_EQ(lane->id().id, "l:2_1_1-2_1_3");
    EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kStart), nullptr);
    EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kStart)->size(), 1);
    EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kFinish), nullptr);
    EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kFinish)->size(), 1);
    EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kStart), nullptr);
    EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kStart)->size(), 0);
    EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kFinish), nullptr);
    EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kFinish)->size(), 1);

    auto *start_branch = lane->GetBranchPoint(api::LaneEnd::kStart);
    EXPECT_NE(start_branch, nullptr);
    EXPECT_NE(start_branch->GetASide(), nullptr);
    EXPECT_NE(start_branch->GetBSide(), nullptr);
    EXPECT_EQ(start_branch->GetASide()->size(), 1);
    EXPECT_EQ(start_branch->GetBSide()->size(), 0);
    EXPECT_EQ(start_branch->GetASide()->get(0).lane, lane);
    EXPECT_EQ(start_branch->GetASide()->get(0).end, api::LaneEnd::kStart);

    auto *end_branch = lane->GetBranchPoint(api::LaneEnd::kFinish);
    EXPECT_NE(end_branch, nullptr);
    EXPECT_NE(end_branch->GetASide(), nullptr);
    EXPECT_NE(end_branch->GetBSide(), nullptr);
    EXPECT_EQ(end_branch->GetASide()->size(), 1);
    EXPECT_EQ(end_branch->GetBSide()->size(), 1);
    EXPECT_EQ(end_branch->GetASide()->get(0).lane, lane);
    EXPECT_EQ(end_branch->GetASide()->get(0).end, api::LaneEnd::kFinish);
  }
  {
    auto junction = road_geometry->junction(3);
    EXPECT_EQ(junction->num_segments(), 1);
    EXPECT_EQ(junction->id().id, "j:2_1_3-1_1_2");

    auto segment = junction->segment(0);
    EXPECT_EQ(segment->num_lanes(), 1);
    EXPECT_EQ(segment->id().id, "s:2_1_3-1_1_2");
    auto lane = segment->lane(0);
    EXPECT_EQ(lane->id().id, "l:2_1_3-1_1_2");
    EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kStart), nullptr);
    EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kStart)->size(), 1);
    EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kFinish), nullptr);
    EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kFinish)->size(), 2);
    EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kStart), nullptr);
    EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kStart)->size(), 1);
    EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kFinish), nullptr);
    EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kFinish)->size(), 1);

    auto *start_branch = lane->GetBranchPoint(api::LaneEnd::kStart);
    EXPECT_NE(start_branch, nullptr);
    EXPECT_NE(start_branch->GetASide(), nullptr);
    EXPECT_NE(start_branch->GetBSide(), nullptr);
    EXPECT_EQ(start_branch->GetASide()->size(), 1);
    EXPECT_EQ(start_branch->GetBSide()->size(), 1);
    EXPECT_EQ(start_branch->GetBSide()->get(0).lane, lane);
    EXPECT_EQ(start_branch->GetBSide()->get(0).end, api::LaneEnd::kStart);

    auto *end_branch = lane->GetBranchPoint(api::LaneEnd::kFinish);
    EXPECT_NE(end_branch, nullptr);
    EXPECT_NE(end_branch->GetASide(), nullptr);
    EXPECT_NE(end_branch->GetBSide(), nullptr);
    EXPECT_EQ(end_branch->GetASide()->size(), 2);
    EXPECT_EQ(end_branch->GetBSide()->size(), 1);
    EXPECT_EQ(end_branch->GetASide()->get(1).lane, lane);
    EXPECT_EQ(end_branch->GetASide()->get(1).end, api::LaneEnd::kFinish);
  }
}


// We create a T connection and check its creation and correct invariants from
// the road geometry. It will look like:

//                                    2.1.1
//                                    |
//                                    v
//                                    2.1.2
//                                    |
//                                    v
//                                    2.1.3
//                                    \
//                                     v
// 1.1.1 ----->1.1.2------>1.1.3------>1.1.4------>1.1.5

// This simple example is useful for checking the how the waypoint grouping is
// working
GTEST_TEST(RNDFBuilder, BuildTToCheckWaypointGrouping) {
  const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  const api::RBounds driveable_bounds(-10/ 2., 10. / 2.);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

  {
    std::vector<DirectedWaypoint> points = {
      DirectedWaypoint(
        ignition::rndf::UniqueId(1, 1, 1),
        ignition::math::Vector3d(0., 0., 0.)),
      DirectedWaypoint(
        ignition::rndf::UniqueId(1, 1, 2),
        ignition::math::Vector3d(5., 0., 0.)),
      DirectedWaypoint(
        ignition::rndf::UniqueId(1, 1, 3),
        ignition::math::Vector3d(5., 0., 0.)),
      DirectedWaypoint(
        ignition::rndf::UniqueId(1, 1, 4),
        ignition::math::Vector3d(20., 0., 0.),
        true,
        false),
      DirectedWaypoint(
        ignition::rndf::UniqueId(1, 1, 5),
        ignition::math::Vector3d(25., 0., 0.)),
    };
    builder->CreateLaneConnections(1, 1, points, lane_bounds, driveable_bounds);
  }
  {
    std::vector<DirectedWaypoint> points = {
      DirectedWaypoint(
        ignition::rndf::UniqueId(2, 1, 1),
        ignition::math::Vector3d(5., 20., 0.)),
      DirectedWaypoint(
        ignition::rndf::UniqueId(2, 1, 2),
        ignition::math::Vector3d(5., 12.5, 0.)),
      DirectedWaypoint(
        ignition::rndf::UniqueId(2, 1, 3),
        ignition::math::Vector3d(5., 5., 0.),
        false,
        true)
    };
    builder->CreateLaneConnections(2, 1, points, lane_bounds, driveable_bounds);
  }
  builder->CreateConnection(lane_bounds,
    driveable_bounds,
    ignition::rndf::UniqueId(2, 1, 3),
    ignition::rndf::UniqueId(1, 1, 4));

  auto road_geometry = builder->Build({"L"});
  EXPECT_NE(road_geometry, nullptr);

  EXPECT_EQ(road_geometry->num_junctions(), 4);
  EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1_1_1-1_1_4");
  EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1_1_4-1_1_5");
  EXPECT_EQ(road_geometry->junction(2)->id().id, "j:2_1_1-2_1_3");
  EXPECT_EQ(road_geometry->junction(3)->id().id, "j:2_1_3-1_1_4");
}

//        * --> *
//  * --> * --> *
GTEST_TEST(RNDFBuilder, LaneStartsAfter) {
  const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

  ConnectedLane l1, l2;
  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 1),
    ignition::math::Vector3d(5.0, 0.0, 0.0)));
  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 2),
    ignition::math::Vector3d(10.0, 0.0, 0.0)));

  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 1),
    ignition::math::Vector3d(0.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 2),
    ignition::math::Vector3d(5.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 3),
    ignition::math::Vector3d(10.0, 5.0, 0.0)));

  l1.lane_bounds = lane_bounds;
  l2.lane_bounds = lane_bounds;
  l1.driveable_bounds = driveable_bounds;
  l2.driveable_bounds = driveable_bounds;
  std::vector<ConnectedLane> connected_lanes = {l1, l2};

  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"LaneStartsAfter"});
  EXPECT_NE(road_geometry, nullptr);

  EXPECT_EQ(road_geometry->num_junctions(), 3);
  EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1_1_1-1_1_2");
  EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1_2_1-1_2_2");
  EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1_2_2-1_2_3");
}

//              * --> *
//  * --> * --> * --> *
GTEST_TEST(RNDFBuilder, LaneStartsAfter2) {
  const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

  ConnectedLane l1, l2;

  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 1),
    ignition::math::Vector3d(5.0, 0.0, 0.0)));
  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 2),
    ignition::math::Vector3d(10.0, 0.0, 0.0)));

  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 1),
    ignition::math::Vector3d(0.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 2),
    ignition::math::Vector3d(2.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 3),
    ignition::math::Vector3d(5.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 4),
    ignition::math::Vector3d(10.0, 5.0, 0.0)));

  l1.lane_bounds = lane_bounds;
  l2.lane_bounds = lane_bounds;
  l1.driveable_bounds = driveable_bounds;
  l2.driveable_bounds = driveable_bounds;
  std::vector<ConnectedLane> connected_lanes = {l1, l2};

  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"LaneStartsAfter2"});
  EXPECT_NE(road_geometry, nullptr);

  EXPECT_EQ(road_geometry->num_junctions(), 4);
  EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1_1_1-1_1_2");
  EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1_2_1-1_2_2");
  EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1_2_2-1_2_3");
  EXPECT_EQ(road_geometry->junction(3)->id().id, "j:1_2_3-1_2_4");
}


//  * --> *
//  * --> * --> *
GTEST_TEST(RNDFBuilder, LaneEndsBefore) {
  const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

  ConnectedLane l1, l2;

  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 1),
    ignition::math::Vector3d(0.0, 0.0, 0.0)));
  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 2),
    ignition::math::Vector3d(5.0, 0.0, 0.0)));

  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 1),
    ignition::math::Vector3d(0.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 2),
    ignition::math::Vector3d(5.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 3),
    ignition::math::Vector3d(10.0, 5.0, 0.0)));

  l1.lane_bounds = lane_bounds;
  l2.lane_bounds = lane_bounds;
  l1.driveable_bounds = driveable_bounds;
  l2.driveable_bounds = driveable_bounds;
  std::vector<ConnectedLane> connected_lanes = {l1, l2};

  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"LaneEndsBefore"});
  EXPECT_NE(road_geometry, nullptr);

  EXPECT_EQ(road_geometry->num_junctions(), 3);
  EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1_1_1-1_1_2");
  EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1_2_1-1_2_2");
  EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1_2_2-1_2_3");
}

//  * --> *
//  * --> * --> * --> *
GTEST_TEST(RNDFBuilder, LaneEndsBefore2) {
  const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

  ConnectedLane l1, l2;

  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 1),
    ignition::math::Vector3d(0.0, 0.0, 0.0)));
  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 2),
    ignition::math::Vector3d(5.0, 0.0, 0.0)));

  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 1),
    ignition::math::Vector3d(0.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 2),
    ignition::math::Vector3d(5.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 3),
    ignition::math::Vector3d(10.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 4),
    ignition::math::Vector3d(15.0, 5.0, 0.0)));

  l1.lane_bounds = lane_bounds;
  l2.lane_bounds = lane_bounds;
  l1.driveable_bounds = driveable_bounds;
  l2.driveable_bounds = driveable_bounds;
  std::vector<ConnectedLane> connected_lanes = {l1, l2};

  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"LaneEndsBefore2"});
  EXPECT_NE(road_geometry, nullptr);

  EXPECT_EQ(road_geometry->num_junctions(), 4);
  EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1_1_1-1_1_2");
  EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1_2_1-1_2_2");
  EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1_2_2-1_2_3");
  EXPECT_EQ(road_geometry->junction(3)->id().id, "j:1_2_3-1_2_4");
}

//        * --> *
//  * --> * --> * --> *
GTEST_TEST(RNDFBuilder, LaneStartsAfterAndEndsBefore) {
  const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

  ConnectedLane l1, l2;

  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 1),
    ignition::math::Vector3d(5.0, 0.0, 0.0)));
  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 2),
    ignition::math::Vector3d(10.0, 0.0, 0.0)));

  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 1),
    ignition::math::Vector3d(0.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 2),
    ignition::math::Vector3d(5.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 3),
    ignition::math::Vector3d(10.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 4),
    ignition::math::Vector3d(15.0, 5.0, 0.0)));

  l1.lane_bounds = lane_bounds;
  l2.lane_bounds = lane_bounds;
  l1.driveable_bounds = driveable_bounds;
  l2.driveable_bounds = driveable_bounds;
  std::vector<ConnectedLane> connected_lanes = {l1, l2};

  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"LaneStartsAfterAndEndsBefore"});
  EXPECT_NE(road_geometry, nullptr);

  EXPECT_EQ(road_geometry->num_junctions(), 4);
  EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1_1_1-1_1_2");
  EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1_2_1-1_2_2");
  EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1_2_2-1_2_3");
  EXPECT_EQ(road_geometry->junction(3)->id().id, "j:1_2_3-1_2_4");
}

//     * --> *
// * ------> *
GTEST_TEST(RNDFBuilder, LaneStartsAfterButNotInPhase) {
  const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

  ConnectedLane l1, l2;

  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 1),
    ignition::math::Vector3d(7.5, 0.0, 0.0)));
  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 2),
    ignition::math::Vector3d(10.0, 0.0, 0.0)));

  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 1),
    ignition::math::Vector3d(0.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 2),
    ignition::math::Vector3d(10.0, 5.0, 0.0)));

  l1.lane_bounds = lane_bounds;
  l2.lane_bounds = lane_bounds;
  l1.driveable_bounds = driveable_bounds;
  l2.driveable_bounds = driveable_bounds;
  std::vector<ConnectedLane> connected_lanes = {l1, l2};

  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"LaneStartsAfterButNotInPhase"});
  EXPECT_NE(road_geometry, nullptr);

  EXPECT_EQ(road_geometry->num_junctions(), 3);
  EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1_1_1-1_1_2");
  EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1_2_1-1_2_3");
  EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1_2_3-1_2_2");
  {
  auto geo_position = road_geometry->junction(1)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(
      road_geometry->junction(1)->segment(0)->lane(0)->length(), 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(7.5, 5.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(2)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(0., 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(7.5, 5.0, 0.0),
    kLinearTolerance);
  }
}

//         * --> *
// * --> * ----> *
GTEST_TEST(RNDFBuilder, LaneStartsAfterButNotInPhase2) {
  const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

  ConnectedLane l1, l2;

  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 1),
    ignition::math::Vector3d(7.5, 0.0, 0.0)));
  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 2),
    ignition::math::Vector3d(10.0, 0.0, 0.0)));

  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 1),
    ignition::math::Vector3d(0.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 2),
    ignition::math::Vector3d(5.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 3),
    ignition::math::Vector3d(10.0, 5.0, 0.0)));

  l1.lane_bounds = lane_bounds;
  l2.lane_bounds = lane_bounds;
  l1.driveable_bounds = driveable_bounds;
  l2.driveable_bounds = driveable_bounds;
  std::vector<ConnectedLane> connected_lanes = {l1, l2};

  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"LaneStartsAfterButNotInPhase2"});
  EXPECT_NE(road_geometry, nullptr);

  EXPECT_EQ(road_geometry->num_junctions(), 4);
  EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1_1_1-1_1_2");
  EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1_2_1-1_2_2");
  EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1_2_2-1_2_4");
  EXPECT_EQ(road_geometry->junction(3)->id().id, "j:1_2_4-1_2_3");
  {
  auto geo_position = road_geometry->junction(2)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(
      road_geometry->junction(2)->segment(0)->lane(0)->length(), 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(7.5, 5.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(3)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(0., 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(7.5, 5.0, 0.0),
    kLinearTolerance);
  }
}

//     * --> *
// * ----------> *
GTEST_TEST(RNDFBuilder, LaneStartsAfterEndsAfterNotInPhase) {
  const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

  ConnectedLane l1, l2;

  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 1),
    ignition::math::Vector3d(2.5, 0.0, 0.0)));
  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 2),
    ignition::math::Vector3d(7.5, 0.0, 0.0)));

  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 1),
    ignition::math::Vector3d(0.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 2),
    ignition::math::Vector3d(10.0, 5.0, 0.0)));

  l1.lane_bounds = lane_bounds;
  l2.lane_bounds = lane_bounds;
  l1.driveable_bounds = driveable_bounds;
  l2.driveable_bounds = driveable_bounds;
  std::vector<ConnectedLane> connected_lanes = {l1, l2};

  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"LaneStartsAfterEndsAfterNotInPhase"});
  EXPECT_NE(road_geometry, nullptr);

  EXPECT_EQ(road_geometry->num_junctions(), 4);
  EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1_1_1-1_1_2");
  EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1_2_1-1_2_3");
  EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1_2_3-1_2_4");
  EXPECT_EQ(road_geometry->junction(3)->id().id, "j:1_2_4-1_2_2");

  {
  auto geo_position = road_geometry->junction(1)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(
      road_geometry->junction(1)->segment(0)->lane(0)->length(), 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(2.5, 5.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(2)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(0., 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(2.5, 5.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(2)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(
      road_geometry->junction(2)->segment(0)->lane(0)->length(), 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(7.5, 5.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(3)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(0., 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(7.5, 5.0, 0.0),
    kLinearTolerance);
  }
}

//     * ----> *
// * ----> * -----> *
GTEST_TEST(RNDFBuilder, TotallyDephasedLanes) {
  const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

  ConnectedLane l1, l2;

  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 1),
    ignition::math::Vector3d(2.5, 0.0, 0.0)));
  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 2),
    ignition::math::Vector3d(7.5, 0.0, 0.0)));

  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 1),
    ignition::math::Vector3d(0.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 2),
    ignition::math::Vector3d(5.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 3),
    ignition::math::Vector3d(10.0, 5.0, 0.0)));

  l1.lane_bounds = lane_bounds;
  l2.lane_bounds = lane_bounds;
  l1.driveable_bounds = driveable_bounds;
  l2.driveable_bounds = driveable_bounds;
  std::vector<ConnectedLane> connected_lanes = {l1, l2};

  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"TotallyDephasedLanes"});
  EXPECT_NE(road_geometry, nullptr);

  EXPECT_EQ(road_geometry->num_junctions(), 6);
  EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1_1_1-1_1_4");
  EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1_1_4-1_1_2");
  EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1_2_1-1_2_4");
  EXPECT_EQ(road_geometry->junction(3)->id().id, "j:1_2_4-1_2_2");
  EXPECT_EQ(road_geometry->junction(4)->id().id, "j:1_2_2-1_2_5");
  EXPECT_EQ(road_geometry->junction(5)->id().id, "j:1_2_5-1_2_3");

  {
  auto geo_position = road_geometry->junction(0)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(
      road_geometry->junction(0)->segment(0)->lane(0)->length(), 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 0.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(1)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(0., 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 0.0, 0.0),
    kLinearTolerance);
  }

  {
  auto geo_position = road_geometry->junction(2)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(
      road_geometry->junction(2)->segment(0)->lane(0)->length(), 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(2.5, 5.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(3)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(0., 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(2.5, 5.0, 0.0),
    kLinearTolerance);
  }

  {
  auto geo_position = road_geometry->junction(4)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(
      road_geometry->junction(4)->segment(0)->lane(0)->length(), 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(7.5, 5.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(5)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(0., 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(7.5, 5.0, 0.0),
    kLinearTolerance);
  }
}

// * ---------> *
//     * -----> *
//         * -> *
GTEST_TEST(RNDFBuilder, ThreeLaneStartWithPhaseEndCoherent) {
  const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

  ConnectedLane l1, l2, l3;

  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 1),
    ignition::math::Vector3d(0.0, 0.0, 0.0)));
  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 2),
    ignition::math::Vector3d(10.0, 0.0, 0.0)));

  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 1),
    ignition::math::Vector3d(2.5, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 2),
    ignition::math::Vector3d(10.0, 5.0, 0.0)));

  l3.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 3, 1),
    ignition::math::Vector3d(5.0, 10.0, 0.0)));
  l3.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 3, 2),
    ignition::math::Vector3d(10.0, 10.0, 0.0)));

  l1.lane_bounds = lane_bounds;
  l2.lane_bounds = lane_bounds;
  l3.lane_bounds = lane_bounds;
  l1.driveable_bounds = driveable_bounds;
  l2.driveable_bounds = driveable_bounds;
  l3.driveable_bounds = driveable_bounds;
  std::vector<ConnectedLane> connected_lanes = {l1, l2, l3};

  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"ThreeLaneStartWithPhaseEndCoherent"});
  EXPECT_NE(road_geometry, nullptr);

  EXPECT_EQ(road_geometry->num_junctions(), 6);
  EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1_1_1-1_1_3");
  EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1_1_3-1_1_4");
  EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1_1_4-1_1_2");
  EXPECT_EQ(road_geometry->junction(3)->id().id, "j:1_2_1-1_2_4");
  EXPECT_EQ(road_geometry->junction(4)->id().id, "j:1_2_4-1_2_2");
  EXPECT_EQ(road_geometry->junction(5)->id().id, "j:1_3_1-1_3_2");

  {
  auto geo_position = road_geometry->junction(0)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(
      road_geometry->junction(0)->segment(0)->lane(0)->length(), 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(2.5, 0.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(1)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(0., 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(2.5, 0.0, 0.0),
    kLinearTolerance);
  }

  {
  auto geo_position = road_geometry->junction(1)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(
      road_geometry->junction(1)->segment(0)->lane(0)->length(), 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 0.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(2)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(0., 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 0.0, 0.0),
    kLinearTolerance);
  }

  {
  auto geo_position = road_geometry->junction(3)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(
      road_geometry->junction(3)->segment(0)->lane(0)->length(), 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 5.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(4)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(0., 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 5.0, 0.0),
    kLinearTolerance);
  }
}

//   * ------> *
//     * -------> *
// * --------------> *
GTEST_TEST(RNDFBuilder, ThreeLaneStartAndEndWithPhases) {
  const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

  ConnectedLane l1, l2, l3;

  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 1),
    ignition::math::Vector3d(2.5, 0.0, 0.0)));
  l1.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 2),
    ignition::math::Vector3d(10.0, 0.0, 0.0)));

  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 1),
    ignition::math::Vector3d(5.0, 5.0, 0.0)));
  l2.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 2),
    ignition::math::Vector3d(12.5, 5.0, 0.0)));

  l3.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 3, 1),
    ignition::math::Vector3d(0.0, 10.0, 0.0)));
  l3.waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 3, 2),
    ignition::math::Vector3d(15.0, 10.0, 0.0)));

  l1.lane_bounds = lane_bounds;
  l2.lane_bounds = lane_bounds;
  l3.lane_bounds = lane_bounds;
  l1.driveable_bounds = driveable_bounds;
  l2.driveable_bounds = driveable_bounds;
  l3.driveable_bounds = driveable_bounds;
  std::vector<ConnectedLane> connected_lanes = {l1, l2, l3};

  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"ThreeLaneStartAndEndWithPhases"});
  EXPECT_NE(road_geometry, nullptr);

  EXPECT_EQ(road_geometry->num_junctions(), 9);
  EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1_1_1-1_1_4");
  EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1_1_4-1_1_2");

  EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1_2_1-1_2_5");
  EXPECT_EQ(road_geometry->junction(3)->id().id, "j:1_2_5-1_2_2");

  EXPECT_EQ(road_geometry->junction(4)->id().id, "j:1_3_1-1_3_3");
  EXPECT_EQ(road_geometry->junction(5)->id().id, "j:1_3_3-1_3_4");
  EXPECT_EQ(road_geometry->junction(6)->id().id, "j:1_3_4-1_3_5");
  EXPECT_EQ(road_geometry->junction(7)->id().id, "j:1_3_5-1_3_6");
  EXPECT_EQ(road_geometry->junction(8)->id().id, "j:1_3_6-1_3_2");
  // Lane 1
  {
  auto geo_position = road_geometry->junction(0)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(
      road_geometry->junction(0)->segment(0)->lane(0)->length(), 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 0.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(1)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(0., 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 0.0, 0.0),
    kLinearTolerance);
  }
  // Lane 2
  {
  auto geo_position = road_geometry->junction(2)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(
      road_geometry->junction(2)->segment(0)->lane(0)->length(), 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 5.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(3)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(0., 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 5.0, 0.0),
    kLinearTolerance);
  }
  // Lane 3
  {
  auto geo_position = road_geometry->junction(4)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(
      road_geometry->junction(4)->segment(0)->lane(0)->length(), 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(2.5, 10.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(5)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(0., 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(2.5, 10.0, 0.0),
    kLinearTolerance);
  }

  {
  auto geo_position = road_geometry->junction(5)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(
      road_geometry->junction(5)->segment(0)->lane(0)->length(), 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 10.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(6)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(0., 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 10.0, 0.0),
    kLinearTolerance);
  }

  {
  auto geo_position = road_geometry->junction(6)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(
      road_geometry->junction(6)->segment(0)->lane(0)->length(), 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 10.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(7)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(0., 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 10.0, 0.0),
    kLinearTolerance);
  }

  {
  auto geo_position = road_geometry->junction(7)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(
      road_geometry->junction(7)->segment(0)->lane(0)->length(), 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(12.5, 10.0, 0.0),
    kLinearTolerance);
  }
  {
  auto geo_position = road_geometry->junction(8)->segment(0)->lane(0)->
    ToGeoPosition(api::LanePosition(0., 0., 0.));
  EXPECT_GEO_NEAR(geo_position, api::GeoPosition(12.5, 10.0, 0.0),
    kLinearTolerance);
  }
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
