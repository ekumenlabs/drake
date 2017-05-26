// #include <memory>
// #include <string>
// #include <tuple>
// #include <vector>

// #include <gtest/gtest.h>

// #include <ignition/math/Vector3.hh>

// #include <ignition/rndf/UniqueId.hh>

// #include "drake/common/drake_throw.h"
// #include "drake/automotive/maliput/rndf/builder.h"

// namespace drake {
// namespace maliput {
// namespace rndf {

// const double kLinearTolerance = 1e-2;

// #define EXPECT_GEO_NEAR(actual, expected, tolerance)         \
// do {                                                       \
//   const api::GeoPosition _actual(actual);                  \
//   const api::GeoPosition _expected(expected);               \
//   const double _tolerance = (tolerance);                   \
//   EXPECT_NEAR(_actual.x(), _expected.x(), _tolerance);         \
//   EXPECT_NEAR(_actual.y(), _expected.y(), _tolerance);         \
//   EXPECT_NEAR(_actual.z(), _expected.z(), _tolerance);         \
// } while (0)

// #define EXPECT_RBOUNDS_EQ(actual, expected)                       \
//   do {                                                            \
//     const api::RBounds _actual(actual.r_min, actual.r_max);       \
//     const api::RBounds _expected(expected.r_min, expected.r_max); \
//     EXPECT_EQ(_actual.r_min, _expected.r_min);                    \
//     EXPECT_EQ(_actual.r_max, _expected.r_max);                    \
//   } while (0)

// <<<<<<< HEAD
// // // Some constructor and operator overloading assertions on DirectedWaypoint
// // // class
// // GTEST_TEST(RNDFBuilder, DirectedWaypointClass) {
// //   std::unique_ptr<DirectedWaypoint> directed_waypoint;

// //   directed_waypoint = std::make_unique<DirectedWaypoint>();
// //   EXPECT_EQ(directed_waypoint->Id(), ignition::rndf::UniqueId());
// //   EXPECT_EQ(directed_waypoint->Position(), ignition::math::Vector3d::Zero);
// //   EXPECT_EQ(directed_waypoint->Tangent(), ignition::math::Vector3d::Zero);

// //   {
// //     std::unique_ptr<DirectedWaypoint> copy_directed_waypoint =
// //       std::make_unique<DirectedWaypoint>(*directed_waypoint);
// //     EXPECT_EQ(copy_directed_waypoint->Id(),
// //       ignition::rndf::UniqueId());
// //     EXPECT_EQ(copy_directed_waypoint->Position(),
// //       ignition::math::Vector3d::Zero);
// //     EXPECT_EQ(copy_directed_waypoint->Tangent(),
// //       ignition::math::Vector3d::Zero);
// //   }

// //   directed_waypoint = std::make_unique<DirectedWaypoint>(
// //     ignition::rndf::UniqueId(1, 2, 3),
// //     ignition::math::Vector3d(1, 2, 3),
// //     false,
// //     true,
// //     ignition::math::Vector3d(4, 5, 6));
// //   EXPECT_EQ(directed_waypoint->Id(), ignition::rndf::UniqueId(1, 2, 3));
// //   EXPECT_EQ(directed_waypoint->Position(), ignition::math::Vector3d(1, 2, 3));
// //   EXPECT_EQ(directed_waypoint->Tangent(), ignition::math::Vector3d(4, 5, 6));
// //   EXPECT_EQ(directed_waypoint->is_entry(), false);
// //   EXPECT_EQ(directed_waypoint->is_exit(), true);

// //   {
// //     DirectedWaypoint copy_directed_waypoint = *directed_waypoint;
// //     EXPECT_EQ(copy_directed_waypoint.Id(), ignition::rndf::UniqueId(1, 2, 3));
// //     EXPECT_EQ(copy_directed_waypoint.Position(),
// //       ignition::math::Vector3d(1, 2, 3));
// //     EXPECT_EQ(copy_directed_waypoint.Tangent(),
// //       ignition::math::Vector3d(4, 5, 6));
// //     EXPECT_EQ(copy_directed_waypoint.is_entry(), false);
// //     EXPECT_EQ(copy_directed_waypoint.is_exit(), true);
// //   }
// // }

// // // Constructor assertions and checks over the quantity of points for the
// // // Connetion class
// // GTEST_TEST(RNDFBuilder, ConnectionClass) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-10 / 2., 10. / 2.);
// //   std::vector<DirectedWaypoint> directed_waypoints;
// //   directed_waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(0.0, 0.0, 0.0),
// //     false, false,
// //     ignition::math::Vector3d(10.0, 0.0, 0.0)));
// //   directed_waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(20.0, 0.0, 0.0),
// //     false, false,
// //     ignition::math::Vector3d(10.0, 0.0, 0.0)));

// //   std::unique_ptr<Connection> connection;
// //   connection = std::make_unique<Connection>("1_1_1-1_1_2",
// //     directed_waypoints,
// //     lane_bounds,
// //     driveable_bounds);

// //   EXPECT_EQ(connection->type(), Connection::kSpline);
// //   EXPECT_EQ(connection->id(), "1_1_1-1_1_2");
// //   EXPECT_EQ(connection->start().Id(), ignition::rndf::UniqueId(1, 1, 1));
// //   EXPECT_EQ(connection->end().Id(), ignition::rndf::UniqueId(1, 1, 2));
// //   EXPECT_EQ(connection->waypoints().size(), 2);

// //   directed_waypoints.clear();
// //   EXPECT_THROW(
// //     std::make_unique<Connection>("1_1_1-1_1_2",
// //       directed_waypoints,
// //       lane_bounds,
// //       driveable_bounds),
// //     std::runtime_error);
// // }

// // // Builder constructor tests. Checks on the bounds and throws assertions
// // GTEST_TEST(RNDFBuilder, BuilderConstructor) {
// //   EXPECT_NO_THROW(std::make_unique<Builder>(
// //     api::RBounds(-5. / 2., 5. / 2.),
// //     api::RBounds(-10 / 2., 10. / 2.),
// //     0.01,
// //     0.01 * M_PI));

// //   EXPECT_THROW(std::make_unique<Builder>(
// //     api::RBounds(-15. / 2., 5. / 2.),
// //     api::RBounds(-10 / 2., 10. / 2.),
// //     0.01,
// //     0.01 * M_PI), std::runtime_error);

// //   EXPECT_THROW(std::make_unique<Builder>(
// //     api::RBounds(-5. / 2., 15. / 2.),
// //     api::RBounds(-10 / 2., 10. / 2.),
// //     0.01,
// //     0.01 * M_PI), std::runtime_error);
// // }

// // // Assertions on the checks when creating a lane using the builder.
// // // When we get the road_geometry, we check the created values and names.
// // GTEST_TEST(RNDFBuilder, BuilderLaneConnections) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-10 / 2., 10. / 2.);
// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2;
// //   l1.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   std::vector<ConnectedLane> lane_vector;
// //   // Check for nullptr validation
// //   EXPECT_THROW(
// //     builder->CreateSegmentConnections(1, nullptr), std::runtime_error);
// //   // Check for lanes vector validation
// //   EXPECT_THROW(
// //     builder->CreateSegmentConnections(1, &lane_vector), std::runtime_error);
// //   // Check for lane waypoint validation
// //   lane_vector.push_back(l1);
// //   EXPECT_THROW(
// //     builder->CreateSegmentConnections(1, &lane_vector), std::runtime_error);
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(0., 0., 0.)));
// //   lane_vector.clear();
// //   lane_vector.push_back(l1);
// //   EXPECT_THROW(
// //     builder->CreateSegmentConnections(1, &lane_vector), std::runtime_error);
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(10., 0., 0.)));
// //   lane_vector.clear();
// //   lane_vector.push_back(l1);
// //   EXPECT_NO_THROW(
// //     builder->CreateSegmentConnections(1, &lane_vector));
// //   lane_vector.clear();
// //   lane_vector.push_back(l1);
// //   lane_vector.push_back(l2);
// //   EXPECT_THROW(
// //     builder->CreateSegmentConnections(2, &lane_vector), std::runtime_error);

// //   auto road_geometry = builder->Build({"One-Lane"});

// //   EXPECT_EQ(road_geometry->CheckInvariants().size(), 0);

// //   EXPECT_EQ(road_geometry->num_junctions(), 1);

// //   auto junction = road_geometry->junction(0);
// //   EXPECT_EQ(junction->num_segments(), 1);
// //   EXPECT_EQ(junction->id().id, "j:1-0");

// //   auto segment = junction->segment(0);
// //   EXPECT_EQ(segment->num_lanes(), 1);
// //   EXPECT_EQ(segment->id().id, "s:1-0");

// //   auto lane = segment->lane(0);
// //   EXPECT_EQ(lane->id().id, "l:1_1_1-1_1_2");
// //   EXPECT_RBOUNDS_EQ(lane->lane_bounds(0.), lane_bounds);
// //   EXPECT_RBOUNDS_EQ(lane->driveable_bounds(0.), driveable_bounds);
// // }

// // // We create a T connection and check its creation and correct invariants from
// // // the road geometry. It will look like:

// // //            2.1.1
// // //            |
// // //            v
// // //            2.1.2
// // //            |
// // //            v
// // //            2.1.3
// // //            \
// // //             v
// // // 1.1.1 ----->1.1.2------>1.1.3

// // // This simple example is useful for checking several Lane and BranchPoint
// // // functions.
// // GTEST_TEST(RNDFBuilder, BuilderConnections) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-10/ 2., 10. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2;
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(0., 0., 0.)));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(10., 0., 0.),
// //     true));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 3),
// //     ignition::math::Vector3d(20., 0., 0.)));

// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(2, 1, 1),
// //     ignition::math::Vector3d(5., 20., 0.)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(2, 1, 2),
// //     ignition::math::Vector3d(5., 12.5, 0.)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(2, 1, 3),
// //     ignition::math::Vector3d(5., 5., 0.),
// //     false,
// //     true));

// //   l1.lane_bounds = lane_bounds;
// //   l2.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   l2.driveable_bounds = driveable_bounds;
// //   {
// //     std::vector<ConnectedLane> connected_lanes = {l1};
// //     builder->CreateSegmentConnections(1, &connected_lanes);
// //   }
// //   {
// //     std::vector<ConnectedLane> connected_lanes = {l2};
// //     builder->CreateSegmentConnections(2, &connected_lanes);
// //   }

// //   EXPECT_THROW(builder->CreateConnection(lane_bounds,
// //     driveable_bounds,
// //     ignition::rndf::UniqueId(1, 1, 4),
// //     ignition::rndf::UniqueId(1, 1, 3)), std::runtime_error);
// //   EXPECT_THROW(builder->CreateConnection(lane_bounds,
// //     driveable_bounds,
// //     ignition::rndf::UniqueId(1, 1, 3),
// //     ignition::rndf::UniqueId(1, 1, 4)), std::runtime_error);

// //   EXPECT_NO_THROW(builder->CreateConnection(lane_bounds,
// //     driveable_bounds,
// //     ignition::rndf::UniqueId(2, 1, 3),
// //     ignition::rndf::UniqueId(1, 1, 2)));

// //   auto road_geometry = builder->Build({"ConnectionsChecker"});
// //   EXPECT_EQ(road_geometry->CheckInvariants().size(), 0);
// // }


// // GTEST_TEST(RNDFBuilder, BuildT) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-10. / 2., 10. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2;
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(0., 0., 0.)));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(10., 0., 0.),
// //     true));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 3),
// //     ignition::math::Vector3d(20., 0., 0.)));

// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(2, 1, 1),
// //     ignition::math::Vector3d(5., 20., 0.)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(2, 1, 2),
// //     ignition::math::Vector3d(5., 12.5, 0.)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(2, 1, 3),
// //     ignition::math::Vector3d(5., 5., 0.),
// //     false,
// //     true));

// //   l1.lane_bounds = lane_bounds;
// //   l2.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   l2.driveable_bounds = driveable_bounds;
// //   {
// //     std::vector<ConnectedLane> connected_lanes = {l1};
// //     builder->CreateSegmentConnections(1, &connected_lanes);
// //   }
// //   {
// //     std::vector<ConnectedLane> connected_lanes = {l2};
// //     builder->CreateSegmentConnections(2, &connected_lanes);
// //   }

// //   builder->CreateConnection(lane_bounds,
// //     driveable_bounds,
// //     ignition::rndf::UniqueId(2, 1, 3),
// //     ignition::rndf::UniqueId(1, 1, 2));

// //   auto road_geometry = builder->Build({"T"});
// //   EXPECT_NE(road_geometry, nullptr);

// //   EXPECT_EQ(road_geometry->num_junctions(), 5);
// //   {
// //     auto junction = road_geometry->junction(0);
// //     EXPECT_EQ(junction->num_segments(), 1);
// //     EXPECT_EQ(junction->id().id, "j:1-0");

// //     auto segment = junction->segment(0);
// //     EXPECT_EQ(segment->num_lanes(), 1);
// //     EXPECT_EQ(segment->id().id, "s:1-0");

// //     auto lane = segment->lane(0);
// //     EXPECT_EQ(lane->id().id, "l:1_1_1-1_1_2");
// //     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kStart), nullptr);
// //     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kStart)->size(), 1);
// //     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kFinish), nullptr);
// //     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kFinish)->size(), 2);
// //     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kStart), nullptr);
// //     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kStart)->size(), 0);
// //     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kFinish), nullptr);
// //     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kFinish)->size(), 1);

// //     auto *start_branch = lane->GetBranchPoint(api::LaneEnd::kStart);
// //     EXPECT_NE(start_branch, nullptr);
// //     EXPECT_NE(start_branch->GetASide(), nullptr);
// //     EXPECT_NE(start_branch->GetBSide(), nullptr);
// //     EXPECT_EQ(start_branch->GetASide()->size(), 1);
// //     EXPECT_EQ(start_branch->GetBSide()->size(), 0);
// //     EXPECT_EQ(start_branch->GetASide()->get(0).lane, lane);
// //     EXPECT_EQ(start_branch->GetASide()->get(0).end, api::LaneEnd::kStart);

// //     auto *end_branch = lane->GetBranchPoint(api::LaneEnd::kFinish);
// //     EXPECT_NE(end_branch, nullptr);
// //     EXPECT_NE(end_branch->GetASide(), nullptr);
// //     EXPECT_NE(end_branch->GetBSide(), nullptr);
// //     EXPECT_EQ(end_branch->GetASide()->size(), 2);
// //     EXPECT_EQ(end_branch->GetBSide()->size(), 1);
// //     EXPECT_EQ(end_branch->GetASide()->get(0).lane, lane);
// //     EXPECT_EQ(end_branch->GetASide()->get(0).end, api::LaneEnd::kFinish);
// //   }
// //   {
// //     auto junction = road_geometry->junction(1);
// //     EXPECT_EQ(junction->num_segments(), 1);
// //     EXPECT_EQ(junction->id().id, "j:1-1");

// //     auto segment = junction->segment(0);
// //     EXPECT_EQ(segment->num_lanes(), 1);
// //     EXPECT_EQ(segment->id().id, "s:1-1");

// //     auto lane = segment->lane(0);
// //     EXPECT_EQ(lane->id().id, "l:1_1_2-1_1_3");
// //     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kStart), nullptr);
// //     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kStart)->size(), 1);
// //     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kFinish), nullptr);
// //     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kFinish)->size(), 1);
// //     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kStart), nullptr);
// //     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kStart)->size(), 2);
// //     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kFinish), nullptr);
// //     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kFinish)->size(), 0);

// //     auto *start_branch = lane->GetBranchPoint(api::LaneEnd::kStart);
// //     EXPECT_NE(start_branch, nullptr);
// //     EXPECT_NE(start_branch->GetASide(), nullptr);
// //     EXPECT_NE(start_branch->GetBSide(), nullptr);
// //     EXPECT_EQ(start_branch->GetASide()->size(), 2);
// //     EXPECT_EQ(start_branch->GetBSide()->size(), 1);
// //     EXPECT_EQ(start_branch->GetBSide()->get(0).lane, lane);
// //     EXPECT_EQ(start_branch->GetBSide()->get(0).end, api::LaneEnd::kStart);

// //     auto *end_branch = lane->GetBranchPoint(api::LaneEnd::kFinish);
// //     EXPECT_NE(end_branch, nullptr);
// //     EXPECT_NE(end_branch->GetASide(), nullptr);
// //     EXPECT_NE(end_branch->GetBSide(), nullptr);
// //     EXPECT_EQ(end_branch->GetASide()->size(), 1);
// //     EXPECT_EQ(end_branch->GetBSide()->size(), 0);
// //     EXPECT_EQ(end_branch->GetASide()->get(0).lane, lane);
// //     EXPECT_EQ(end_branch->GetASide()->get(0).end, api::LaneEnd::kFinish);
// //   }
// //   {
// //     auto junction = road_geometry->junction(2);
// //     EXPECT_EQ(junction->num_segments(), 1);
// //     EXPECT_EQ(junction->id().id, "j:2-0");

// //     auto segment = junction->segment(0);
// //     EXPECT_EQ(segment->num_lanes(), 1);
// //     EXPECT_EQ(segment->id().id, "s:2-0");

// //     auto lane = segment->lane(0);
// //     EXPECT_EQ(lane->id().id, "l:2_1_1-2_1_2");
// //     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kStart), nullptr);
// //     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kStart)->size(), 1);
// //     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kFinish), nullptr);
// //     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kFinish)->size(), 1);
// //     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kStart), nullptr);
// //     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kStart)->size(), 0);
// //     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kFinish), nullptr);
// //     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kFinish)->size(), 1);

// //     auto *start_branch = lane->GetBranchPoint(api::LaneEnd::kStart);
// //     EXPECT_NE(start_branch, nullptr);
// //     EXPECT_NE(start_branch->GetASide(), nullptr);
// //     EXPECT_NE(start_branch->GetBSide(), nullptr);
// //     EXPECT_EQ(start_branch->GetASide()->size(), 1);
// //     EXPECT_EQ(start_branch->GetBSide()->size(), 0);
// //     EXPECT_EQ(start_branch->GetASide()->get(0).lane, lane);
// //     EXPECT_EQ(start_branch->GetASide()->get(0).end, api::LaneEnd::kStart);

// //     // TODO(@agalbachicar) End brnach
// //   }
// //   {
// //     auto junction = road_geometry->junction(3);
// //     EXPECT_EQ(junction->num_segments(), 1);
// //     EXPECT_EQ(junction->id().id, "j:2-1");

// //     auto segment = junction->segment(0);
// //     EXPECT_EQ(segment->num_lanes(), 1);
// //     EXPECT_EQ(segment->id().id, "s:2-1");

// //     auto lane = segment->lane(0);
// //     EXPECT_EQ(lane->id().id, "l:2_1_2-2_1_3");
// //     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kStart), nullptr);
// //     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kStart)->size(), 1);
// //     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kFinish), nullptr);
// //     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kFinish)->size(), 1);
// //     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kStart), nullptr);
// //     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kStart)->size(), 1);
// //     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kFinish), nullptr);
// //     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kFinish)->size(), 1);

// //     // TODO(@agalbachicar) Start branch

// //     auto *end_branch = lane->GetBranchPoint(api::LaneEnd::kFinish);
// //     EXPECT_NE(end_branch, nullptr);
// //     EXPECT_NE(end_branch->GetASide(), nullptr);
// //     EXPECT_NE(end_branch->GetBSide(), nullptr);
// //     EXPECT_EQ(end_branch->GetASide()->size(), 1);
// //     EXPECT_EQ(end_branch->GetBSide()->size(), 1);
// //     EXPECT_EQ(end_branch->GetASide()->get(0).lane, lane);
// //     EXPECT_EQ(end_branch->GetASide()->get(0).end, api::LaneEnd::kFinish);
// //   }

// //   {
// //     auto junction = road_geometry->junction(4);
// //     EXPECT_EQ(junction->num_segments(), 1);
// //     EXPECT_EQ(junction->id().id, "j:2_1_3-1_1_2");

// //     auto segment = junction->segment(0);
// //     EXPECT_EQ(segment->num_lanes(), 1);
// //     EXPECT_EQ(segment->id().id, "s:2_1_3-1_1_2");
// //     auto lane = segment->lane(0);
// //     EXPECT_EQ(lane->id().id, "l:2_1_3-1_1_2");
// //     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kStart), nullptr);
// //     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kStart)->size(), 1);
// //     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kFinish), nullptr);
// //     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kFinish)->size(), 2);
// //     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kStart), nullptr);
// //     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kStart)->size(), 1);
// //     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kFinish), nullptr);
// //     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kFinish)->size(), 1);

// //     auto *start_branch = lane->GetBranchPoint(api::LaneEnd::kStart);
// //     EXPECT_NE(start_branch, nullptr);
// //     EXPECT_NE(start_branch->GetASide(), nullptr);
// //     EXPECT_NE(start_branch->GetBSide(), nullptr);
// //     EXPECT_EQ(start_branch->GetASide()->size(), 1);
// //     EXPECT_EQ(start_branch->GetBSide()->size(), 1);
// //     EXPECT_EQ(start_branch->GetBSide()->get(0).lane, lane);
// //     EXPECT_EQ(start_branch->GetBSide()->get(0).end, api::LaneEnd::kStart);

// //     auto *end_branch = lane->GetBranchPoint(api::LaneEnd::kFinish);
// //     EXPECT_NE(end_branch, nullptr);
// //     EXPECT_NE(end_branch->GetASide(), nullptr);
// //     EXPECT_NE(end_branch->GetBSide(), nullptr);
// //     EXPECT_EQ(end_branch->GetASide()->size(), 2);
// //     EXPECT_EQ(end_branch->GetBSide()->size(), 1);
// //     EXPECT_EQ(end_branch->GetASide()->get(1).lane, lane);
// //     EXPECT_EQ(end_branch->GetASide()->get(1).end, api::LaneEnd::kFinish);
// //   }
// // }

// // /*
// // // We create a T connection and check its creation and correct invariants from
// // // the road geometry. It will look like:

// // //                                    2.1.1
// // //                                    |
// // //                                    v
// // //                                    2.1.2
// // //                                    |
// // //                                    v
// // //                                    2.1.3
// // //                                    \
// // //                                     v
// // // 1.1.1 ----->1.1.2------>1.1.3------>1.1.4------>1.1.5

// // // This simple example is useful for checking the how the waypoint grouping is
// // // working
// // GTEST_TEST(RNDFBuilder, BuildTToCheckWaypointGrouping) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-10/ 2., 10. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   {
// //     std::vector<DirectedWaypoint> points = {
// //       DirectedWaypoint(
// //         ignition::rndf::UniqueId(1, 1, 1),
// //         ignition::math::Vector3d(0., 0., 0.)),
// //       DirectedWaypoint(
// //         ignition::rndf::UniqueId(1, 1, 2),
// //         ignition::math::Vector3d(5., 0., 0.)),
// //       DirectedWaypoint(
// //         ignition::rndf::UniqueId(1, 1, 3),
// //         ignition::math::Vector3d(5., 0., 0.)),
// //       DirectedWaypoint(
// //         ignition::rndf::UniqueId(1, 1, 4),
// //         ignition::math::Vector3d(20., 0., 0.),
// //         true,
// //         false),
// //       DirectedWaypoint(
// //         ignition::rndf::UniqueId(1, 1, 5),
// //         ignition::math::Vector3d(25., 0., 0.)),
// //     };
// //     builder->CreateLaneConnections(1, 1, points, lane_bounds, driveable_bounds);
// //   }
// //   {
// //     std::vector<DirectedWaypoint> points = {
// //       DirectedWaypoint(
// //         ignition::rndf::UniqueId(2, 1, 1),
// //         ignition::math::Vector3d(5., 20., 0.)),
// //       DirectedWaypoint(
// //         ignition::rndf::UniqueId(2, 1, 2),
// //         ignition::math::Vector3d(5., 12.5, 0.)),
// //       DirectedWaypoint(
// //         ignition::rndf::UniqueId(2, 1, 3),
// //         ignition::math::Vector3d(5., 5., 0.),
// //         false,
// //         true)
// //     };
// //     builder->CreateLaneConnections(2, 1, points, lane_bounds, driveable_bounds);
// //   }
// //   builder->CreateConnection(lane_bounds,
// //     driveable_bounds,
// //     ignition::rndf::UniqueId(2, 1, 3),
// //     ignition::rndf::UniqueId(1, 1, 4));

// //   auto road_geometry = builder->Build({"L"});
// //   EXPECT_NE(road_geometry, nullptr);

// //   EXPECT_EQ(road_geometry->num_junctions(), 4);
// //   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1_1_1-1_1_4");
// //   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1_1_4-1_1_5");
// //   EXPECT_EQ(road_geometry->junction(2)->id().id, "j:2_1_1-2_1_3");
// //   EXPECT_EQ(road_geometry->junction(3)->id().id, "j:2_1_3-1_1_4");
// // }
// // */

// // // This test checks that the lanes are set inside the segment in the correct
// // // order. First we build a road geometry with a simple segment with two lanes
// // // which are OK and then we build the same but the points are swapped. The
// // // expected final result is the correct order of the lanes inside the segment.
// // GTEST_TEST(RNDFBuilder, LaneRightToLeft) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   {
// //     ConnectedLane l1, l2;
// //     l1.waypoints.push_back(DirectedWaypoint(
// //       ignition::rndf::UniqueId(1, 1, 1),
// //       ignition::math::Vector3d(0.0, 0.0, 0.0)));
// //     l1.waypoints.push_back(DirectedWaypoint(
// //       ignition::rndf::UniqueId(1, 1, 2),
// //       ignition::math::Vector3d(10.0, 0.0, 0.0)));

// //     l2.waypoints.push_back(DirectedWaypoint(
// //       ignition::rndf::UniqueId(1, 2, 1),
// //       ignition::math::Vector3d(0.0, 5.0, 0.0)));
// //     l2.waypoints.push_back(DirectedWaypoint(
// //       ignition::rndf::UniqueId(1, 2, 2),
// //       ignition::math::Vector3d(10.0, 5.0, 0.0)));


// //     l1.lane_bounds = lane_bounds;
// //     l2.lane_bounds = lane_bounds;
// //     l1.driveable_bounds = driveable_bounds;
// //     l2.driveable_bounds = driveable_bounds;
// //     std::vector<ConnectedLane> connected_lanes = {l1, l2};

// //     builder->CreateSegmentConnections(1, &connected_lanes);
// //     auto road_geometry = builder->Build({"LaneRightToLeft"});
// //     EXPECT_NE(road_geometry, nullptr);

// //     EXPECT_EQ(road_geometry->num_junctions(), 1);
// //     EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
// //     EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
// //     EXPECT_EQ(
// //       road_geometry->junction(0)->segment(0)->lane(0)->id().id,
// //       "l:1_1_1-1_1_2");
// //     EXPECT_EQ(
// //       road_geometry->junction(0)->segment(0)->lane(1)->id().id,
// //       "l:1_2_1-1_2_2");
// //   }

// //   // Create a new builder
// //   builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);
// //   {
// //     ConnectedLane l1, l2;
// //     l1.waypoints.push_back(DirectedWaypoint(
// //       ignition::rndf::UniqueId(1, 1, 1),
// //       ignition::math::Vector3d(0.0, 5.0, 0.0)));
// //     l1.waypoints.push_back(DirectedWaypoint(
// //       ignition::rndf::UniqueId(1, 1, 2),
// //       ignition::math::Vector3d(10.0, 5.0, 0.0)));

// //     l2.waypoints.push_back(DirectedWaypoint(
// //       ignition::rndf::UniqueId(1, 2, 1),
// //       ignition::math::Vector3d(0.0, 0.0, 0.0)));
// //     l2.waypoints.push_back(DirectedWaypoint(
// //       ignition::rndf::UniqueId(1, 2, 2),
// //       ignition::math::Vector3d(10.0, 0.0, 0.0)));


// //     l1.lane_bounds = lane_bounds;
// //     l2.lane_bounds = lane_bounds;
// //     l1.driveable_bounds = driveable_bounds;
// //     l2.driveable_bounds = driveable_bounds;
// //     std::vector<ConnectedLane> connected_lanes = {l1, l2};

// //     builder->CreateSegmentConnections(1, &connected_lanes);
// //     auto road_geometry = builder->Build({"LaneLeftToRight"});
// //     EXPECT_NE(road_geometry, nullptr);

// //     EXPECT_EQ(road_geometry->num_junctions(), 1);
// //     EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
// //     EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
// //     EXPECT_EQ(
// //       road_geometry->junction(0)->segment(0)->lane(0)->id().id,
// //       "l:1_2_1-1_2_2");
// //     EXPECT_EQ(
// //       road_geometry->junction(0)->segment(0)->lane(1)->id().id,
// //       "l:1_1_1-1_1_2");
// //   }
// // }


// // //        * --> *
// // //  * --> * --> *
// // GTEST_TEST(RNDFBuilder, LaneStartsAfter) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2;
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(5.0, 0.0, 0.0)));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(10.0, 0.0, 0.0)));

// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 1),
// //     ignition::math::Vector3d(0.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 2),
// //     ignition::math::Vector3d(5.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 3),
// //     ignition::math::Vector3d(10.0, 5.0, 0.0)));

// //   l1.lane_bounds = lane_bounds;
// //   l2.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   l2.driveable_bounds = driveable_bounds;
// //   std::vector<ConnectedLane> connected_lanes = {l1, l2};

// //   builder->CreateSegmentConnections(1, &connected_lanes);

// //   auto road_geometry = builder->Build({"LaneStartsAfter"});
// //   EXPECT_NE(road_geometry, nullptr);

// //   EXPECT_EQ(road_geometry->num_junctions(), 2);
// //   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1-1");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id, "s:1-1");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 2);

// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_2_1-1_2_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(1)->id().id, "l:1_2_2-1_2_3");
// // }

// // //              * --> *
// // //  * --> * --> * --> *
// // GTEST_TEST(RNDFBuilder, LaneStartsAfter2) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2;

// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(5.0, 0.0, 0.0)));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(10.0, 0.0, 0.0)));

// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 1),
// //     ignition::math::Vector3d(0.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 2),
// //     ignition::math::Vector3d(2.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 3),
// //     ignition::math::Vector3d(5.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 4),
// //     ignition::math::Vector3d(10.0, 5.0, 0.0)));

// //   l1.lane_bounds = lane_bounds;
// //   l2.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   l2.driveable_bounds = driveable_bounds;
// //   std::vector<ConnectedLane> connected_lanes = {l1, l2};

// //   builder->CreateSegmentConnections(1, &connected_lanes);

// //   auto road_geometry = builder->Build({"LaneStartsAfter2"});
// //   EXPECT_NE(road_geometry, nullptr);

// //   EXPECT_EQ(road_geometry->num_junctions(), 3);
// //   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1-1");
// //   EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1-2");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id, "s:1-1");
// //   EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().id, "s:1-2");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 1);
// //   EXPECT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 2);

// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_2_1-1_2_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(0)->id().id, "l:1_2_2-1_2_3");
// //   EXPECT_EQ(
// //     road_geometry->junction(2)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(2)->segment(0)->lane(1)->id().id, "l:1_2_3-1_2_4");
// // }

// // //  * --> *
// // //  * --> * --> *
// // GTEST_TEST(RNDFBuilder, LaneEndsBefore) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2;

// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(0.0, 0.0, 0.0)));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(5.0, 0.0, 0.0)));

// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 1),
// //     ignition::math::Vector3d(0.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 2),
// //     ignition::math::Vector3d(5.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 3),
// //     ignition::math::Vector3d(10.0, 5.0, 0.0)));

// //   l1.lane_bounds = lane_bounds;
// //   l2.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   l2.driveable_bounds = driveable_bounds;
// //   std::vector<ConnectedLane> connected_lanes = {l1, l2};

// //   builder->CreateSegmentConnections(1, &connected_lanes);

// //   auto road_geometry = builder->Build({"LaneEndsBefore"});
// //   EXPECT_NE(road_geometry, nullptr);

// //   EXPECT_EQ(road_geometry->num_junctions(), 2);
// //   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1-1");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id, "s:1-1");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 2);
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 1);

// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(1)->id().id, "l:1_2_1-1_2_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(0)->id().id, "l:1_2_2-1_2_3");
// // }

// // //  * --> *
// // //  * --> * --> * --> *
// // GTEST_TEST(RNDFBuilder, LaneEndsBefore2) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2;

// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(0.0, 0.0, 0.0)));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(5.0, 0.0, 0.0)));

// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 1),
// //     ignition::math::Vector3d(0.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 2),
// //     ignition::math::Vector3d(5.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 3),
// //     ignition::math::Vector3d(10.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 4),
// //     ignition::math::Vector3d(15.0, 5.0, 0.0)));

// //   l1.lane_bounds = lane_bounds;
// //   l2.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   l2.driveable_bounds = driveable_bounds;
// //   std::vector<ConnectedLane> connected_lanes = {l1, l2};

// //   builder->CreateSegmentConnections(1, &connected_lanes);

// //   auto road_geometry = builder->Build({"LaneEndsBefore2"});
// //   EXPECT_NE(road_geometry, nullptr);

// //   EXPECT_EQ(road_geometry->num_junctions(), 3);
// //   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1-1");
// //   EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1-2");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id, "s:1-1");
// //   EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().id, "s:1-2");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 2);
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 1);
// //   EXPECT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 1);

// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(1)->id().id, "l:1_2_1-1_2_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(0)->id().id, "l:1_2_2-1_2_3");
// //   EXPECT_EQ(
// //     road_geometry->junction(2)->segment(0)->lane(0)->id().id, "l:1_2_3-1_2_4");
// // }

// // //        * --> *
// // //  * --> * --> * --> *
// // GTEST_TEST(RNDFBuilder, LaneStartsAfterAndEndsBefore) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2;

// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(5.0, 0.0, 0.0)));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(10.0, 0.0, 0.0)));

// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 1),
// //     ignition::math::Vector3d(0.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 2),
// //     ignition::math::Vector3d(5.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 3),
// //     ignition::math::Vector3d(10.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 4),
// //     ignition::math::Vector3d(15.0, 5.0, 0.0)));

// //   l1.lane_bounds = lane_bounds;
// //   l2.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   l2.driveable_bounds = driveable_bounds;
// //   std::vector<ConnectedLane> connected_lanes = {l1, l2};

// //   builder->CreateSegmentConnections(1, &connected_lanes);

// //   auto road_geometry = builder->Build({"LaneStartsAfterAndEndsBefore"});
// //   EXPECT_NE(road_geometry, nullptr);

// //   EXPECT_EQ(road_geometry->num_junctions(), 3);
// //   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1-1");
// //   EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1-2");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id, "s:1-1");
// //   EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().id, "s:1-2");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 2);
// //   EXPECT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 1);

// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_2_1-1_2_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(1)->id().id, "l:1_2_2-1_2_3");
// //   EXPECT_EQ(
// //     road_geometry->junction(2)->segment(0)->lane(0)->id().id, "l:1_2_3-1_2_4");
// // }

// // //     * --> *
// // // * ------> *
// // GTEST_TEST(RNDFBuilder, LaneStartsAfterButNotInPhase) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2;

// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(7.5, 0.0, 0.0)));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(10.0, 0.0, 0.0)));

// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 1),
// //     ignition::math::Vector3d(0.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 2),
// //     ignition::math::Vector3d(10.0, 5.0, 0.0)));

// //   l1.lane_bounds = lane_bounds;
// //   l2.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   l2.driveable_bounds = driveable_bounds;
// //   std::vector<ConnectedLane> connected_lanes = {l1, l2};

// //   builder->CreateSegmentConnections(1, &connected_lanes);

// //   auto road_geometry = builder->Build({"LaneStartsAfterButNotInPhase"});
// //   EXPECT_NE(road_geometry, nullptr);

// //   EXPECT_EQ(road_geometry->num_junctions(), 2);
// //   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1-1");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id, "s:1-1");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 2);

// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_2_1-1_2_3");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(1)->id().id, "l:1_2_3-1_2_2");
// //   {
// //   auto geo_position = road_geometry->junction(0)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(0)->segment(0)->lane(0)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(7.5, 5.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(1)->segment(0)->lane(1)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(7.5, 5.0, 0.0),
// //     kLinearTolerance);
// //   }
// // }

// // //         * --> *
// // // * --> * ----> *
// // GTEST_TEST(RNDFBuilder, LaneStartsAfterButNotInPhase2) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2;

// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(7.5, 0.0, 0.0)));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(10.0, 0.0, 0.0)));

// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 1),
// //     ignition::math::Vector3d(0.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 2),
// //     ignition::math::Vector3d(5.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 3),
// //     ignition::math::Vector3d(10.0, 5.0, 0.0)));

// //   l1.lane_bounds = lane_bounds;
// //   l2.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   l2.driveable_bounds = driveable_bounds;
// //   std::vector<ConnectedLane> connected_lanes = {l1, l2};

// //   builder->CreateSegmentConnections(1, &connected_lanes);

// //   auto road_geometry = builder->Build({"LaneStartsAfterButNotInPhase2"});
// //   EXPECT_NE(road_geometry, nullptr);

// //   EXPECT_EQ(road_geometry->num_junctions(), 3);
// //   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1-1");
// //   EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1-2");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id, "s:1-1");
// //   EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().id, "s:1-2");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 1);
// //   EXPECT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 2);

// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_2_1-1_2_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(0)->id().id, "l:1_2_2-1_2_4");
// //   EXPECT_EQ(
// //     road_geometry->junction(2)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(2)->segment(0)->lane(1)->id().id, "l:1_2_4-1_2_3");

// //   {
// //   auto geo_position = road_geometry->junction(1)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(1)->segment(0)->lane(0)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(7.5, 5.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(2)->segment(0)->lane(1)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(7.5, 5.0, 0.0),
// //     kLinearTolerance);
// //   }
// // }

// // //     * --> *
// // // * ----------> *
// // GTEST_TEST(RNDFBuilder, LaneStartsAfterEndsAfterNotInPhase) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2;

// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(2.5, 0.0, 0.0)));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(7.5, 0.0, 0.0)));

// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 1),
// //     ignition::math::Vector3d(0.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 2),
// //     ignition::math::Vector3d(10.0, 5.0, 0.0)));

// //   l1.lane_bounds = lane_bounds;
// //   l2.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   l2.driveable_bounds = driveable_bounds;
// //   std::vector<ConnectedLane> connected_lanes = {l1, l2};

// //   builder->CreateSegmentConnections(1, &connected_lanes);

// //   auto road_geometry = builder->Build({"LaneStartsAfterEndsAfterNotInPhase"});
// //   EXPECT_NE(road_geometry, nullptr);

// //   EXPECT_EQ(road_geometry->num_junctions(), 3);
// //   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1-1");
// //   EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1-2");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id, "s:1-1");
// //   EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().id, "s:1-2");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 2);
// //   EXPECT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 1);

// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_2_1-1_2_3");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(1)->id().id, "l:1_2_3-1_2_4");
// //   EXPECT_EQ(
// //     road_geometry->junction(2)->segment(0)->lane(0)->id().id, "l:1_2_4-1_2_2");

// //   {
// //   auto geo_position = road_geometry->junction(0)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(0)->segment(0)->lane(0)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(2.5, 5.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(1)->segment(0)->lane(1)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(2.5, 5.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(1)->segment(0)->lane(1)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(1)->segment(0)->lane(1)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(7.5, 5.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(2)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(7.5, 5.0, 0.0),
// //     kLinearTolerance);
// //   }
// // }

// // //     * ----> *
// // // * ----> * -----> *
// // GTEST_TEST(RNDFBuilder, TotallyDephasedLanes) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2;

// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(2.5, 0.0, 0.0)));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(7.5, 0.0, 0.0)));

// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 1),
// //     ignition::math::Vector3d(0.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 2),
// //     ignition::math::Vector3d(5.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 3),
// //     ignition::math::Vector3d(10.0, 5.0, 0.0)));

// //   l1.lane_bounds = lane_bounds;
// //   l2.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   l2.driveable_bounds = driveable_bounds;
// //   std::vector<ConnectedLane> connected_lanes = {l1, l2};

// //   builder->CreateSegmentConnections(1, &connected_lanes);

// //   auto road_geometry = builder->Build({"TotallyDephasedLanes"});
// //   EXPECT_NE(road_geometry, nullptr);

// //   EXPECT_EQ(road_geometry->num_junctions(), 4);
// //   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1-1");
// //   EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1-2");
// //   EXPECT_EQ(road_geometry->junction(3)->id().id, "j:1-3");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id, "s:1-1");
// //   EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().id, "s:1-2");
// //   EXPECT_EQ(road_geometry->junction(3)->segment(0)->id().id, "s:1-3");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 2);
// //   EXPECT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 2);
// //   EXPECT_EQ(road_geometry->junction(3)->segment(0)->num_lanes(), 1);

// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_2_1-1_2_4");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_4");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(1)->id().id, "l:1_2_4-1_2_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(2)->segment(0)->lane(0)->id().id, "l:1_1_4-1_1_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(2)->segment(0)->lane(1)->id().id, "l:1_2_2-1_2_5");
// //   EXPECT_EQ(
// //     road_geometry->junction(3)->segment(0)->lane(0)->id().id, "l:1_2_5-1_2_3");

// //   {
// //   auto geo_position = road_geometry->junction(0)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(0)->segment(0)->lane(0)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(2.5, 5.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(1)->segment(0)->lane(1)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(2.5, 5.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(1)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(1)->segment(0)->lane(0)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 0.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(2)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 0.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(2)->segment(0)->lane(1)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(2)->segment(0)->lane(1)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(7.5, 5.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(3)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(7.5, 5.0, 0.0),
// //     kLinearTolerance);
// //   }
// // }

// // // * ---------> *
// // //     * -----> *
// // //         * -> *
// // GTEST_TEST(RNDFBuilder, ThreeLaneStartWithPhaseEndCoherent) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2, l3;

// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(0.0, 0.0, 0.0)));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(10.0, 0.0, 0.0)));

// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 1),
// //     ignition::math::Vector3d(2.5, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 2),
// //     ignition::math::Vector3d(10.0, 5.0, 0.0)));

// //   l3.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 3, 1),
// //     ignition::math::Vector3d(5.0, 10.0, 0.0)));
// //   l3.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 3, 2),
// //     ignition::math::Vector3d(10.0, 10.0, 0.0)));

// //   l1.lane_bounds = lane_bounds;
// //   l2.lane_bounds = lane_bounds;
// //   l3.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   l2.driveable_bounds = driveable_bounds;
// //   l3.driveable_bounds = driveable_bounds;
// //   std::vector<ConnectedLane> connected_lanes = {l1, l2, l3};

// //   builder->CreateSegmentConnections(1, &connected_lanes);

// //   auto road_geometry = builder->Build({"ThreeLaneStartWithPhaseEndCoherent"});
// //   EXPECT_NE(road_geometry, nullptr);

// //   EXPECT_EQ(road_geometry->num_junctions(), 3);
// //   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1-1");
// //   EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1-2");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id, "s:1-1");
// //   EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().id, "s:1-2");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 2);
// //   EXPECT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 3);

// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_3");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(0)->id().id, "l:1_1_3-1_1_4");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(1)->id().id, "l:1_2_1-1_2_4");
// //   EXPECT_EQ(
// //     road_geometry->junction(2)->segment(0)->lane(0)->id().id, "l:1_1_4-1_1_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(2)->segment(0)->lane(1)->id().id, "l:1_2_4-1_2_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(2)->segment(0)->lane(2)->id().id, "l:1_3_1-1_3_2");

// //   {
// //   auto geo_position = road_geometry->junction(0)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(0)->segment(0)->lane(0)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(2.5, 0.0, 0.0),
// //     kLinearTolerance);
// //   }

// //   {
// //   auto geo_position = road_geometry->junction(1)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(2.5, 0.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(1)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(1)->segment(0)->lane(0)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 0.0, 0.0),
// //     kLinearTolerance);
// //   }

// //   {
// //   auto geo_position = road_geometry->junction(2)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 0.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(1)->segment(0)->lane(1)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(1)->segment(0)->lane(1)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 5.0, 0.0),
// //     kLinearTolerance);
// //   }

// //   {
// //   auto geo_position = road_geometry->junction(2)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 0.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(2)->segment(0)->lane(1)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 5.0, 0.0),
// //     kLinearTolerance);
// //   }
// // }

// // //   * ------> *
// // //     * -------> *
// // // * --------------> *
// // GTEST_TEST(RNDFBuilder, ThreeLaneStartAndEndWithPhases) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2, l3;

// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(2.5, 0.0, 0.0)));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(10.0, 0.0, 0.0)));

// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 1),
// //     ignition::math::Vector3d(5.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 2),
// //     ignition::math::Vector3d(12.5, 5.0, 0.0)));

// //   l3.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 3, 1),
// //     ignition::math::Vector3d(0.0, 10.0, 0.0)));
// //   l3.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 3, 2),
// //     ignition::math::Vector3d(15.0, 10.0, 0.0)));

// //   l1.lane_bounds = lane_bounds;
// //   l2.lane_bounds = lane_bounds;
// //   l3.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   l2.driveable_bounds = driveable_bounds;
// //   l3.driveable_bounds = driveable_bounds;
// //   std::vector<ConnectedLane> connected_lanes = {l1, l2, l3};

// //   builder->CreateSegmentConnections(1, &connected_lanes);

// //   auto road_geometry = builder->Build({"ThreeLaneStartAndEndWithPhases"});
// //   EXPECT_NE(road_geometry, nullptr);

// //   EXPECT_EQ(road_geometry->num_junctions(), 5);
// //   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1-1");
// //   EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1-2");
// //   EXPECT_EQ(road_geometry->junction(3)->id().id, "j:1-3");
// //   EXPECT_EQ(road_geometry->junction(4)->id().id, "j:1-4");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id, "s:1-1");
// //   EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().id, "s:1-2");
// //   EXPECT_EQ(road_geometry->junction(3)->segment(0)->id().id, "s:1-3");
// //   EXPECT_EQ(road_geometry->junction(4)->segment(0)->id().id, "s:1-4");

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
// //   EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 2);
// //   EXPECT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 3);
// //   EXPECT_EQ(road_geometry->junction(3)->segment(0)->num_lanes(), 2);
// //   EXPECT_EQ(road_geometry->junction(4)->segment(0)->num_lanes(), 1);

// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_3_1-1_3_3");

// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_4");
// //   EXPECT_EQ(
// //     road_geometry->junction(1)->segment(0)->lane(1)->id().id, "l:1_3_3-1_3_4");

// //   EXPECT_EQ(
// //     road_geometry->junction(2)->segment(0)->lane(0)->id().id, "l:1_1_4-1_1_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(2)->segment(0)->lane(1)->id().id, "l:1_2_1-1_2_5");
// //   EXPECT_EQ(
// //     road_geometry->junction(2)->segment(0)->lane(2)->id().id, "l:1_3_4-1_3_5");

// //   EXPECT_EQ(
// //     road_geometry->junction(3)->segment(0)->lane(0)->id().id, "l:1_2_5-1_2_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(3)->segment(0)->lane(1)->id().id, "l:1_3_5-1_3_6");

// //   EXPECT_EQ(
// //     road_geometry->junction(4)->segment(0)->lane(0)->id().id, "l:1_3_6-1_3_2");

// //   // Segment 1-0
// //   {
// //   auto geo_position = road_geometry->junction(0)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(0)->segment(0)->lane(0)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(2.5, 10.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   // Segment 1-1
// //   {
// //   auto geo_position = road_geometry->junction(1)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(1)->segment(0)->lane(0)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 0.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(1)->segment(0)->lane(1)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(2.5, 10.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(1)->segment(0)->lane(1)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(1)->segment(0)->lane(1)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5.0, 10.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   // Segment 1-2
// //   {
// //   auto geo_position = road_geometry->junction(2)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5, 0.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(2)->segment(0)->lane(2)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(5, 10.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(2)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(2)->segment(0)->lane(0)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10, 0.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(2)->segment(0)->lane(1)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(2)->segment(0)->lane(1)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10, 5.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(2)->segment(0)->lane(2)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(2)->segment(0)->lane(2)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10, 10.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   // Segment 1-3
// //   {
// //   auto geo_position = road_geometry->junction(3)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 5.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(3)->segment(0)->lane(1)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 10.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   {
// //   auto geo_position = road_geometry->junction(3)->segment(0)->lane(1)->
// //     ToGeoPosition(api::LanePosition(
// //       road_geometry->junction(3)->segment(0)->lane(1)->length(), 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(12.5, 10.0, 0.0),
// //     kLinearTolerance);
// //   }
// //   // Segment 1-4
// //   {
// //   auto geo_position = road_geometry->junction(4)->segment(0)->lane(0)->
// //     ToGeoPosition(api::LanePosition(0., 0., 0.));
// //   EXPECT_GEO_NEAR(geo_position, api::GeoPosition(12.5, 10.0, 0.0),
// //     kLinearTolerance);
// //   }
// // }


// // //  * <-- *
// // //  * --> *
// // GTEST_TEST(RNDFBuilder, InvertedLanesCaseA) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2;
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(0.0, 0.0, 0.0)));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(10.0, 0.0, 0.0)));

// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 1),
// //     ignition::math::Vector3d(10.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 2),
// //     ignition::math::Vector3d(0.0, 5.0, 0.0)));

// //   l1.lane_bounds = lane_bounds;
// //   l2.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   l2.driveable_bounds = driveable_bounds;
// //   std::vector<ConnectedLane> connected_lanes = {l1, l2};

// //   builder->CreateSegmentConnections(1, &connected_lanes);

// //   auto road_geometry = builder->Build({"InvertedLanesCaseA"});
// //   EXPECT_NE(road_geometry, nullptr);

// //   EXPECT_EQ(road_geometry->num_junctions(), 1);
// //   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
// //   EXPECT_EQ(road_geometry->junction(0)->num_segments(), 1);

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 2);
// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(1)->id().id, "l:1_2_1-1_2_2");
// // }

// // //  * --> *
// // //  * <-- *
// // GTEST_TEST(RNDFBuilder, InvertedLanesCaseARotated) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2;
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(0.0, 0.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(10.0, 0.0, 0.0)));

// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 1),
// //     ignition::math::Vector3d(10.0, 5.0, 0.0)));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 2),
// //     ignition::math::Vector3d(0.0, 5.0, 0.0)));

// //   l1.lane_bounds = lane_bounds;
// //   l2.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   l2.driveable_bounds = driveable_bounds;
// //   std::vector<ConnectedLane> connected_lanes = {l1, l2};

// //   builder->CreateSegmentConnections(1, &connected_lanes);

// //   auto road_geometry = builder->Build({"InvertedLanesCaseA"});
// //   EXPECT_NE(road_geometry, nullptr);

// //   EXPECT_EQ(road_geometry->num_junctions(), 1);
// //   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
// //   EXPECT_EQ(road_geometry->junction(0)->num_segments(), 1);

// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
// //   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 2);
// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_2_1-1_2_2");
// //   EXPECT_EQ(
// //     road_geometry->junction(0)->segment(0)->lane(1)->id().id, "l:1_1_1-1_1_2");
// // }

// // //  * <-- *
// // //  * --> *
// // GTEST_TEST(RNDFBuilder, InvertedLanesCaseB) {
// //   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
// //   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// //   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
// //     lane_bounds,
// //     driveable_bounds,
// //     0.01,
// //     0.01 * M_PI);

// //   ConnectedLane l1, l2;
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 1),
// //     ignition::math::Vector3d(0.0, 0.0, 0.0)));
// //   l1.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 1, 2),
// //     ignition::math::Vector3d(10.0, 0.0, 0.0)));

// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 1),
// //     ignition::math::Vector3d(5.0, 5.0, 0.0)));
// //   l2.waypoints.push_back(DirectedWaypoint(
// //     ignition::rndf::UniqueId(1, 2, 2),
// //     ignition::math::Vector3d(0.0, 5.0, 0.0)));

// //   l1.lane_bounds = lane_bounds;
// //   l2.lane_bounds = lane_bounds;
// //   l1.driveable_bounds = driveable_bounds;
// //   l2.driveable_bounds = driveable_bounds;
// //   std::vector<ConnectedLane> connected_lanes = {l1, l2};

// //   builder->CreateSegmentConnections(1, &connected_lanes);

// //   auto road_geometry = builder->Build({"InvertedLanesCaseA"});
// //   EXPECT_NE(road_geometry, nullptr);

// //   std::cout << "Num junctions: " << road_geometry->num_junctions() << std::endl;
// //   for (int i  = 0; i < road_geometry->num_junctions(); i++) {
// //     std::cout << "- " << road_geometry->junction(i)->id().id << std::endl;
// //   }
// // }


// GTEST_TEST(RNDFBuilder, NA_Lane) {
//   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
//   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// =======
// // Some constructor and operator overloading assertions on DirectedWaypoint
// // class
// GTEST_TEST(RNDFBuilder, DirectedWaypointClass) {
//   std::unique_ptr<DirectedWaypoint> directed_waypoint;

//   directed_waypoint = std::make_unique<DirectedWaypoint>();
//   EXPECT_EQ(directed_waypoint->Id(), ignition::rndf::UniqueId());
//   EXPECT_EQ(directed_waypoint->Position(), ignition::math::Vector3d::Zero);
//   EXPECT_EQ(directed_waypoint->Tangent(), ignition::math::Vector3d::Zero);

//   {
//     std::unique_ptr<DirectedWaypoint> copy_directed_waypoint =
//       std::make_unique<DirectedWaypoint>(*directed_waypoint);
//     EXPECT_EQ(copy_directed_waypoint->Id(),
//       ignition::rndf::UniqueId());
//     EXPECT_EQ(copy_directed_waypoint->Position(),
//       ignition::math::Vector3d::Zero);
//     EXPECT_EQ(copy_directed_waypoint->Tangent(),
//       ignition::math::Vector3d::Zero);
//   }

//   directed_waypoint = std::make_unique<DirectedWaypoint>(
//     ignition::rndf::UniqueId(1, 2, 3),
//     ignition::math::Vector3d(1, 2, 3),
//     false,
//     true,
//     ignition::math::Vector3d(4, 5, 6));
//   EXPECT_EQ(directed_waypoint->Id(), ignition::rndf::UniqueId(1, 2, 3));
//   EXPECT_EQ(directed_waypoint->Position(), ignition::math::Vector3d(1, 2, 3));
//   EXPECT_EQ(directed_waypoint->Tangent(), ignition::math::Vector3d(4, 5, 6));
//   EXPECT_EQ(directed_waypoint->is_entry(), false);
//   EXPECT_EQ(directed_waypoint->is_exit(), true);

//   {
//     DirectedWaypoint copy_directed_waypoint = *directed_waypoint;
//     EXPECT_EQ(copy_directed_waypoint.Id(), ignition::rndf::UniqueId(1, 2, 3));
//     EXPECT_EQ(copy_directed_waypoint.Position(),
//       ignition::math::Vector3d(1, 2, 3));
//     EXPECT_EQ(copy_directed_waypoint.Tangent(),
//       ignition::math::Vector3d(4, 5, 6));
//     EXPECT_EQ(copy_directed_waypoint.is_entry(), false);
//     EXPECT_EQ(copy_directed_waypoint.is_exit(), true);
//   }
// }

// // Constructor assertions and checks over the quantity of points for the
// // Connetion class
// GTEST_TEST(RNDFBuilder, ConnectionClass) {
//   const double width = 5.;
//   std::vector<DirectedWaypoint> directed_waypoints;
//   directed_waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 1),
//     ignition::math::Vector3d(0.0, 0.0, 0.0),
//     false, false,
//     ignition::math::Vector3d(10.0, 0.0, 0.0)));
//   directed_waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 2),
//     ignition::math::Vector3d(20.0, 0.0, 0.0),
//     false, false,
//     ignition::math::Vector3d(10.0, 0.0, 0.0)));

//   std::unique_ptr<Connection> connection;
//   connection = std::make_unique<Connection>("1_1_1-1_1_2",
//     directed_waypoints,
//     width);

//   EXPECT_EQ(connection->type(), Connection::kSpline);
//   EXPECT_EQ(connection->id(), "1_1_1-1_1_2");
//   EXPECT_EQ(connection->start().Id(), ignition::rndf::UniqueId(1, 1, 1));
//   EXPECT_EQ(connection->end().Id(), ignition::rndf::UniqueId(1, 1, 2));
//   EXPECT_EQ(connection->waypoints().size(), 2);

//   directed_waypoints.clear();
//   EXPECT_THROW(
//     std::make_unique<Connection>(std::string("1_1_1-1_1_2"),
//       directed_waypoints,
//       width),
//     std::runtime_error);
// }

// // Assertions on the checks when creating a lane using the builder.
// // When we get the road_geometry, we check the created values and names.
// GTEST_TEST(RNDFBuilder, BuilderLaneConnections) {
//   const double width = 5.;
//   const api::RBounds lane_bounds(-width / 2., width / 2.);
//   const api::RBounds driveable_bounds(-width / 2., width / 2.);
//   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
//     0.01,
//     0.01 * M_PI);

//   ConnectedLane l1, l2;
//   l1.width = width;
//   std::vector<ConnectedLane> lane_vector;
//   // Check for nullptr validation
//   EXPECT_THROW(
//     builder->CreateSegmentConnections(1, nullptr), std::runtime_error);
//   // Check for lanes vector validation
//   EXPECT_THROW(
//     builder->CreateSegmentConnections(1, &lane_vector), std::runtime_error);
//   // Check for lane waypoint validation
//   lane_vector.push_back(l1);
//   EXPECT_THROW(
//     builder->CreateSegmentConnections(1, &lane_vector), std::runtime_error);
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 1),
//     ignition::math::Vector3d(0., 0., 0.)));
//   lane_vector.clear();
//   lane_vector.push_back(l1);
//   EXPECT_THROW(
//     builder->CreateSegmentConnections(1, &lane_vector), std::runtime_error);
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 2),
//     ignition::math::Vector3d(10., 0., 0.)));
//   lane_vector.clear();
//   lane_vector.push_back(l1);
//   EXPECT_NO_THROW(
//     builder->CreateSegmentConnections(1, &lane_vector));
//   lane_vector.clear();
//   lane_vector.push_back(l1);
//   lane_vector.push_back(l2);
//   EXPECT_THROW(
//     builder->CreateSegmentConnections(2, &lane_vector), std::runtime_error);

//   auto road_geometry = builder->Build({"One-Lane"});

//   EXPECT_EQ(road_geometry->CheckInvariants().size(), 0);

//   EXPECT_EQ(road_geometry->num_junctions(), 1);

//   auto junction = road_geometry->junction(0);
//   EXPECT_EQ(junction->num_segments(), 1);
//   EXPECT_EQ(junction->id().id, "j:1-0");

//   auto segment = junction->segment(0);
//   EXPECT_EQ(segment->num_lanes(), 1);
//   EXPECT_EQ(segment->id().id, "s:1-0");

//   auto lane = segment->lane(0);
//   EXPECT_EQ(lane->id().id, "l:1_1_1-1_1_2");
//   EXPECT_RBOUNDS_EQ(lane->lane_bounds(0.), lane_bounds);
//   EXPECT_RBOUNDS_EQ(lane->driveable_bounds(0.), driveable_bounds);
// }

// // We create a T connection and check its creation and correct invariants from
// // the road geometry. It will look like:

// //            2.1.1
// //            |
// //            v
// //            2.1.2
// //            |
// //            v
// //            2.1.3
// //            \
// //             v
// // 1.1.1 ----->1.1.2------>1.1.3

// // This simple example is useful for checking several Lane and BranchPoint
// // functions.
// GTEST_TEST(RNDFBuilder, BuilderConnections) {
//   const double width = 5.;
//   const api::RBounds lane_bounds(-width / 2., width / 2.);
//   const api::RBounds driveable_bounds(-width / 2., width / 2.);
//   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
//     0.01,
//     0.01 * M_PI);

//   ConnectedLane l1, l2;
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 1),
//     ignition::math::Vector3d(0., 0., 0.)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 2),
//     ignition::math::Vector3d(10., 0., 0.),
//     true));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 3),
//     ignition::math::Vector3d(20., 0., 0.)));

//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(2, 1, 1),
//     ignition::math::Vector3d(5., 20., 0.)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(2, 1, 2),
//     ignition::math::Vector3d(5., 12.5, 0.)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(2, 1, 3),
//     ignition::math::Vector3d(5., 5., 0.),
//     false,
//     true));

//   l1.width = width;
//   l2.width = width;
//   {
//     std::vector<ConnectedLane> connected_lanes = {l1};
//     builder->CreateSegmentConnections(1, &connected_lanes);
//   }
//   {
//     std::vector<ConnectedLane> connected_lanes = {l2};
//     builder->CreateSegmentConnections(2, &connected_lanes);
//   }

//   EXPECT_THROW(builder->CreateConnection(width,
//     ignition::rndf::UniqueId(1, 1, 4),
//     ignition::rndf::UniqueId(1, 1, 3)), std::runtime_error);
//   EXPECT_THROW(builder->CreateConnection(width,
//     ignition::rndf::UniqueId(1, 1, 3),
//     ignition::rndf::UniqueId(1, 1, 4)), std::runtime_error);

//   EXPECT_NO_THROW(builder->CreateConnection(width,
//     ignition::rndf::UniqueId(2, 1, 3),
//     ignition::rndf::UniqueId(1, 1, 2)));

//   auto road_geometry = builder->Build({"ConnectionsChecker"});
//   EXPECT_EQ(road_geometry->CheckInvariants().size(), 0);
// }


// GTEST_TEST(RNDFBuilder, BuildT) {
//   const double width = 5.;
//   const api::RBounds lane_bounds(-width / 2., width / 2.);
//   const api::RBounds driveable_bounds(-width / 2., width / 2.);
//   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
//     0.01,
//     0.01 * M_PI);

//   ConnectedLane l1, l2;
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 1),
//     ignition::math::Vector3d(0., 0., 0.)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 2),
//     ignition::math::Vector3d(10., 0., 0.),
//     true));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 3),
//     ignition::math::Vector3d(20., 0., 0.)));

//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(2, 1, 1),
//     ignition::math::Vector3d(5., 20., 0.)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(2, 1, 2),
//     ignition::math::Vector3d(5., 12.5, 0.)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(2, 1, 3),
//     ignition::math::Vector3d(5., 5., 0.),
//     false,
//     true));

//   l1.width = width;
//   l2.width = width;
//   {
//     std::vector<ConnectedLane> connected_lanes = {l1};
//     builder->CreateSegmentConnections(1, &connected_lanes);
//   }
//   {
//     std::vector<ConnectedLane> connected_lanes = {l2};
//     builder->CreateSegmentConnections(2, &connected_lanes);
//   }

//   builder->CreateConnection(width,
//     ignition::rndf::UniqueId(2, 1, 3),
//     ignition::rndf::UniqueId(1, 1, 2));

//   auto road_geometry = builder->Build({"T"});
//   EXPECT_NE(road_geometry, nullptr);

//   EXPECT_EQ(road_geometry->num_junctions(), 5);
//   {
//     auto junction = road_geometry->junction(0);
//     EXPECT_EQ(junction->num_segments(), 1);
//     EXPECT_EQ(junction->id().id, "j:1-0");

//     auto segment = junction->segment(0);
//     EXPECT_EQ(segment->num_lanes(), 1);
//     EXPECT_EQ(segment->id().id, "s:1-0");

//     auto lane = segment->lane(0);
//     EXPECT_EQ(lane->id().id, "l:1_1_1-1_1_2");
//     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kStart), nullptr);
//     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kStart)->size(), 1);
//     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kFinish), nullptr);
//     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kFinish)->size(), 2);
//     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kStart), nullptr);
//     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kStart)->size(), 0);
//     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kFinish), nullptr);
//     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kFinish)->size(), 1);

//     auto *start_branch = lane->GetBranchPoint(api::LaneEnd::kStart);
//     EXPECT_NE(start_branch, nullptr);
//     EXPECT_NE(start_branch->GetASide(), nullptr);
//     EXPECT_NE(start_branch->GetBSide(), nullptr);
//     EXPECT_EQ(start_branch->GetASide()->size(), 1);
//     EXPECT_EQ(start_branch->GetBSide()->size(), 0);
//     EXPECT_EQ(start_branch->GetASide()->get(0).lane, lane);
//     EXPECT_EQ(start_branch->GetASide()->get(0).end, api::LaneEnd::kStart);

//     auto *end_branch = lane->GetBranchPoint(api::LaneEnd::kFinish);
//     EXPECT_NE(end_branch, nullptr);
//     EXPECT_NE(end_branch->GetASide(), nullptr);
//     EXPECT_NE(end_branch->GetBSide(), nullptr);
//     EXPECT_EQ(end_branch->GetASide()->size(), 2);
//     EXPECT_EQ(end_branch->GetBSide()->size(), 1);
//     EXPECT_EQ(end_branch->GetASide()->get(0).lane, lane);
//     EXPECT_EQ(end_branch->GetASide()->get(0).end, api::LaneEnd::kFinish);
//   }
//   {
//     auto junction = road_geometry->junction(1);
//     EXPECT_EQ(junction->num_segments(), 1);
//     EXPECT_EQ(junction->id().id, "j:1-1");

//     auto segment = junction->segment(0);
//     EXPECT_EQ(segment->num_lanes(), 1);
//     EXPECT_EQ(segment->id().id, "s:1-1");

//     auto lane = segment->lane(0);
//     EXPECT_EQ(lane->id().id, "l:1_1_2-1_1_3");
//     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kStart), nullptr);
//     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kStart)->size(), 1);
//     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kFinish), nullptr);
//     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kFinish)->size(), 1);
//     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kStart), nullptr);
//     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kStart)->size(), 2);
//     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kFinish), nullptr);
//     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kFinish)->size(), 0);

//     auto *start_branch = lane->GetBranchPoint(api::LaneEnd::kStart);
//     EXPECT_NE(start_branch, nullptr);
//     EXPECT_NE(start_branch->GetASide(), nullptr);
//     EXPECT_NE(start_branch->GetBSide(), nullptr);
//     EXPECT_EQ(start_branch->GetASide()->size(), 2);
//     EXPECT_EQ(start_branch->GetBSide()->size(), 1);
//     EXPECT_EQ(start_branch->GetBSide()->get(0).lane, lane);
//     EXPECT_EQ(start_branch->GetBSide()->get(0).end, api::LaneEnd::kStart);

//     auto *end_branch = lane->GetBranchPoint(api::LaneEnd::kFinish);
//     EXPECT_NE(end_branch, nullptr);
//     EXPECT_NE(end_branch->GetASide(), nullptr);
//     EXPECT_NE(end_branch->GetBSide(), nullptr);
//     EXPECT_EQ(end_branch->GetASide()->size(), 1);
//     EXPECT_EQ(end_branch->GetBSide()->size(), 0);
//     EXPECT_EQ(end_branch->GetASide()->get(0).lane, lane);
//     EXPECT_EQ(end_branch->GetASide()->get(0).end, api::LaneEnd::kFinish);
//   }
//   {
//     auto junction = road_geometry->junction(2);
//     EXPECT_EQ(junction->num_segments(), 1);
//     EXPECT_EQ(junction->id().id, "j:2-0");

//     auto segment = junction->segment(0);
//     EXPECT_EQ(segment->num_lanes(), 1);
//     EXPECT_EQ(segment->id().id, "s:2-0");

//     auto lane = segment->lane(0);
//     EXPECT_EQ(lane->id().id, "l:2_1_1-2_1_2");
//     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kStart), nullptr);
//     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kStart)->size(), 1);
//     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kFinish), nullptr);
//     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kFinish)->size(), 1);
//     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kStart), nullptr);
//     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kStart)->size(), 0);
//     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kFinish), nullptr);
//     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kFinish)->size(), 1);

//     auto *start_branch = lane->GetBranchPoint(api::LaneEnd::kStart);
//     EXPECT_NE(start_branch, nullptr);
//     EXPECT_NE(start_branch->GetASide(), nullptr);
//     EXPECT_NE(start_branch->GetBSide(), nullptr);
//     EXPECT_EQ(start_branch->GetASide()->size(), 1);
//     EXPECT_EQ(start_branch->GetBSide()->size(), 0);
//     EXPECT_EQ(start_branch->GetASide()->get(0).lane, lane);
//     EXPECT_EQ(start_branch->GetASide()->get(0).end, api::LaneEnd::kStart);

//     // TODO(@agalbachicar) End brnach
//   }
//   {
//     auto junction = road_geometry->junction(3);
//     EXPECT_EQ(junction->num_segments(), 1);
//     EXPECT_EQ(junction->id().id, "j:2-1");

//     auto segment = junction->segment(0);
//     EXPECT_EQ(segment->num_lanes(), 1);
//     EXPECT_EQ(segment->id().id, "s:2-1");

//     auto lane = segment->lane(0);
//     EXPECT_EQ(lane->id().id, "l:2_1_2-2_1_3");
//     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kStart), nullptr);
//     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kStart)->size(), 1);
//     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kFinish), nullptr);
//     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kFinish)->size(), 1);
//     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kStart), nullptr);
//     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kStart)->size(), 1);
//     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kFinish), nullptr);
//     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kFinish)->size(), 1);

//     // TODO(@agalbachicar) Start branch

//     auto *end_branch = lane->GetBranchPoint(api::LaneEnd::kFinish);
//     EXPECT_NE(end_branch, nullptr);
//     EXPECT_NE(end_branch->GetASide(), nullptr);
//     EXPECT_NE(end_branch->GetBSide(), nullptr);
//     EXPECT_EQ(end_branch->GetASide()->size(), 1);
//     EXPECT_EQ(end_branch->GetBSide()->size(), 1);
//     EXPECT_EQ(end_branch->GetASide()->get(0).lane, lane);
//     EXPECT_EQ(end_branch->GetASide()->get(0).end, api::LaneEnd::kFinish);
//   }

//   {
//     auto junction = road_geometry->junction(4);
//     EXPECT_EQ(junction->num_segments(), 1);
//     EXPECT_EQ(junction->id().id, "j:2_1_3-1_1_2");

//     auto segment = junction->segment(0);
//     EXPECT_EQ(segment->num_lanes(), 1);
//     EXPECT_EQ(segment->id().id, "s:2_1_3-1_1_2");
//     auto lane = segment->lane(0);
//     EXPECT_EQ(lane->id().id, "l:2_1_3-1_1_2");
//     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kStart), nullptr);
//     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kStart)->size(), 1);
//     EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kFinish), nullptr);
//     EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kFinish)->size(), 2);
//     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kStart), nullptr);
//     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kStart)->size(), 1);
//     EXPECT_NE(lane->GetOngoingBranches(api::LaneEnd::kFinish), nullptr);
//     EXPECT_EQ(lane->GetOngoingBranches(api::LaneEnd::kFinish)->size(), 1);

//     auto *start_branch = lane->GetBranchPoint(api::LaneEnd::kStart);
//     EXPECT_NE(start_branch, nullptr);
//     EXPECT_NE(start_branch->GetASide(), nullptr);
//     EXPECT_NE(start_branch->GetBSide(), nullptr);
//     EXPECT_EQ(start_branch->GetASide()->size(), 1);
//     EXPECT_EQ(start_branch->GetBSide()->size(), 1);
//     EXPECT_EQ(start_branch->GetBSide()->get(0).lane, lane);
//     EXPECT_EQ(start_branch->GetBSide()->get(0).end, api::LaneEnd::kStart);

//     auto *end_branch = lane->GetBranchPoint(api::LaneEnd::kFinish);
//     EXPECT_NE(end_branch, nullptr);
//     EXPECT_NE(end_branch->GetASide(), nullptr);
//     EXPECT_NE(end_branch->GetBSide(), nullptr);
//     EXPECT_EQ(end_branch->GetASide()->size(), 2);
//     EXPECT_EQ(end_branch->GetBSide()->size(), 1);
//     EXPECT_EQ(end_branch->GetASide()->get(1).lane, lane);
//     EXPECT_EQ(end_branch->GetASide()->get(1).end, api::LaneEnd::kFinish);
//   }
// }

// /*
// // We create a T connection and check its creation and correct invariants from
// // the road geometry. It will look like:

// //                                    2.1.1
// //                                    |
// //                                    v
// //                                    2.1.2
// //                                    |
// //                                    v
// //                                    2.1.3
// //                                    \
// //                                     v
// // 1.1.1 ----->1.1.2------>1.1.3------>1.1.4------>1.1.5

// // This simple example is useful for checking the how the waypoint grouping is
// // working
// GTEST_TEST(RNDFBuilder, BuildTToCheckWaypointGrouping) {
//   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
//   const api::RBounds driveable_bounds(-10/ 2., 10. / 2.);

//   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
//     lane_bounds,
//     driveable_bounds,
//     0.01,
//     0.01 * M_PI);

//   {
//     std::vector<DirectedWaypoint> points = {
//       DirectedWaypoint(
//         ignition::rndf::UniqueId(1, 1, 1),
//         ignition::math::Vector3d(0., 0., 0.)),
//       DirectedWaypoint(
//         ignition::rndf::UniqueId(1, 1, 2),
//         ignition::math::Vector3d(5., 0., 0.)),
//       DirectedWaypoint(
//         ignition::rndf::UniqueId(1, 1, 3),
//         ignition::math::Vector3d(5., 0., 0.)),
//       DirectedWaypoint(
//         ignition::rndf::UniqueId(1, 1, 4),
//         ignition::math::Vector3d(20., 0., 0.),
//         true,
//         false),
//       DirectedWaypoint(
//         ignition::rndf::UniqueId(1, 1, 5),
//         ignition::math::Vector3d(25., 0., 0.)),
//     };
//     builder->CreateLaneConnections(1, 1, points, lane_bounds, driveable_bounds);
//   }
//   {
//     std::vector<DirectedWaypoint> points = {
//       DirectedWaypoint(
//         ignition::rndf::UniqueId(2, 1, 1),
//         ignition::math::Vector3d(5., 20., 0.)),
//       DirectedWaypoint(
//         ignition::rndf::UniqueId(2, 1, 2),
//         ignition::math::Vector3d(5., 12.5, 0.)),
//       DirectedWaypoint(
//         ignition::rndf::UniqueId(2, 1, 3),
//         ignition::math::Vector3d(5., 5., 0.),
//         false,
//         true)
//     };
//     builder->CreateLaneConnections(2, 1, points, lane_bounds, driveable_bounds);
//   }
//   builder->CreateConnection(lane_bounds,
//     driveable_bounds,
//     ignition::rndf::UniqueId(2, 1, 3),
//     ignition::rndf::UniqueId(1, 1, 4));

//   auto road_geometry = builder->Build({"L"});
//   EXPECT_NE(road_geometry, nullptr);

//   EXPECT_EQ(road_geometry->num_junctions(), 4);
//   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1_1_1-1_1_4");
//   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1_1_4-1_1_5");
//   EXPECT_EQ(road_geometry->junction(2)->id().id, "j:2_1_1-2_1_3");
//   EXPECT_EQ(road_geometry->junction(3)->id().id, "j:2_1_3-1_1_4");
// }
// */

// // This test checks that the lanes are set inside the segment in the correct
// // order. First we build a road geometry with a simple segment with two lanes
// // which are OK and then we build the same but the points are swapped. The
// // expected final result is the correct order of the lanes inside the segment.
// GTEST_TEST(RNDFBuilder, LaneRightToLeft) {
//   const double width = 5.;
//   const api::RBounds lane_bounds(-width / 2., width / 2.);
//   const api::RBounds driveable_bounds(-width / 2., width / 2.);
//   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
//     0.01,
//     0.01 * M_PI);

//   {
//     ConnectedLane l1, l2;
//     l1.waypoints.push_back(DirectedWaypoint(
//       ignition::rndf::UniqueId(1, 1, 1),
//       ignition::math::Vector3d(0.0, 0.0, 0.0)));
//     l1.waypoints.push_back(DirectedWaypoint(
//       ignition::rndf::UniqueId(1, 1, 2),
//       ignition::math::Vector3d(10.0, 0.0, 0.0)));

//     l2.waypoints.push_back(DirectedWaypoint(
//       ignition::rndf::UniqueId(1, 2, 1),
//       ignition::math::Vector3d(0.0, 5.0, 0.0)));
//     l2.waypoints.push_back(DirectedWaypoint(
//       ignition::rndf::UniqueId(1, 2, 2),
//       ignition::math::Vector3d(10.0, 5.0, 0.0)));


//     l1.width = width;
//     l2.width = width;
//     std::vector<ConnectedLane> connected_lanes = {l1, l2};

//     builder->CreateSegmentConnections(1, &connected_lanes);
//     auto road_geometry = builder->Build({"LaneRightToLeft"});
//     EXPECT_NE(road_geometry, nullptr);

//     EXPECT_EQ(road_geometry->num_junctions(), 1);
//     EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
//     EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
//     EXPECT_EQ(
//       road_geometry->junction(0)->segment(0)->lane(0)->id().id,
//       "l:1_1_1-1_1_2");
//     EXPECT_EQ(
//       road_geometry->junction(0)->segment(0)->lane(1)->id().id,
//       "l:1_2_1-1_2_2");
//   }

//   // Create a new builder
//   builder = std::make_unique<Builder>(
//     0.01,
//     0.01 * M_PI);
//   {
//     ConnectedLane l1, l2;
//     l1.waypoints.push_back(DirectedWaypoint(
//       ignition::rndf::UniqueId(1, 1, 1),
//       ignition::math::Vector3d(0.0, 5.0, 0.0)));
//     l1.waypoints.push_back(DirectedWaypoint(
//       ignition::rndf::UniqueId(1, 1, 2),
//       ignition::math::Vector3d(10.0, 5.0, 0.0)));

//     l2.waypoints.push_back(DirectedWaypoint(
//       ignition::rndf::UniqueId(1, 2, 1),
//       ignition::math::Vector3d(0.0, 0.0, 0.0)));
//     l2.waypoints.push_back(DirectedWaypoint(
//       ignition::rndf::UniqueId(1, 2, 2),
//       ignition::math::Vector3d(10.0, 0.0, 0.0)));


//     l1.width = width;
//     l2.width = width;
//     std::vector<ConnectedLane> connected_lanes = {l1, l2};

//     builder->CreateSegmentConnections(1, &connected_lanes);
//     auto road_geometry = builder->Build({"LaneLeftToRight"});
//     EXPECT_NE(road_geometry, nullptr);

//     EXPECT_EQ(road_geometry->num_junctions(), 1);
//     EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
//     EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
//     EXPECT_EQ(
//       road_geometry->junction(0)->segment(0)->lane(0)->id().id,
//       "l:1_2_1-1_2_2");
//     EXPECT_EQ(
//       road_geometry->junction(0)->segment(0)->lane(1)->id().id,
//       "l:1_1_1-1_1_2");
//   }
// }


// //        * --> *
// //  * --> * --> *
// GTEST_TEST(RNDFBuilder, LaneStartsAfter) {
//   const double width = 5.;
//   const api::RBounds lane_bounds(-width / 2., width / 2.);
//   const api::RBounds driveable_bounds(-width / 2., width / 2.);
//   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
//     0.01,
//     0.01 * M_PI);

//   ConnectedLane l1, l2;
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 1),
//     ignition::math::Vector3d(5.0, 0.0, 0.0)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 2),
//     ignition::math::Vector3d(10.0, 0.0, 0.0)));

//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 1),
//     ignition::math::Vector3d(0.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 2),
//     ignition::math::Vector3d(5.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 3),
//     ignition::math::Vector3d(10.0, 5.0, 0.0)));

//   l1.width = width;
//   l2.width = width;
//   std::vector<ConnectedLane> connected_lanes = {l1, l2};

//   builder->CreateSegmentConnections(1, &connected_lanes);

//   auto road_geometry = builder->Build({"LaneStartsAfter"});
//   EXPECT_NE(road_geometry, nullptr);

//   EXPECT_EQ(road_geometry->num_junctions(), 2);
//   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
//   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1-1");

//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
//   EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id, "s:1-1");

//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
//   EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 2);

//   EXPECT_EQ(
//     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_2_1-1_2_2");
//   EXPECT_EQ(
//     road_geometry->junction(1)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_2");
//   EXPECT_EQ(
//     road_geometry->junction(1)->segment(0)->lane(1)->id().id, "l:1_2_2-1_2_3");
// }

// //              * --> *
// //  * --> * --> * --> *
// GTEST_TEST(RNDFBuilder, LaneStartsAfter2) {
//   const double width = 5.;
//   const api::RBounds lane_bounds(-width / 2., width / 2.);
//   const api::RBounds driveable_bounds(-width / 2., width / 2.);
//   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
//     0.01,
//     0.01 * M_PI);

//   ConnectedLane l1, l2;

//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 1),
//     ignition::math::Vector3d(5.0, 0.0, 0.0)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 2),
//     ignition::math::Vector3d(10.0, 0.0, 0.0)));

//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 1),
//     ignition::math::Vector3d(0.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 2),
//     ignition::math::Vector3d(2.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 3),
//     ignition::math::Vector3d(5.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 4),
//     ignition::math::Vector3d(10.0, 5.0, 0.0)));

//   l1.width = width;
//   l2.width = width;
//   std::vector<ConnectedLane> connected_lanes = {l1, l2};

//   builder->CreateSegmentConnections(1, &connected_lanes);

//   auto road_geometry = builder->Build({"LaneStartsAfter2"});
//   EXPECT_NE(road_geometry, nullptr);

//   EXPECT_EQ(road_geometry->num_junctions(), 3);
//   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
//   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1-1");
//   EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1-2");

//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
//   EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id, "s:1-1");
//   EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().id, "s:1-2");

//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
//   EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 1);
//   EXPECT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 2);

//   EXPECT_EQ(
//     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_2_1-1_2_2");
//   EXPECT_EQ(
//     road_geometry->junction(1)->segment(0)->lane(0)->id().id, "l:1_2_2-1_2_3");
//   EXPECT_EQ(
//     road_geometry->junction(2)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_2");
//   EXPECT_EQ(
//     road_geometry->junction(2)->segment(0)->lane(1)->id().id, "l:1_2_3-1_2_4");
// }

// //  * --> *
// //  * --> * --> *
// GTEST_TEST(RNDFBuilder, LaneEndsBefore) {
//   const double width = 5.;
//   const api::RBounds lane_bounds(-width / 2., width / 2.);
//   const api::RBounds driveable_bounds(-width / 2., width / 2.);
//   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
//     0.01,
//     0.01 * M_PI);

//   ConnectedLane l1, l2;

//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 1),
//     ignition::math::Vector3d(0.0, 0.0, 0.0)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 2),
//     ignition::math::Vector3d(5.0, 0.0, 0.0)));

//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 1),
//     ignition::math::Vector3d(0.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 2),
//     ignition::math::Vector3d(5.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 3),
//     ignition::math::Vector3d(10.0, 5.0, 0.0)));

//   l1.width = width;
//   l2.width = width;
//   std::vector<ConnectedLane> connected_lanes = {l1, l2};

//   builder->CreateSegmentConnections(1, &connected_lanes);

//   auto road_geometry = builder->Build({"LaneEndsBefore"});
//   EXPECT_NE(road_geometry, nullptr);

//   EXPECT_EQ(road_geometry->num_junctions(), 2);
//   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
//   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1-1");

//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
//   EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id, "s:1-1");

//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 2);
//   EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 1);

//   EXPECT_EQ(
//     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_2");
//   EXPECT_EQ(
//     road_geometry->junction(0)->segment(0)->lane(1)->id().id, "l:1_2_1-1_2_2");
//   EXPECT_EQ(
//     road_geometry->junction(1)->segment(0)->lane(0)->id().id, "l:1_2_2-1_2_3");
// }

// //  * --> *
// //  * --> * --> * --> *
// GTEST_TEST(RNDFBuilder, LaneEndsBefore2) {
//   const double width = 5.;
//   const api::RBounds lane_bounds(-width / 2., width / 2.);
//   const api::RBounds driveable_bounds(-width / 2., width / 2.);
// >>>>>>> b0aa276... Tests passing.
//   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
//     0.01,
//     0.01 * M_PI);

//   ConnectedLane l1, l2;
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 1),
//     ignition::math::Vector3d(0.0, 0.0, 0.0)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 2),
//     ignition::math::Vector3d(5.0, 0.0, 0.0)));
// <<<<<<< HEAD
// =======

//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 1),
//     ignition::math::Vector3d(0.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 2),
//     ignition::math::Vector3d(5.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 3),
//     ignition::math::Vector3d(10.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 4),
//     ignition::math::Vector3d(15.0, 5.0, 0.0)));

//   l1.width = width;
//   l2.width = width;
//   std::vector<ConnectedLane> connected_lanes = {l1, l2};

//   builder->CreateSegmentConnections(1, &connected_lanes);

//   auto road_geometry = builder->Build({"LaneEndsBefore2"});
//   EXPECT_NE(road_geometry, nullptr);

//   EXPECT_EQ(road_geometry->num_junctions(), 3);
//   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
//   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1-1");
//   EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1-2");

//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
//   EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id, "s:1-1");
//   EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().id, "s:1-2");

//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 2);
//   EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 1);
//   EXPECT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 1);

//   EXPECT_EQ(
//     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_2");
//   EXPECT_EQ(
//     road_geometry->junction(0)->segment(0)->lane(1)->id().id, "l:1_2_1-1_2_2");
//   EXPECT_EQ(
//     road_geometry->junction(1)->segment(0)->lane(0)->id().id, "l:1_2_2-1_2_3");
//   EXPECT_EQ(
//     road_geometry->junction(2)->segment(0)->lane(0)->id().id, "l:1_2_3-1_2_4");
// }

// //        * --> *
// //  * --> * --> * --> *
// GTEST_TEST(RNDFBuilder, LaneStartsAfterAndEndsBefore) {
//   const double width = 5.;
//   const api::RBounds lane_bounds(-width / 2., width / 2.);
//   const api::RBounds driveable_bounds(-width / 2., width / 2.);
//   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
//     0.01,
//     0.01 * M_PI);

//   ConnectedLane l1, l2;

//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 1),
//     ignition::math::Vector3d(5.0, 0.0, 0.0)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 2),
//     ignition::math::Vector3d(10.0, 0.0, 0.0)));

//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 1),
//     ignition::math::Vector3d(0.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 2),
//     ignition::math::Vector3d(5.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 3),
//     ignition::math::Vector3d(10.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 4),
//     ignition::math::Vector3d(15.0, 5.0, 0.0)));

//   l1.width = width;
//   l2.width = width;
//   std::vector<ConnectedLane> connected_lanes = {l1, l2};

//   builder->CreateSegmentConnections(1, &connected_lanes);

//   auto road_geometry = builder->Build({"LaneStartsAfterAndEndsBefore"});
//   EXPECT_NE(road_geometry, nullptr);

//   EXPECT_EQ(road_geometry->num_junctions(), 3);
//   EXPECT_EQ(road_geometry->junction(0)->id().id, "j:1-0");
//   EXPECT_EQ(road_geometry->junction(1)->id().id, "j:1-1");
//   EXPECT_EQ(road_geometry->junction(2)->id().id, "j:1-2");

//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, "s:1-0");
//   EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id, "s:1-1");
//   EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().id, "s:1-2");

//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
//   EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 2);
//   EXPECT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 1);

//   EXPECT_EQ(
//     road_geometry->junction(0)->segment(0)->lane(0)->id().id, "l:1_2_1-1_2_2");
//   EXPECT_EQ(
//     road_geometry->junction(1)->segment(0)->lane(0)->id().id, "l:1_1_1-1_1_2");
//   EXPECT_EQ(
//     road_geometry->junction(1)->segment(0)->lane(1)->id().id, "l:1_2_2-1_2_3");
//   EXPECT_EQ(
//     road_geometry->junction(2)->segment(0)->lane(0)->id().id, "l:1_2_3-1_2_4");
// }

// //     * --> *
// // * ------> *
// GTEST_TEST(RNDFBuilder, LaneStartsAfterButNotInPhase) {
//   const double width = 5.;
//   const api::RBounds lane_bounds(-width / 2., width / 2.);
//   const api::RBounds driveable_bounds(-width / 2., width / 2.);
//   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
//     0.01,
//     0.01 * M_PI);

//   ConnectedLane l1, l2;

//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 1),
//     ignition::math::Vector3d(7.5, 0.0, 0.0)));
// >>>>>>> b0aa276... Tests passing.
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 3),
//     ignition::math::Vector3d(10.0, 0.0, 0.0)));

// <<<<<<< HEAD
//   l1.lane_bounds = lane_bounds;
//   l1.driveable_bounds = driveable_bounds;
//   std::vector<ConnectedLane> connected_lanes = {l1};
// =======
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 1),
//     ignition::math::Vector3d(0.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 2),
//     ignition::math::Vector3d(10.0, 5.0, 0.0)));

//   l1.width = width;
//   l2.width = width;
//   std::vector<ConnectedLane> connected_lanes = {l1, l2};
// >>>>>>> b0aa276... Tests passing.

//   builder->CreateSegmentConnections(1, &connected_lanes);

//   auto road_geometry = builder->Build({"NA_Lane"});
//   EXPECT_NE(road_geometry, nullptr);

//   EXPECT_EQ(road_geometry->num_junctions(), 1);
//   EXPECT_EQ(road_geometry->junction(0)->id().id, std::string("j:1-0"));
//   EXPECT_EQ(road_geometry->junction(0)->num_segments(), 1);
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, std::string("s:1-0"));
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->id().id, std::string("l:1_1_1-1_1_3"));
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(0);
//     auto geo_position = lane->ToGeoPosition(api::LanePosition(0., 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(0.0, 0.0, 0.0),
//       kLinearTolerance);
//   }
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(0);
//     auto geo_position =
//       lane->ToGeoPosition(api::LanePosition(lane->length(), 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 0.0, 0.0),
//       kLinearTolerance);
//   }
// }

// <<<<<<< HEAD
// GTEST_TEST(RNDFBuilder, NA_Two_Lane) {
//   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
//   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// =======
// //         * --> *
// // * --> * ----> *
// GTEST_TEST(RNDFBuilder, LaneStartsAfterButNotInPhase2) {
//   const double width = 5.;
//   const api::RBounds lane_bounds(-width / 2., width / 2.);
//   const api::RBounds driveable_bounds(-width / 2., width / 2.);
// >>>>>>> b0aa276... Tests passing.
//   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
//     0.01,
//     0.01 * M_PI);

//   ConnectedLane l1, l2;
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 1),
//     ignition::math::Vector3d(0.0, 0.0, 0.0)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 2),
//     ignition::math::Vector3d(5.0, 0.0, 0.0)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 3),
//     ignition::math::Vector3d(10.0, 0.0, 0.0)));

//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 1),
//     ignition::math::Vector3d(0.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 2),
//     ignition::math::Vector3d(5.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 3),
//     ignition::math::Vector3d(10.0, 5.0, 0.0)));

// <<<<<<< HEAD
//   l1.lane_bounds = lane_bounds;
//   l1.driveable_bounds = driveable_bounds;
//   l2.lane_bounds = lane_bounds;
//   l2.driveable_bounds = driveable_bounds;
// =======
//   l1.width = width;
//   l2.width = width;
// >>>>>>> b0aa276... Tests passing.
//   std::vector<ConnectedLane> connected_lanes = {l1, l2};

//   builder->CreateSegmentConnections(1, &connected_lanes);

//   auto road_geometry = builder->Build({"NA_Lane"});
//   EXPECT_NE(road_geometry, nullptr);

//   EXPECT_EQ(road_geometry->num_junctions(), 1);
//   EXPECT_EQ(road_geometry->junction(0)->id().id, std::string("j:1-0"));
//   EXPECT_EQ(road_geometry->junction(0)->num_segments(), 1);
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, std::string("s:1-0"));
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 2);
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->id().id, std::string("l:1_1_1-1_1_3"));
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(1)->id().id, std::string("l:1_2_1-1_2_3"));
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(0);
//     auto geo_position = lane->ToGeoPosition(api::LanePosition(0., 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(0.0, 0.0, 0.0),
//       kLinearTolerance);
//   }
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(0);
//     auto geo_position =
//       lane->ToGeoPosition(api::LanePosition(lane->length(), 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 0.0, 0.0),
//       kLinearTolerance);
//   }
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(1);
//     auto geo_position = lane->ToGeoPosition(api::LanePosition(0., 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(0.0, 5.0, 0.0),
//       kLinearTolerance);
//   }
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(1);
//     auto geo_position =
//       lane->ToGeoPosition(api::LanePosition(lane->length(), 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 5.0, 0.0),
//       kLinearTolerance);
//   }
// }

// <<<<<<< HEAD

// GTEST_TEST(RNDFBuilder, NA_Two_Lane_With_Phase_At_Beginning) {
//   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
//   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// =======
// //     * --> *
// // * ----------> *
// GTEST_TEST(RNDFBuilder, LaneStartsAfterEndsAfterNotInPhase) {
//   const double width = 5.;
//   const api::RBounds lane_bounds(-width / 2., width / 2.);
//   const api::RBounds driveable_bounds(-width / 2., width / 2.);
// >>>>>>> b0aa276... Tests passing.
//   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
//     0.01,
//     0.01 * M_PI);

//   ConnectedLane l1, l2;
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 1),
//     ignition::math::Vector3d(0.0, 0.0, 0.0)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 2),
//     ignition::math::Vector3d(5.0, 0.0, 0.0)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 3),
//     ignition::math::Vector3d(10.0, 0.0, 0.0)));

//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 1),
//     ignition::math::Vector3d(5.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 2),
//     ignition::math::Vector3d(10.0, 5.0, 0.0)));

// <<<<<<< HEAD
//   l1.lane_bounds = lane_bounds;
//   l1.driveable_bounds = driveable_bounds;
//   l2.lane_bounds = lane_bounds;
//   l2.driveable_bounds = driveable_bounds;
// =======
//   l1.width = width;
//   l2.width = width;
// >>>>>>> b0aa276... Tests passing.
//   std::vector<ConnectedLane> connected_lanes = {l1, l2};

//   builder->CreateSegmentConnections(1, &connected_lanes);

//   auto road_geometry = builder->Build({"NA_Two_Lane_With_Phase_At_Beginning"});
//   EXPECT_NE(road_geometry, nullptr);

//   EXPECT_EQ(road_geometry->num_junctions(), 1);
//   EXPECT_EQ(road_geometry->junction(0)->id().id, std::string("j:1-0"));
//   EXPECT_EQ(road_geometry->junction(0)->num_segments(), 1);
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, std::string("s:1-0"));
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 2);
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->id().id, std::string("l:1_1_1-1_1_3"));
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(1)->id().id, std::string("l:1_2_3-1_2_2"));
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(0);
//     auto geo_position = lane->ToGeoPosition(api::LanePosition(0., 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(0.0, 0.0, 0.0),
//       kLinearTolerance);
//   }
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(0);
//     auto geo_position =
//       lane->ToGeoPosition(api::LanePosition(lane->length(), 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 0.0, 0.0),
//       kLinearTolerance);
//   }
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(1);
//     auto geo_position = lane->ToGeoPosition(api::LanePosition(0., 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(0.0, 5.0, 0.0),
//       kLinearTolerance);
//   }
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(1);
//     auto geo_position =
//       lane->ToGeoPosition(api::LanePosition(lane->length(), 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 5.0, 0.0),
//       kLinearTolerance);
//   }
// }

// <<<<<<< HEAD
// GTEST_TEST(RNDFBuilder, NA_Two_Lane_With_Phase_At_Beginning_And_Ending) {
//   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
//   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// =======
// //     * ----> *
// // * ----> * -----> *
// GTEST_TEST(RNDFBuilder, TotallyDephasedLanes) {
//   const double width = 5.;
//   const api::RBounds lane_bounds(-width / 2., width / 2.);
//   const api::RBounds driveable_bounds(-width / 2., width / 2.);
// >>>>>>> b0aa276... Tests passing.
//   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
//     0.01,
//     0.01 * M_PI);

//   ConnectedLane l1, l2;
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 1),
//     ignition::math::Vector3d(0.0, 0.0, 0.0)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 2),
//     ignition::math::Vector3d(5.0, 0.0, 0.0)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 3),
//     ignition::math::Vector3d(10.0, 0.0, 0.0)));

//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 1),
//     ignition::math::Vector3d(2.5, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 2),
//     ignition::math::Vector3d(5.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 3),
//     ignition::math::Vector3d(7.5, 5.0, 0.0)));

// <<<<<<< HEAD
//   l1.lane_bounds = lane_bounds;
//   l1.driveable_bounds = driveable_bounds;
//   l2.lane_bounds = lane_bounds;
//   l2.driveable_bounds = driveable_bounds;
// =======
//   l1.width = width;
//   l2.width = width;
// >>>>>>> b0aa276... Tests passing.
//   std::vector<ConnectedLane> connected_lanes = {l1, l2};

//   builder->CreateSegmentConnections(1, &connected_lanes);

//   auto road_geometry = builder->Build({"NA_Two_Lane_With_Phase_At_Beginning_And_Ending"});
//   EXPECT_NE(road_geometry, nullptr);

//   EXPECT_EQ(road_geometry->num_junctions(), 1);
//   EXPECT_EQ(road_geometry->junction(0)->id().id, std::string("j:1-0"));
//   EXPECT_EQ(road_geometry->junction(0)->num_segments(), 1);
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, std::string("s:1-0"));
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 2);
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->id().id, std::string("l:1_1_1-1_1_3"));
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(1)->id().id, std::string("l:1_2_4-1_2_5"));
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(0);
//     auto geo_position = lane->ToGeoPosition(api::LanePosition(0., 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(0.0, 0.0, 0.0),
//       kLinearTolerance);
//   }
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(0);
//     auto geo_position =
//       lane->ToGeoPosition(api::LanePosition(lane->length(), 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 0.0, 0.0),
//       kLinearTolerance);
//   }
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(1);
//     auto geo_position = lane->ToGeoPosition(api::LanePosition(0., 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(0.0, 5.0, 0.0),
//       kLinearTolerance);
//   }
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(1);
//     auto geo_position =
//       lane->ToGeoPosition(api::LanePosition(lane->length(), 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 5.0, 0.0),
//       kLinearTolerance);
//   }
// }

// <<<<<<< HEAD
// GTEST_TEST(RNDFBuilder, NA_Two_Lane_With_Phase_At_Beginning_And_Ending_One_Inverse) {
//   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
//   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// =======
// // * ---------> *
// //     * -----> *
// //         * -> *
// GTEST_TEST(RNDFBuilder, ThreeLaneStartWithPhaseEndCoherent) {
//   const double width = 5.;
//   const api::RBounds lane_bounds(-width / 2., width / 2.);
//   const api::RBounds driveable_bounds(-width / 2., width / 2.);
// >>>>>>> b0aa276... Tests passing.
//   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
//     0.01,
//     0.01 * M_PI);

//   ConnectedLane l1, l2;
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 1),
//     ignition::math::Vector3d(0.0, 0.0, 0.0)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 2),
//     ignition::math::Vector3d(5.0, 0.0, 0.0)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 3),
//     ignition::math::Vector3d(10.0, 0.0, 0.0)));

//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 1),
//     ignition::math::Vector3d(7.5, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 2),
//     ignition::math::Vector3d(5.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 3),
//     ignition::math::Vector3d(2.5, 5.0, 0.0)));

// <<<<<<< HEAD
//   l1.lane_bounds = lane_bounds;
//   l1.driveable_bounds = driveable_bounds;
//   l2.lane_bounds = lane_bounds;
//   l2.driveable_bounds = driveable_bounds;
//   std::vector<ConnectedLane> connected_lanes = {l1, l2};
// =======
//   l1.width = width;
//   l2.width = width;
//   l3.width = width;
//   std::vector<ConnectedLane> connected_lanes = {l1, l2, l3};
// >>>>>>> b0aa276... Tests passing.

//   builder->CreateSegmentConnections(1, &connected_lanes);

//   auto road_geometry = builder->Build({"NA_Two_Lane_With_Phase_At_Beginning_And_Ending_One_Inverse"});
//   EXPECT_NE(road_geometry, nullptr);

//   EXPECT_EQ(road_geometry->num_junctions(), 1);
//   EXPECT_EQ(road_geometry->junction(0)->id().id, std::string("j:1-0"));
//   EXPECT_EQ(road_geometry->junction(0)->num_segments(), 1);
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, std::string("s:1-0"));
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 2);
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->id().id, std::string("l:1_1_1-1_1_3"));
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(1)->id().id, std::string("l:1_2_5-1_2_4"));
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(0);
//     auto geo_position = lane->ToGeoPosition(api::LanePosition(0., 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(0.0, 0.0, 0.0),
//       kLinearTolerance);
//   }
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(0);
//     auto geo_position =
//       lane->ToGeoPosition(api::LanePosition(lane->length(), 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 0.0, 0.0),
//       kLinearTolerance);
//   }
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(1);
//     auto geo_position = lane->ToGeoPosition(api::LanePosition(0., 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 5.0, 0.0),
//       kLinearTolerance);
//   }
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(1);
//     auto geo_position =
//       lane->ToGeoPosition(api::LanePosition(lane->length(), 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(0.0, 5.0, 0.0),
//       kLinearTolerance);
//   }
// }

// <<<<<<< HEAD
// GTEST_TEST(RNDFBuilder, NA_Three_Lanes_With_Phase_At_Beginning_And_Ending_One_Inverse) {
//   const api::RBounds lane_bounds(-5. / 2., 5. / 2.);
//   const api::RBounds driveable_bounds(-5/ 2., 5. / 2.);

// =======
// //   * ------> *
// //     * -------> *
// // * --------------> *
// GTEST_TEST(RNDFBuilder, ThreeLaneStartAndEndWithPhases) {
//   const double width = 5.;
//   const api::RBounds lane_bounds(-width / 2., width / 2.);
//   const api::RBounds driveable_bounds(-width / 2., width / 2.);
// >>>>>>> b0aa276... Tests passing.
//   std::unique_ptr<Builder> builder = std::make_unique<Builder>(
//     0.01,
//     0.01 * M_PI);

//   ConnectedLane l1, l2, l3;
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 1),
//     ignition::math::Vector3d(-3., 0.0, 0.0)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 2),
//     ignition::math::Vector3d(5.0, 0.0, 0.0)));
//   l1.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 1, 3),
//     ignition::math::Vector3d(10.0, 0.0, 0.0)));

//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 1),
//     ignition::math::Vector3d(7.5, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 2),
//     ignition::math::Vector3d(5.0, 5.0, 0.0)));
//   l2.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 2, 3),
//     ignition::math::Vector3d(2.5, 5.0, 0.0)));

//   l3.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 3, 1),
//     ignition::math::Vector3d(1, -5.0, 0.0)));
//   l3.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 3, 2),
//     ignition::math::Vector3d(3.0, -5.0, 0.0)));
//   l3.waypoints.push_back(DirectedWaypoint(
//     ignition::rndf::UniqueId(1, 3, 3),
//     ignition::math::Vector3d(8, -5.0, 0.0)));

// <<<<<<< HEAD
//   l1.lane_bounds = lane_bounds;
//   l1.driveable_bounds = driveable_bounds;
//   l2.lane_bounds = lane_bounds;
//   l2.driveable_bounds = driveable_bounds;
//   std::vector<ConnectedLane> connected_lanes = {l1, /*l2,*/ l3};

// =======
//   l1.width = width;
//   l2.width = width;
//   l3.width = width;
//   std::vector<ConnectedLane> connected_lanes = {l1, l2, l3};
// >>>>>>> b0aa276... Tests passing.

//   auto bounding_box = std::make_tuple<ignition::math::Vector3d,
//     ignition::math::Vector3d> (ignition::math::Vector3d(-3., -5., 0.),
//       ignition::math::Vector3d(10., 5., 0.));
//   builder->SetBoundingBox(bounding_box);
//   builder->CreateSegmentConnections(1, &connected_lanes);

//   auto road_geometry = builder->Build({"NA_Three_Lanes_With_Phase_At_Beginning_And_Ending_One_Inverse"});
//   EXPECT_NE(road_geometry, nullptr);

//   EXPECT_EQ(road_geometry->num_junctions(), 1);
//   EXPECT_EQ(road_geometry->junction(0)->id().id, std::string("j:1-0"));
//   EXPECT_EQ(road_geometry->junction(0)->num_segments(), 1);
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id, std::string("s:1-0"));
//   // EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 3);
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 2);
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->id().id, std::string("l:1_3_4-1_3_5"));
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(1)->id().id, std::string("l:1_1_1-1_1_3"));
//   EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(2)->id().id, std::string("l:1_2_5-1_2_4"));
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(0);
//     auto geo_position = lane->ToGeoPosition(api::LanePosition(0., 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(0.0, -5., 0.0),
//       kLinearTolerance);
//     std::cout << lane->id().id << geo_position << std::endl;
//   }
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(0);
//     auto geo_position =
//       lane->ToGeoPosition(api::LanePosition(lane->length(), 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, -5., 0.0),
//       kLinearTolerance);
//     std::cout << lane->id().id << geo_position << std::endl;
//   }

//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(1);
//     auto geo_position = lane->ToGeoPosition(api::LanePosition(0., 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(0.0, 0.0, 0.0),
//       kLinearTolerance);
//     std::cout << lane->id().id << geo_position << std::endl;
//   }
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(1);
//     auto geo_position =
//       lane->ToGeoPosition(api::LanePosition(lane->length(), 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 0.0, 0.0),
//       kLinearTolerance);
//     std::cout << lane->id().id << geo_position << std::endl;
//   }
// /*
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(2);
//     auto geo_position = lane->ToGeoPosition(api::LanePosition(0., 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(10.0, 5.0, 0.0),
//       kLinearTolerance);
//     std::cout << lane->id().id << geo_position << std::endl;
//   }
//   {
//     const auto *lane = road_geometry->junction(0)->segment(0)->lane(2);
//     auto geo_position =
//       lane->ToGeoPosition(api::LanePosition(lane->length(), 0., 0.));
//     EXPECT_GEO_NEAR(geo_position, api::GeoPosition(0.0, 5.0, 0.0),
//       kLinearTolerance);
//     std::cout << lane->id().id << geo_position << std::endl;
//   }
// */
// }

// }  // namespace rndf
// }  // namespace maliput
// }  // namespace drake
