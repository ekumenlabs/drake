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

GTEST_TEST(RNDFBuilder, DirectedWaypointClass) {
  std::unique_ptr<DirectedWaypoint> directed_waypoint;

  directed_waypoint = std::make_unique<DirectedWaypoint>();
  EXPECT_EQ(directed_waypoint->Id(), ignition::rndf::UniqueId());
  EXPECT_EQ(directed_waypoint->Position(), ignition::math::Vector3d::Zero);
  EXPECT_EQ(directed_waypoint->Tangent(), ignition::math::Vector3d::Zero);

  {
    std::unique_ptr<DirectedWaypoint> copy_directed_waypoint = std::make_unique<DirectedWaypoint>(*directed_waypoint);
    EXPECT_EQ(copy_directed_waypoint->Id(), ignition::rndf::UniqueId());
    EXPECT_EQ(copy_directed_waypoint->Position(), ignition::math::Vector3d::Zero);
    EXPECT_EQ(copy_directed_waypoint->Tangent(), ignition::math::Vector3d::Zero);
  }

  directed_waypoint = std::make_unique<DirectedWaypoint>(ignition::rndf::UniqueId(1,2,3), ignition::math::Vector3d(1,2,3), ignition::math::Vector3d(4,5,6));
  EXPECT_EQ(directed_waypoint->Id(), ignition::rndf::UniqueId(1,2,3));
  EXPECT_EQ(directed_waypoint->Position(), ignition::math::Vector3d(1,2,3));
  EXPECT_EQ(directed_waypoint->Tangent(), ignition::math::Vector3d(4,5,6));
  {
    DirectedWaypoint copy_directed_waypoint = *directed_waypoint;
    EXPECT_EQ(directed_waypoint->Id(), ignition::rndf::UniqueId(1,2,3));
    EXPECT_EQ(directed_waypoint->Position(), ignition::math::Vector3d(1,2,3));
    EXPECT_EQ(directed_waypoint->Tangent(), ignition::math::Vector3d(4,5,6));
  }
}

GTEST_TEST(RNDFBuilder, ConnectionClass) {
  std::vector<DirectedWaypoint> directed_waypoints;
  directed_waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1,1,1),
    ignition::math::Vector3d(0.0, 0.0, 0.0),
    ignition::math::Vector3d(10.0, 0.0, 0.0)));
  directed_waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1,1,2),
    ignition::math::Vector3d(20.0, 0.0, 0.0),
    ignition::math::Vector3d(10.0, 0.0, 0.0)));

  std::unique_ptr<Connection> connection;
  connection = std::make_unique<Connection>("1_1_1-1_1_2", directed_waypoints);

  EXPECT_EQ(connection->type(), Connection::kSpline);
  EXPECT_EQ(connection->id(), "1_1_1-1_1_2");
  EXPECT_EQ(connection->start().Id(), ignition::rndf::UniqueId(1,1,1));
  EXPECT_EQ(connection->end().Id(), ignition::rndf::UniqueId(1,1,2));
  EXPECT_EQ(connection->waypoints().size(), 2);

  directed_waypoints.clear();
  EXPECT_THROW(std::make_unique<Connection>("1_1_1-1_1_2", directed_waypoints), std::runtime_error);
}

GTEST_TEST(RNDFBuilder, BuilderConstructor) {
  EXPECT_NO_THROW(std::make_unique<Builder>(
    api::RBounds(-5. / 2., 5. / 2.),
    api::RBounds(-10/ 2., 10. / 2.),
    0.01,
    0.01 * M_PI));

  EXPECT_THROW(std::make_unique<Builder>(
    api::RBounds(-15. / 2., 5. / 2.),
    api::RBounds(-10/ 2., 10. / 2.),
    0.01,
    0.01 * M_PI), std::runtime_error);

  EXPECT_THROW(std::make_unique<Builder>(
    api::RBounds(-5. / 2., 15. / 2.),
    api::RBounds(-10/ 2., 10. / 2.),
    0.01,
    0.01 * M_PI), std::runtime_error);
}

GTEST_TEST(RNDFBuilder, BuilderLaneConnections) {
  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    api::RBounds(-5. / 2., 5. / 2.),
    api::RBounds(-10/ 2., 10. / 2.),
    0.01,
    0.01 * M_PI);

  EXPECT_THROW(
  	builder->CreateLaneConnections(1, 1,
	  std::vector<ignition::math::Vector3d>()),
    std::runtime_error);

  std::vector<ignition::math::Vector3d> points = {
  	ignition::math::Vector3d(0., 0., 0.)
  };
  EXPECT_THROW(
  	builder->CreateLaneConnections(1, 1, points),
    std::runtime_error);

  points.push_back(ignition::math::Vector3d(10., 0., 0.));
  EXPECT_NO_THROW(builder->CreateLaneConnections(1, 1, points));

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
}

GTEST_TEST(RNDFBuilder, BuilderConnections) {
  api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  api::RBounds driveable_bounds(-10/ 2., 10. / 2.);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

	{
	  std::vector<ignition::math::Vector3d> points = {
	  	ignition::math::Vector3d(0., 0., 0.), // 1.1.1
	  	ignition::math::Vector3d(10., 0., 0.), // 1.1.2
	  	ignition::math::Vector3d(20., 0., 0.) // 1.1.3
	  };
	  builder->CreateLaneConnections(1, 1, points);
	}

	{
	  std::vector<ignition::math::Vector3d> points = {
	  	ignition::math::Vector3d(5., 20., 0.), // 2.1.1
	  	ignition::math::Vector3d(5., 12.5, 0.), // 2.1.2
	  	ignition::math::Vector3d(5., 5., 0.) // 2.1.3
	  };
	  builder->CreateLaneConnections(2, 1, points);
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
  api::RBounds lane_bounds(-5. / 2., 5. / 2.);
  api::RBounds driveable_bounds(-10/ 2., 10. / 2.);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    lane_bounds,
    driveable_bounds,
    0.01,
    0.01 * M_PI);

  {
    std::vector<ignition::math::Vector3d> points = {
      ignition::math::Vector3d(0., 0., 0.), // 1.1.1
      ignition::math::Vector3d(10., 0., 0.), // 1.1.2
      ignition::math::Vector3d(20., 0., 0.) // 1.1.3
    };
    builder->CreateLaneConnections(1, 1, points);
  }

  {
    std::vector<ignition::math::Vector3d> points = {
      ignition::math::Vector3d(5., 20., 0.), // 2.1.1
      ignition::math::Vector3d(5., 12.5, 0.), // 2.1.2
      ignition::math::Vector3d(5., 5., 0.) // 2.1.3
    };
    builder->CreateLaneConnections(2, 1, points);
  }
  builder->CreateConnection(lane_bounds,
    driveable_bounds,
    ignition::rndf::UniqueId(2, 1, 3),
    ignition::rndf::UniqueId(1, 1, 2));

  auto road_geometry = builder->Build({"T"});
  EXPECT_NE(road_geometry, nullptr);


  EXPECT_EQ(road_geometry->num_junctions(), 5);
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
    EXPECT_EQ(junction->id().id, "j:2_1_1-2_1_2");

    auto segment = junction->segment(0);
    EXPECT_EQ(segment->num_lanes(), 1);
    EXPECT_EQ(segment->id().id, "s:2_1_1-2_1_2");

    auto lane = segment->lane(0);
    EXPECT_EQ(lane->id().id, "l:2_1_1-2_1_2");
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
    EXPECT_EQ(junction->id().id, "j:2_1_2-2_1_3");

    auto segment = junction->segment(0);
    EXPECT_EQ(segment->num_lanes(), 1);
    EXPECT_EQ(segment->id().id, "s:2_1_2-2_1_3");

    auto lane = segment->lane(0);
    EXPECT_EQ(lane->id().id, "l:2_1_2-2_1_3");
    EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kStart), nullptr);
    EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kStart)->size(), 1);
    EXPECT_NE(lane->GetConfluentBranches(api::LaneEnd::kFinish), nullptr);
    EXPECT_EQ(lane->GetConfluentBranches(api::LaneEnd::kFinish)->size(), 1);
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
    EXPECT_EQ(end_branch->GetASide()->size(), 1);
    EXPECT_EQ(end_branch->GetBSide()->size(), 1);
    EXPECT_EQ(end_branch->GetASide()->get(0).lane, lane);
    EXPECT_EQ(end_branch->GetASide()->get(0).end, api::LaneEnd::kFinish);
  }
  {
    auto junction = road_geometry->junction(4);
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

} // rndf
} // maliput
} // drake
