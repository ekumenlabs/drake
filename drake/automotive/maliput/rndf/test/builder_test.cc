#include <iostream>
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
/*
  EXPECT_THROW(std::make_unique<Builder>(
    api::RBounds(-15. / 2., 5. / 2.),
    api::RBounds(-10/ 2., 10. / 2.),
    0.01,
    0.01 * M_PI), std::runtime_error);
*/
/*
  EXPECT_THROW(std::make_unique<Builder>(
    api::RBounds(-5. / 2., 15. / 2.),
    api::RBounds(-10/ 2., 10. / 2.),
    0.01,
    0.01 * M_PI), std::runtime_error);
*/
}


} // rndf
} // maliput
} // drake
