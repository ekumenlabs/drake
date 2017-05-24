#include "drake/automotive/maliput/rndf/builder.h"

#include <cmath>
#include <utility>

#include "drake/automotive/maliput/rndf/branch_point.h"
#include "drake/automotive/maliput/rndf/spline_lane.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace maliput {
namespace rndf {

const double Builder::kWaypointDistancePhase = 0.3;

const double Builder::kLinearStep = 1e-2;

std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>
  DirectedWaypoint::CalculateBoundingBox(
    const std::vector<DirectedWaypoint> &directed_waypoints) {
  std::vector<double> x_coordinates, y_coordinates;
  for (const auto &dw : directed_waypoints) {
    x_coordinates.push_back(dw.Position().X());
    y_coordinates.push_back(dw.Position().Y());
  }
  return std::make_tuple(
    ignition::math::Vector3d(
      *std::min_element(x_coordinates.begin(), x_coordinates.end()),
      *std::min_element(y_coordinates.begin(), y_coordinates.end()),
      0.0),
    ignition::math::Vector3d(
      *std::max_element(x_coordinates.begin(), x_coordinates.end()),
      *std::max_element(y_coordinates.begin(), y_coordinates.end()),
      0.0));
}

void Builder::CreateConnection(
  const double width,
  const ignition::rndf::UniqueId &exit,
  const ignition::rndf::UniqueId &entry) {
  const auto &exit_it = directed_waypoints_.find(exit.String());
  const auto &entry_it = directed_waypoints_.find(entry.String());
  DRAKE_THROW_UNLESS(exit_it != directed_waypoints_.end());
  DRAKE_THROW_UNLESS(entry_it != directed_waypoints_.end());

  std::vector<DirectedWaypoint> control_points = {
    exit_it->second,
    entry_it->second
  };
  std::string key_id = BuildName(exit) + std::string("-") + BuildName(entry);
  CreateLane(key_id, width, control_points);
}

void Builder::SetInvertedLanes(
  std::vector<ConnectedLane> *lanes) {
  const auto base_point = std::get<0>(bounding_box_) -
    ignition::math::Vector3d(1., 1., 0);
  // Get all the momentums and rotate the lanes if necessary
  int first_lane_sign_momentum, other_lane_sign_momentum;
  for (int i = 0; i < static_cast<int>(lanes->size()); i++) {
    double momentum =
      CalculateRNDFLaneMomentum(base_point, lanes->at(i).waypoints());
    if (i == 0) {
      first_lane_sign_momentum = std::copysign(1.0, momentum);
    } else {
      other_lane_sign_momentum = std::copysign(1.0, momentum);
      if (other_lane_sign_momentum != first_lane_sign_momentum) {
        lanes->at(i).set_inverse_direction(true);
      }
    }
  }
}

void Builder::CreateSegmentConnections(
  const uint segment_id,
  std::vector<ConnectedLane> *lanes) {
  // Checks
  DRAKE_THROW_UNLESS(lanes != nullptr);
  DRAKE_THROW_UNLESS(lanes->size() >= 1);
  for (const auto &lane : *lanes) {
    DRAKE_THROW_UNLESS(lane.waypoints.size() >= 2);
  }
  // Build the vector of waypoints
  for (ConnectedLane &lane : *lanes) {
    BuildTangentsForWaypoints(&(lane.waypoints()));
  }

  // Check the momentum that each lane produces and then set the connected lane
  // accordingly
  SetInvertedLanes(lanes);

  // Split the lanes in groups so we get them by lane directions
  std::map<int, std::vector<ConnectedLane>> segments;
  GroupLanesByDirection(lanes, &segments);

  for (auto it : segments) {
    CreateNewControlPointsForLanes(&(it.second));
    // Fill in the map the waypoints
    int num_of_waypoints = it.second[0].waypoints().size();
    int num_of_lanes = it.second.size();
    for (int i = 0; i < (num_of_waypoints - 1); i++) {
      // Find the valid RNDF lane pieces.
      std::vector<int> valid_lane_ids;
      for (int j = 0; j < num_of_lanes; j++) {
        if (it.second[j].waypoints()[i].Id().Valid() &&
          it.second[j].waypoints()[i + 1].Id().Valid()) {
          valid_lane_ids.push_back(j);
        }
      }
      // Order them from right to left
      std::vector<int> ids;
      for (int i = 0; i < static_cast<int>(it.second.size()); i++) {
        ids.push_back(i);
      }
      OrderLaneIds(&(it.second), &valid_lane_ids, 0);
      // Create a segment name
      std::string segment_key_name = std::to_string(segment_id) +
        std::string("-") + std::to_string(it.first) +
        std::string("-") + std::to_string(i);

      // Add the lanes
      for (uint j = 0; j < valid_lane_ids.size(); j++) {
        // Create the lane
        std::vector<DirectedWaypoint> wps;
        wps.push_back(it.second[valid_lane_ids[j]].waypoints()[i]);
        wps.push_back(it.second[valid_lane_ids[j]].waypoints()[i + 1]);
        CreateLane(segment_key_name,
          it.second[valid_lane_ids[j]].width(),
          wps);
        // Add the pair of waypoints to the map
        directed_waypoints_[
          it.second[valid_lane_ids[j]].waypoints()[i].Id().String()] =
            it.second[valid_lane_ids[j]].waypoints()[i];
        directed_waypoints_[
          it.second[valid_lane_ids[j]].waypoints()[i + 1].Id().String()] =
            it.second[valid_lane_ids[j]].waypoints()[i + 1];
      }
    }
  }
}


void Builder::CreateConnectionsForZones(const double width,
  std::vector<DirectedWaypoint> *perimeter_waypoints) {
  DRAKE_THROW_UNLESS(perimeter_waypoints != nullptr);
  DRAKE_THROW_UNLESS(perimeter_waypoints->size() > 0);
  // We calculate the mean coordinates from all the waypoints of the perimeter
  ignition::math::Vector3d center(0., 0., 0.);
  for (const auto &waypoint : *perimeter_waypoints) {
    center += waypoint.Position();
  }
  center /= static_cast<double>(perimeter_waypoints->size());
  // Fill the tangents for the entries and exits
  std::vector<DirectedWaypoint> entries, exits;
  for (DirectedWaypoint &waypoint : *perimeter_waypoints) {
    if (waypoint.is_entry()) {
      waypoint.Tangent() = (center - waypoint.Position()).Normalize();
      entries.push_back(waypoint);
      directed_waypoints_[waypoint.Id().String()] = waypoint;
    } else if (waypoint.is_exit()) {
      waypoint.Tangent() = (waypoint.Position() - center).Normalize();
      exits.push_back(waypoint);
      directed_waypoints_[waypoint.Id().String()] = waypoint;
    }
  }

  for (const auto entry : entries) {
    for (const auto exit : exits) {
      std::vector<DirectedWaypoint> control_points = {
        entry, exit
      };
      std::string key_id = BuildName(exit.Id()) + std::string("-") + BuildName(entry.Id());
      CreateLane(key_id, width, control_points);
    }
  }
}

std::unique_ptr<ignition::math::Spline> Builder::CreateSpline(
  const std::vector<DirectedWaypoint> *waypoints) {
  DRAKE_DEMAND(waypoints != nullptr);
  // We generate the spline
  std::unique_ptr<ignition::math::Spline> spline =
    std::make_unique<ignition::math::Spline>();
  spline->AutoCalculate(true);
  spline->Tension(SplineLane::Tension());
  // Add only valid waypoints
  for (const auto &point : *waypoints) {
    if (!point.Id().Valid())
      continue;
    spline->AddPoint(ignition::math::Vector3d(
      point.Position().X(), point.Position().Y(), 0.));
  }
  return spline;
}

void Builder::GroupLanesByDirection(const std::vector<ConnectedLane> *lanes,
  std::map<int, std::vector<ConnectedLane>> *segments) const {
  // Checks
  DRAKE_DEMAND(lanes != nullptr);
  DRAKE_DEMAND(segments != nullptr);

  // Start grouping the lanes through continous ids
  bool current_inversion = lanes->front().inverse_direction();
  int segment_group = 0;
  for (int i = 0; i < static_cast<int>(lanes->size()); i++) {
    if (current_inversion == lanes->at(i).inverse_direction()) {
      // Create an entry for the map if it doesn't exist
      if (segments->find(segment_group) == segments->end()) {
        (*segments)[current_inversion] = std::vector<ConnectedLane>();
      }
      // Add the lane
      (*segments)[current_inversion].push_back(lanes->at(i));
    } else {
      current_inversion = lanes->at(i).inverse_direction();
      segment_group ++;
      (*segments)[current_inversion] = std::vector<ConnectedLane>();
      (*segments)[current_inversion].push_back(lanes->at(i));
    }
  }
}


void Builder::OrderLaneIds(
    std::vector<ConnectedLane> *lanes,
    std::vector<int> *lane_ids,
    const int index) {
  DRAKE_DEMAND(lane_ids != nullptr);
  DRAKE_DEMAND(lane_ids->size() > 0);
  // Check for a sigle lane road where we don't have to compute anything
  if (lane_ids->size() == 1)
    return;
  // Fill the id_waypoint_list
  std::vector<std::pair<int, DirectedWaypoint>> id_waypoint_list;
  for (int i = 0; i < static_cast<int>(lane_ids->size()); i++) {
    id_waypoint_list.push_back(std::make_pair(lane_ids->at(i),
      lanes->at(i).waypoints().front()));
  }
  // Sort the list
  std::sort(id_waypoint_list.begin(), id_waypoint_list.end(),
    [] (const auto &a, const auto &b) {
      auto v_b_a = b.second.Position() - a.second.Position();
      ignition::math::Vector3d n_a(
        -a.second.Tangent().Y(), a.second.Tangent().X(), 0.0);
      n_a.Normalize();
      if (v_b_a.Dot(n_a) > 0.)
        return true;
      return false;
    });
  lane_ids->clear();
  for (const auto &it : id_waypoint_list) {
    lane_ids->push_back(it.first);
  }
}


void Builder::BuildTangentsForWaypoints(
  std::vector<DirectedWaypoint> *waypoints) {
  DRAKE_DEMAND(waypoints != nullptr);
  // We generate the spline
  std::unique_ptr<ignition::math::Spline> spline = CreateSpline(waypoints);
  uint base_id = 0;
  for (uint i = 0; i < waypoints->size(); i++) {
    if (!waypoints->at(i).Id().Valid()) {
      base_id = i;
      continue;
    } else {
      waypoints->at(i).Tangent() = spline->Tangent(i - base_id);
    }
  }
}


double Builder::ComputeDistance(
  const DirectedWaypoint &base,
  const DirectedWaypoint &destiny) const {
  return (destiny.Position() - base.Position()).Dot(
    ignition::math::Vector3d(base.Tangent()).Normalize());
}

std::vector<int> Builder::GetStartingLaneIds(std::vector<ConnectedLane> *lanes,
  const bool start_check) const {
  DRAKE_DEMAND(lanes != nullptr);
  if (lanes->size() == 1) {
    return {0};
  }
  // We create
  std::vector<std::vector<double>> distances_matrix;
  for (int i = 0; i < static_cast<int>(lanes->size()); i++) {
    distances_matrix.push_back(std::vector<double>());
    for (int j = 0; j < static_cast<int>(lanes->size()); j++) {
      if (i == j) {
        // It is the distance against the same point, no computation is needed
        distances_matrix[i].push_back(0.);
      } else {
        // We compute the distance between respective points
        int first_index, second_index;
        if ((start_check == true && !lanes->at(i).inverse_direction()) ||
          (start_check == false && lanes->at(i).inverse_direction())) {
          first_index = 0;
        } else {
          first_index = lanes->at(i).waypoints().size() - 1;
        }

        if ((start_check == true && !lanes->at(j).inverse_direction()) ||
          (start_check == false && lanes->at(j).inverse_direction())) {
          second_index = 0;
        } else {
          second_index = lanes->at(j).waypoints().size() - 1;
        }

        DirectedWaypoint first_wp(lanes->at(i).waypoints()[first_index]);
        DirectedWaypoint second_wp(lanes->at(j).waypoints()[second_index]);
        if (lanes->at(i).inverse_direction) {
          first_wp.Tangent() = - first_wp.Tangent();
        }
        if (lanes->at(j).inverse_direction) {
          second_wp.Tangent() = - second_wp.Tangent();
        }
        distances_matrix[i].push_back(
          ComputeDistance(first_wp, second_wp));
      }
    }
  }
  // Compute the number of valid distances
  std::vector<std::tuple<int, int>> index_valid_distances;
  int i = 0;
  for (const auto distances : distances_matrix) {
    int number_of_valid_distances = 0;
    for (const auto d : distances) {
      if (start_check) {
        if (d > 0.0) {
          number_of_valid_distances++;
        }
      } else {
        if (d < 0.0) {
          number_of_valid_distances++;
        }
      }
    }
    index_valid_distances.push_back(std::make_tuple(i,
      number_of_valid_distances));
    i++;
  }
  // Sort the tuple vector by increasing number of valid distances.
  // This gives us the first values of the vectors with the lane ids with
  // waypoints that appear first
  std::sort(std::begin(index_valid_distances), std::end(index_valid_distances),
    [](auto const &t_a, auto const &t_b) {
      return std::get<1>(t_a) > std::get<1>(t_b);
    });

  // Create a vector with all the lane ids that appear first and then return it
  std::vector<int> ids;
  for (const auto id_zeros : index_valid_distances) {
    if (std::get<1>(id_zeros) == std::get<1>(index_valid_distances[0]))
      ids.push_back(std::get<0>(id_zeros));
  }
  return ids;
}

std::vector<int> Builder::GetInitialLaneToProcess(
  std::vector<ConnectedLane> *lanes,
  const int index) {
  DRAKE_DEMAND(lanes != nullptr);
  // Compute the distance matrix
  std::vector<std::vector<double>> distances_matrix;
  for (uint i = 0; i < lanes->size(); i++) {
    distances_matrix.push_back(std::vector<double>());
    for (uint j = 0; j < lanes->size(); j++) {
      if (i == j) {
        // It is the distance against the same point
        distances_matrix[i].push_back(-1.);
      } else if (index >= static_cast<int>(lanes->at(i).waypoints().size()) ||
        index >= static_cast<int>(lanes->at(j).waypoints().size())) {
        // In this case it's none sense to compute the distance as one of the
        // lanes is shorter that the other at least in terms of waypoints.
        distances_matrix[i].push_back(-2.);
      } else {
        // We compute the distance and in case it's less than zero we
        // truncate it to zero to set mean that this point is further
        // than the other.
        distances_matrix[i].push_back(
          ComputeDistance(lanes->at(i).waypoints()[index], lanes->at(j).waypoints()[index]));
      }
    }
  }
  // Compute the number of valid distances
  std::vector<std::tuple<int, int>> index_valid_distances;
  int i = 0;
  for (const auto distances : distances_matrix) {
    int number_of_valid_distances = 0;
    for (const auto d : distances) {
      if (d > kWaypointDistancePhase) {
        number_of_valid_distances++;
      }
    }
    index_valid_distances.push_back(std::make_tuple(i,
      number_of_valid_distances));
    i++;
  }

  // Sort the tuple vector by increasing number of valid distances.
  // This gives us the first values of the vectors with the lane ids with
  // waypoints that appear first
  std::sort(std::begin(index_valid_distances), std::end(index_valid_distances),
    [](auto const &t_a, auto const &t_b) {
      return std::get<1>(t_a) > std::get<1>(t_b);
    });

  // Create a vector with all the lane ids that appear first and then return it
  std::vector<int> ids;
  for (const auto id_zeros : index_valid_distances) {
    if (std::get<1>(id_zeros) == std::get<1>(index_valid_distances[0]))
      ids.push_back(std::get<0>(id_zeros));
  }
  // In case we have all the lane ids, no one is first, all are on the same
  // line.
  if (ids.size() == index_valid_distances.size()) {
    ids.clear();
  }
  return ids;
}

void Builder::AddWaypointIfNecessary(const std::vector<int> &ids,
  std::vector<ConnectedLane> *lanes,
  const int index) {
  DRAKE_DEMAND(lanes != nullptr);

  for (int i = 0; i < static_cast<int>(lanes->size()); i++) {
    ConnectedLane &lane = lanes->at(i);
    // Check id the id is in the ids (first-to-appear vector)
    if (std::find(ids.begin(), ids.end(), i) != ids.end()) {
      continue;
    }

    if (ids.size() == 0 && static_cast<int>(lane.waypoints().size()) > index) {
      continue;
    }
    if (static_cast<int>(lane.waypoints().size()) <= index) {
      // No more lane, so we add a new blank directed waypoint
      lane.waypoints.push_back(DirectedWaypoint());
    } else if (lane.waypoints()[index].Id().Z() == 1) {
      // As the waypoint is first one, we need to add one blank at the
      // beginning.
      std::vector<DirectedWaypoint> new_lane;
      new_lane.insert(new_lane.begin(), lane.waypoints.begin(), lane.waypoints.begin() + index);
      new_lane.push_back(DirectedWaypoint());
      new_lane.insert(new_lane.end(), lane.waypoints.begin() + index, lane.waypoints.end());
      lane.waypoints = new_lane;
    } else {
      // Here we need to add a waypoint to the respective position of the side
      // lane.
      // Build the spline
      auto spline = CreateSpline(&(lanes->at(i).waypoints));
      std::unique_ptr<ArcLengthParameterizedSpline> arc_lenght_param_spline =
        std::make_unique<ArcLengthParameterizedSpline>(std::move(spline),
          linear_tolerance_);
      double s = arc_lenght_param_spline->FindClosestPointTo(
        lanes->at(ids[0]).waypoints()[index].Position(), kLinearStep);
      // Build the waypoint
      DirectedWaypoint new_wp(
        ignition::rndf::UniqueId(lanes->at(i).waypoints()[index-1].Id().X(),
          lanes->at(i).waypoints()[index-1].Id().Y(),
          lanes->at(i).waypoints.size() + 1),
        arc_lenght_param_spline->InterpolateMthDerivative(0, s),
        false,
        false,
        arc_lenght_param_spline->InterpolateMthDerivative(1, s));
      // Insert it into the vector.
      std::vector<DirectedWaypoint> new_lane;
      new_lane.insert(new_lane.begin(), lane.waypoints.begin(), lane.waypoints.begin() + index);
      new_lane.push_back(new_wp);
      new_lane.insert(new_lane.end(), lane.waypoints.begin() + index, lane.waypoints.end());
      lane.waypoints = new_lane;
    }
  }
}

void Builder::CreateNewControlPointsForLanes(
  std::vector<ConnectedLane> *lanes) {
  DRAKE_DEMAND(lanes != nullptr);
  // Load the tangents for each lane waypoints
  for (ConnectedLane &lane : *lanes) {
    BuildTangentsForWaypoints(&(lane.waypoints));
  }
  int i = 0;

  bool should_continue = true;
  while (should_continue) {
    // Get the lanes ids which appear first
    auto ids = GetInitialLaneToProcess(lanes, i);
    // We need to check if we need to create a waypoint for the other lane and
    // add them if necessary.
    AddWaypointIfNecessary(ids, lanes, i);
    // Check if index is bounded to any of the lanes
    i++;
    should_continue = false;
    for (const auto lane : *lanes) {
      if (i < static_cast<int>(lane.waypoints.size())) {
        should_continue = true;
        break;
      }
    }
  }
}


ignition::math::Vector3d Builder::ConstructPointForLane(
  const DirectedWaypoint &base, const DirectedWaypoint &other_lane_base) const {
  const ignition::math::Vector3d g_l_1(
    base.Position().X(), base.Position().Y(), 0.);
  ignition::math::Vector3d t_1 = base.Tangent();
  t_1.Normalize();
  const ignition::math::Vector3d n_1(-t_1.Y(), t_1.X(), 0.);

  const ignition::math::Vector3d g_l_0_a(
    other_lane_base.Position().X(), other_lane_base.Position().Y(), 0.);
  ignition::math::Vector3d t_0 = other_lane_base.Tangent();
  t_0.Normalize();

  auto delta =
    (g_l_0_a - g_l_1).Cross(n_1).Dot(t_0.Cross(n_1)) /
    t_0.Cross(n_1).SquaredLength() * (-1);
  return g_l_0_a + t_0 * delta;
}

std::unique_ptr<const api::RoadGeometry> Builder::Build(
    const api::RoadGeometryId& id) {
  std::unique_ptr<std::map<std::string, BranchPoint *>> branch_point_map =
    std::make_unique<std::map<std::string, BranchPoint *>>();
  auto road_geometry = std::make_unique<RoadGeometry>(
    id,
    linear_tolerance_,
    angular_tolerance_);

  // Build a lane for each connection and create their respective branch points
  for (const auto &it_connection : connections_) {
    // Build a junction and the segment for the lanes
    Junction* junction =
      road_geometry->NewJunction(
        {std::string("j:") + it_connection.first});

    DRAKE_DEMAND(junction != nullptr);
    Segment *segment = junction->NewSegment(
      {std::string("s:") + it_connection.first});

    DRAKE_DEMAND(segment != nullptr);

    for (const auto &connection : it_connection.second) {
      // Create the lane
      drake::maliput::rndf::Lane *lane =
        BuildConnection(segment, connection.get());

      DRAKE_DEMAND(lane != nullptr);
      // Build the branch points of necessary for the lane
      BuildOrUpdateBranchpoints(connection.get(),
        lane,
        branch_point_map.get(),
        road_geometry.get());
    }
  }
  // Make sure we didn't screw up!
  std::vector<std::string> failures = road_geometry->CheckInvariants();
  for (const auto& s : failures) {
    drake::log()->error(s);
  }
  DRAKE_THROW_UNLESS(failures.size() == 0);

  return std::move(road_geometry);
}

void Builder::CreateLane(
  const std::string &key_id,
  const double width,
  const std::vector<DirectedWaypoint> &control_points) {
  DRAKE_DEMAND(control_points.size() > 1);
  const auto &start_id = control_points.front().Id();
  const auto &end_id = control_points.back().Id();
  const std::string name =
    BuildName(start_id) + std::string("-") + BuildName(end_id);
  if (connections_.find(key_id) == connections_.end()) {
    connections_[key_id] = std::vector<std::unique_ptr<Connection>>();
  }
  connections_[key_id].push_back(std::make_unique<Connection>(
    name,
    control_points,
    width));
}

namespace {
// Determine the heading (in xy-plane) along the centerline when
// travelling towards/into the lane, from the specified end.
double HeadingIntoLane(const api::Lane* const lane,
                       const api::LaneEnd::Which end) {
  switch (end) {
    case api::LaneEnd::kStart: {
      return lane->GetOrientation({0., 0., 0.}).yaw();
    }
    case api::LaneEnd::kFinish: {
      return lane->GetOrientation({lane->length(), 0., 0.}).yaw() + M_PI;
    }
    default: { DRAKE_ABORT(); }
  }
}
}  // namespace


void Builder::AttachLaneEndToBranchPoint(
    Lane* lane,
    const api::LaneEnd::Which end,
    BranchPoint *branch_point) {
  DRAKE_DEMAND(lane != nullptr);
  DRAKE_DEMAND(branch_point != nullptr);
  // Tell the lane about its branch-point.
  switch (end) {
    case api::LaneEnd::kStart: {
      lane->SetStartBp(branch_point);
      break;
    }
    case api::LaneEnd::kFinish: {
      lane->SetEndBp(branch_point);
      break;
    }
    default: {
      DRAKE_ABORT();
    }
  }
  // Now, tell the branch-point about the lane.
  //
  // Is this the first lane-end added to the branch-point?
  // If so, just stick it on A-Side.
  // (NB: We just test size of A-Side, since A-Side is always populated first.)
  if (branch_point->GetASide()->size() == 0) {
    branch_point->AddABranch({lane, end});
    return;
  }
  // Otherwise, assess if this new lane-end is parallel or anti-parallel to
  // the first lane-end.  Parallel: go to same, A-side; anti-parallel:
  // other, B-side.  Do this by examining the dot-product of the heading
  // vectors (rather than goofing around with cyclic angle arithmetic).
  const double new_h = HeadingIntoLane(lane, end);
  const api::LaneEnd old_le = branch_point->GetASide()->get(0);
  const double old_h = HeadingIntoLane(old_le.lane, old_le.end);
  if (((std::cos(new_h) * std::cos(old_h)) +
       (std::sin(new_h) * std::sin(old_h))) > 0.) {
    branch_point->AddABranch({lane, end});
  } else {
    branch_point->AddBBranch({lane, end});
  }
}

Lane* Builder::BuildConnection(
  Segment *segment,
  const Connection *connection) {
  DRAKE_DEMAND(segment != nullptr);
  // Create a new segment and assign a lane to it
  Lane* lane{};
  api::LaneId lane_id{std::string("l:") + connection->id()};

  std::vector<std::tuple<
    ignition::math::Vector3d,
    ignition::math::Vector3d>> points_tangents;
  for (const auto& directed_waypoint : connection->waypoints()) {
    points_tangents.push_back(std::make_tuple(
      directed_waypoint.Position(), directed_waypoint.Tangent()));
  }
  lane = segment->NewSplineLane(lane_id,
    points_tangents,
    connection->width());

  return lane;
}

void Builder::BuildOrUpdateBranchpoints(
  Connection *connection,
  Lane *lane,
  std::map<std::string, BranchPoint*> *branch_point_map,
  RoadGeometry *road_geometry) {
  DRAKE_DEMAND(connection != nullptr);
  DRAKE_DEMAND(lane != nullptr);
  DRAKE_DEMAND(branch_point_map != nullptr);
  DRAKE_DEMAND(road_geometry != nullptr);
  // First we care about the start of the branch point
  BranchPoint* bp{nullptr};
  auto it = branch_point_map->find(connection->start().Id().String());
  if (it == branch_point_map->end()) {
    bp = road_geometry->NewBranchPoint(
      {"bp:" + std::to_string(road_geometry->num_branch_points())});
    DRAKE_DEMAND(bp != nullptr);
    (*branch_point_map)[connection->start().Id().String()] = bp;
  } else {
    bp = it->second;
  }
  AttachLaneEndToBranchPoint(lane, api::LaneEnd::kStart, bp);
  // Now, it's the turn of the end lane end
  bp = nullptr;
  it = branch_point_map->find(connection->end().Id().String());
  if (it == branch_point_map->end()) {
    bp = road_geometry->NewBranchPoint(
      {"bp:" + std::to_string(road_geometry->num_branch_points())});
    DRAKE_DEMAND(bp != nullptr);
    (*branch_point_map)[connection->end().Id().String()] = bp;
  } else {
    bp = it->second;
  }
  AttachLaneEndToBranchPoint(lane, api::LaneEnd::kFinish, bp);
}

double Builder::CalculateMomentum(
  const ignition::math::Vector3d &point,
  const DirectedWaypoint &wp) {
  const auto v_wp_point = wp.Position() - point;
  const auto result = wp.Tangent().Cross(v_wp_point);
  // As all the points should be on the x-y plane, the result should have all
  // its value on the z component, so we get it.
  return result.Z();
}
double Builder::CalculateRNDFLaneMomentum(
  const ignition::math::Vector3d &base_point,
  const std::vector<DirectedWaypoint> &wps) {
  double momentum = 0.;
  for (const auto &wp : wps) {
    momentum += CalculateMomentum(base_point, wp);
  }
  return momentum;
}

std::string Builder::BuildName(const uint segment_id,
  const uint lane_id) const {
  return std::to_string(segment_id) + "_" +
    std::to_string(lane_id) + "_";
}

std::string Builder::BuildName(const uint segment_id,
  const uint lane_id,
  const uint waypoint_id) const {
  return std::to_string(segment_id) + "_" +
    std::to_string(lane_id) + "_" +
    std::to_string(waypoint_id);
}

std::string Builder::BuildName(const ignition::rndf::UniqueId &id) const {
  return std::to_string(id.X()) + "_" +
    std::to_string(id.Y()) + "_" +
    std::to_string(id.Z());
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
