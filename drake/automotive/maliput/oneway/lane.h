#pragma once

#include <memory>
#include <vector>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"

#include <ignition/math.hh>

namespace drake {
namespace maliput {
namespace oneway {

// Forward declarations.
class BranchPoint;
class Segment;

/**
 Dragway's implementation of api::Lane. The lane is flat with a height of
 zero.

 The following lane is implemented:

  <pre>
                     lane_bounds
        |<------------------------------->|
                 driveable_bounds
    |<--------------------------------------->|

    -------------------------------------------  ———  s = length()
    |                    :                    |   ^
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |
    |                    :                    |   |           world
    |                    :                    |   |           frame:
    |                    :                    |   |
    |                    :                    |   |                X
    |                    :                    |   |                ^
    |                    :                    |   |                |
    |                    :                    |   v                |
    ---------------------o---------------------  ———  s = 0   Y <--o

            r_max                r_min
    |<-------------------|------------------->|

                                            y_offset
                         |<----------------------------------------|
  </pre>

  The lane's frame is defined by three coordinates: (`s`, `r`, `h`). Coordinate
  `s` is between zero and `length()`. It specifies the longitudinal traversal of
  the lane. Coordinate `r` is a value between `r_min` and `r_max`. It specifies
  the lateral traversal at a particular `s`. Coordinate `h` specifies the
  height above the lane's surface at a particular `s` and `r` (the lane's
  surface itself is always at `h = 0`). Since Dragway lanes are flat and level,
  `z = h` for all values of `s` and `r` and, in the Dragway's case, `z = 0` for
  the surface itself. The origin of the lane's frame is defined by the `o` along
  the above-shown `s = 0` line.
**/
class Lane final : public api::Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Lane)

  /// Constructs a dragway Lane.
  ///
  /// @param segment The Segment to which this lane belongs.
  ///
  /// @param id the lane ID. This can be any user-defined value.
  ///
  /// @param length The total length of the lane.
  ///
  /// @param bounds nominal bounds of the lane, uniform along the entire
  ///        reference path, which are alos the @p driveable_bounds.
  Lane(const Segment* segment, const api::LaneId& id,
       double length, const api::RBounds& bounds);

  ~Lane() final = default;

 private:
  const api::LaneId do_id() const final { return id_; }

  const api::Segment* do_segment() const final;

  int do_index() const final { return 0; }

  const api::Lane* do_to_left() const final { return nullptr; }

  const api::Lane* do_to_right() const final { return nullptr; }

  const api::BranchPoint* DoGetBranchPoint(
      const api::LaneEnd::Which which_end) const final;

  const api::LaneEndSet* DoGetConfluentBranches(
      const api::LaneEnd::Which which_end) const final;

  const api::LaneEndSet* DoGetOngoingBranches(
      const api::LaneEnd::Which which_end) const final;

  std::unique_ptr<api::LaneEnd> DoGetDefaultBranch(
      const api::LaneEnd::Which which_end) const final;

  double do_length() const final { return length_; }

  api::RBounds do_lane_bounds(double) const final;

  api::RBounds do_driveable_bounds(double) const final;

  api::LanePosition DoEvalMotionDerivatives(
      const api::LanePosition& position,
      const api::IsoLaneVelocity& velocity) const final;

  api::GeoPosition DoToGeoPosition(const api::LanePosition& lane_pos) const
      final;

  api::Rotation DoGetOrientation(const api::LanePosition& lane_pos) const
      final;

  api::LanePosition DoToLanePosition(const api::GeoPosition& geo_pos,
                                     api::GeoPosition* nearest_point,
                                     double* distance) const final;

  double ComputeLength(
    const std::vector<ignition::math::Vector3d> &points,
    std::vector<double> *lengths = nullptr);

  std::vector<ignition::math::Vector3d> InterpolateRoad(
    const std::vector<ignition::math::Vector3d> &points,
    const double minimum_distance);

  const Segment* segment_{};  // The segment to which this lane belongs.
  const api::LaneId id_;
  //const double length_{};
  double length_{};
  const api::RBounds bounds_;

  // The following variable is actually `const` after construction.
  std::unique_ptr<api::BranchPoint> branch_point_;

  ignition::math::Spline spline_;
  std::vector<ignition::math::Vector3d> points_;
  std::vector<double> lengths_;
};


}  // namespace oneway
}  // namespace maliput
}  // namespace drake
