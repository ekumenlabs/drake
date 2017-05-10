#pragma once

#include <memory>
#include <vector>
#include <tuple>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/rndf/lane.h"
#include "drake/common/drake_copyable.h"

#include "ignition/math/Vector3.hh"

namespace drake {
namespace maliput {
namespace rndf {

class SplineLane;

/// An api::Segment implementation.
class Segment : public api::Segment {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Segment)

  /// Constructs a new Segment.
  ///
  /// The Segment is not fully initialized until NewSplineLane() method is
  /// called.
  /// Right now, we don't have support for multilane, so just call
  ///  NewSplineLane() once in the lifespan of the object.
  /// @p junction must remain valid for the lifetime of this class.
  Segment(const api::SegmentId& id, api::Junction* junction)
      : id_(id), junction_(junction) {}

  /// Gives the segment a newly constructed SplineLane.
  /// @throw An exception if the object already has created a lane.
  SplineLane* NewSplineLane(const api::LaneId& id,
                      const std::vector<std::tuple<ignition::math::Vector3d,
                        ignition::math::Vector3d>> &control_points,
                      const api::RBounds& lane_bounds,
                      const api::RBounds& driveable_bounds);

  ~Segment() override = default;

 private:
  const api::SegmentId do_id() const override { return id_; }

  const api::Junction* do_junction() const override;

  int do_num_lanes() const override {
    return lanes_.size();
  }

  const api::Lane* do_lane(int index) const override;

  api::SegmentId id_;
  api::Junction* junction_{};
  std::vector<std::unique_ptr<Lane>> lanes_;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
