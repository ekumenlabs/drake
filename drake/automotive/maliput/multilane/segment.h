#pragma once

#include <memory>
#include <utility>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/multilane/cubic_polynomial.h"
#include "drake/automotive/maliput/multilane/lane.h"
#include "drake/automotive/maliput/multilane/road_curve.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace multilane {

class ArcLane;
class LineLane;

/// An api::Segment implementation.
class Segment : public api::Segment {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Segment)

  /// Constructs a new Segment.
  ///
  /// The Segment is not fully initialized until NewLane() is called exactly
  /// once. @p junction must remain valid for the lifetime of this class.
  /// @param id ID of the segment.
  /// @param junction Parent junction.
  /// @param road_curve A curve that defines the reference trajectory over the
  /// segment. A child Lane object will be constructed from an offset of the
  /// road_curve's reference curve.
  Segment(const api::SegmentId& id,
          api::Junction* junction,
          std::unique_ptr<RoadCurve> road_curve,
          int num_lanes,
          double r0, double r_spacing, double width,
          const api::HBounds& elevation_bounds)
      : id_(id), junction_(junction), road_curve_(std::move(road_curve)),
      num_lanes_(num_lanes), r0_(r0), r_spacing_(r_spacing), width_(width),
      lane_width_(width / static_cast<double>(num_lanes)),
      elevation_bounds_(elevation_bounds) {
    DRAKE_DEMAND(road_curve_.get() != nullptr);
    DRAKE_DEMAND(num_lanes_ > 0);
    DRAKE_DEMAND(width_ > 0);
  }

  /// Creates a new Lane object.
  /// This method should be called only once in the lifespan of the object. The
  /// Segment class only supports one Lane.
  /// @param id Lane's ID.
  /// @param lane_bounds Lateral extents of the Lane's frame in which the car is
  /// supposed to be inside the lane.
  /// @param driveable_bounds Lateral extents of the Lane's frame in which the
  /// car is supposed to be inside the segment.
  /// @param elevation_bounds Height extents of the Lane's frame.
  /// @param r0 Lateral displacement of the Lane with respect to segment
  /// RoadCurve's reference curve.
  /// @return A Lane object.
  Lane* NewLane(const api::LaneId& id);

  ~Segment() override = default;

 private:
  const api::SegmentId do_id() const override { return id_; }

  const api::Junction* do_junction() const override;

  int do_num_lanes() const override { return lanes_.size(); }

  const api::Lane* do_lane(int index) const override;

  // Segment's ID.
  api::SegmentId id_;
  // Parent junction.
  api::Junction* junction_{};
  // Child Lanes vectors.
  std::vector<std::unique_ptr<Lane>> lanes_;
  // Reference trajectory over the Segment's surface.
  std::unique_ptr<RoadCurve> road_curve_;

  const int num_lanes_{};

  const double r0_{};

  const double r_spacing_{};

  const double width_;

  const double lane_width_;

  const api::RBounds driveable_bounds_;

  const api::HBounds elevation_bounds_;
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
