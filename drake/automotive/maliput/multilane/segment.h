#pragma once

#include <memory>
#include <utility>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/multilane/lane.h"
#include "drake/automotive/maliput/multilane/road_curve.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace multilane {

class Lane;

class Segment : public api::Segment {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Segment)

  Segment(const api::SegmentId& id,
          api::Junction* junction,
          std::unique_ptr<RoadCurve> road_curve)
      : id_(id), junction_(junction), road_curve_(std::move(road_curve)) {
    DRAKE_THROW_UNLESS(road_curve_.get() != nullptr);
  }

  ~Segment() override = default;

  Lane* NewLane(api::LaneId id,
                const api::RBounds& lane_bounds,
                const api::RBounds& driveable_bounds,
                const api::HBounds& elevation_bounds);

 private:
  const api::SegmentId do_id() const override { return id_; }

  const api::Junction* do_junction() const override;

  int do_num_lanes() const override { return 1; }

  const api::Lane* do_lane(int index) const override;

  api::SegmentId id_;
  api::Junction* junction_{};
  std::unique_ptr<Lane> lane_;
  std::unique_ptr<RoadCurve> road_curve_;
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
