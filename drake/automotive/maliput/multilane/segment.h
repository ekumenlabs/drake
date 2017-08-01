#pragma once

#include <memory>
#include <utility>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/multilane/lane.h"
#include "drake/automotive/maliput/multilane/segment_geometry.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace multilane {

class Lane;

/// An R^3 rotation parameterized by roll, pitch, yaw.
///
/// This effects a compound rotation around space-fixed x-y-z axes:
///
///   Rot3(yaw,pitch,roll) * V = RotZ(yaw) * RotY(pitch) * RotX(roll) * V
class Rot3 {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Rot3)

  Rot3(double roll, double pitch, double yaw) : rpy_(roll, pitch, yaw) {}

  /// Applies the rotation to a 3-vector.
  Vector3<double> apply(const Vector3<double>& in) const {
    return math::rpy2rotmat(rpy_) * in;
  }

  double yaw() const { return rpy_(2); }
  double pitch() const { return rpy_(1); }
  double roll() const { return rpy_(0); }

 private:
  Eigen::Matrix<double, 3, 1, Eigen::DontAlign> rpy_;
};

class Segment : public api::Segment {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Segment)

  Segment(const api::SegmentId& id,
          api::Junction* junction,
          std::unique_ptr<SegmentGeometry> geometry)
      : id_(id), junction_(junction), geometry_(std::move(geometry)) {
    DRAKE_THROW_UNLESS(geometry_.get() != nullptr);
  }

  ~Segment() override = default;

  Lane* NewLane(api::LaneId id,
                const api::RBounds& lane_bounds,
                const api::RBounds& driveable_bounds,
                const api::HBounds& elevation_bounds);

  const SegmentGeometry& get_reference_geometry() const {
    return *geometry_;
  }

  api::GeoPosition DoToGeoPosition(
      const api::LanePosition& lane_pos) const;

  api::Rotation DoGetOrientation(
      const api::LanePosition& lane_pos) const;

  api::LanePosition DoEvalMotionDerivatives(
      const api::LanePosition& position,
      const api::IsoLaneVelocity& velocity) const;

  api::LanePosition DoToLanePosition(
      const api::GeoPosition& geo_position,
      const api::RBounds& driveable_bounds,
      const api::HBounds& elevation_bounds) const;

  double TrajectoryLength() const {
    return geometry_->trajectory_length();
  }

 private:
  const api::SegmentId do_id() const override { return id_; }

  const api::Junction* do_junction() const override;

  int do_num_lanes() const override { return 1; }

  const api::Lane* do_lane(int index) const override;

  // The geometry here revolves around an abstract "world function"
  //
  //    W: (p,r,h) --> (x,y,z)
  //
  // which maps a `Lane`-frame position to its corresponding representation in
  // world coordinates (with the caveat that instead of the lane's native
  // longitudinal coordinate 's', the reference curve parameter 'p' is used).
  //
  // W is derived from the three functions which define the lane:
  //
  //   G: p --> (x,y)     = the reference curve, a.k.a. xy_of_p()
  //   Z: p --> z / q_max = the elevation function, a.k.a. elevation_
  //   Θ: p --> θ / q_max = the superelevation function, a.k.a. superelevation_
  //
  // as:
  //
  //   (x,y,z) = W(p,r,h) = (G(p), Z(p)) + R_αβγ*(0,r,h)
  //
  // where R_αβγ is the roll/pitch/yaw rotation given by angles:
  //
  //   α = Θ(p)
  //   β = -atan(dZ/dp) at p
  //   γ = atan2(dG_y/dp, dG_x/dp) at p
  //
  // (R_αβγ is essentially the orientation of the (s,r,h) `Lane`-frame
  // at a location (s,0,0) on the reference-line of the lane.  However, it
  // is *not* necessarily the correct orientation at r != 0 or h != 0.)
  //
  // The following methods compute various terms derived from the above which
  // see repeated use.

  // Returns the parametric position p along the reference curve corresponding
  // to longitudinal position @p s along the lane.
  double p_from_s(const double s) const;

  // Returns the rotation R_αβγ, evaluated at @p p along the reference curve.
  Rot3 Rabg_of_p(const double p) const;

  // Returns W' = ∂W/∂p, the partial differential of W with respect to p,
  // evaluated at @p p, @p r, @p h.
  //
  // (@p Rabg must be the result of Rabg_of_p(p) --- passed in here to
  // avoid recomputing it.)
  Vector3<double> W_prime_of_prh(const double p, const double r, const double h,
                    const Rot3& Rabg) const;

  // Returns the s-axis unit-vector, expressed in the world frame,
  // of the (s,r,h) `Lane`-frame (with respect to the world frame).
  //
  // (@p Rabg must be the result of Rabg_of_p(p) --- passed in here to
  // avoid recomputing it.)
  Vector3<double> s_hat_of_prh(const double p, const double r, const double h,
                  const Rot3& Rabg) const;

  // Returns the r-axis unit-vector, expressed in the world frame,
  // of the (s,r,h) `Lane`-frame (with respect to the world frame).
  //
  // (@p Rabg must be the result of Rabg_of_p(p) --- passed in here to
  // avoid recomputing it.)
  Vector3<double> r_hat_of_Rabg(const Rot3& Rabg) const;

  api::SegmentId id_;
  api::Junction* junction_{};
  std::unique_ptr<Lane> lane_;
  std::unique_ptr<SegmentGeometry> geometry_;
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
