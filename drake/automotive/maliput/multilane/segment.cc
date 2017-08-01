#include "drake/automotive/maliput/multilane/segment.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace multilane {

Lane* Segment::NewLane(api::LaneId id,
                       const api::RBounds& lane_bounds,
                       const api::RBounds& driveable_bounds,
                       const api::HBounds& elevation_bounds) {
  DRAKE_DEMAND(lane_.get() == nullptr);
  lane_ = std::make_unique<Lane>(
      id, this, lane_bounds, driveable_bounds, elevation_bounds);
  return lane_.get();
}

api::GeoPosition Segment::DoToGeoPosition(
    const api::LanePosition& lane_pos) const {
  // Recover parameter p from arc-length position s.
  const double p = p_from_s(lane_pos.s());

  const double p_scale = geometry_->length();
  // Calculate z (elevation) of (s,0,0);
  const double z = geometry_->elevation().f_p(p) * p_scale;
  // Calculate x,y of (s,0,0).
  const Vector2<double> xy = geometry_->xy_of_p(p);
  // Calculate orientation of (s,r,h) basis at (s,0,0).
  const Rot3 ypr = Rabg_of_p(p);
  // Rotate (0,r,h) and sum with mapped (s,0,0).
  const Vector3<double> xyz = ypr.apply({0., lane_pos.r(), lane_pos.h()}) +
                 Vector3<double>(xy.x(), xy.y(), z);
  return {xyz.x(), xyz.y(), xyz.z()};
}

api::Rotation Segment::DoGetOrientation(
    const api::LanePosition& lane_pos) const {
  // Recover linear parameter p from arc-length position s.
  const double p = p_from_s(lane_pos.s());
  const double r = lane_pos.r();
  const double h = lane_pos.h();
  // Calculate orientation of (s,r,h) basis at (s,0,0).
  const Rot3 Rabg = Rabg_of_p(p);

  // Calculate s,r basis vectors at (s,r,h)...
  const Vector3<double> s_hat = s_hat_of_prh(p, r, h, Rabg);
  const Vector3<double> r_hat = r_hat_of_Rabg(Rabg);
  // ...and then derive orientation from those basis vectors.
  //
  // (s_hat  r_hat  h_hat) is an orthonormal basis, obtained by rotating the
  // (x_hat  y_hat  z_hat) basis by some R-P-Y rotation; in this case, we know
  // the value of (s_hat  r_hat  h_hat) (w.r.t. 'xyz' world frame), so we are
  // trying to recover the roll/pitch/yaw.  Since (x_hat  y_hat  z_hat) is an
  // identity matrix (e.g., x_hat = column vector (1, 0, 0), etc), then
  // (s_hat  r_hat  h_hat) equals the R-P-Y matrix itself.
  // If we define a = alpha = roll, b = beta = pitch, g = gamma = yaw,
  // then s_hat is the first column of the rotation, r_hat is the second:
  //   s_hat = (cb * cg, cb * sg, - sb)
  //   r_hat = (- ca * sg + sa * sb * cg, ca * cg + sa * sb * sg, sa * cb)
  // We solve the above for a, b, g.
  const double gamma = std::atan2(s_hat.y(),
                                  s_hat.x());
  const double beta = std::atan2(-s_hat.z(),
                                 Vector2<double>(s_hat.x(), s_hat.y()).norm());
  const double cb = std::cos(beta);
  const double alpha =
      std::atan2(r_hat.z() / cb,
                 ((r_hat.y() * s_hat.x()) - (r_hat.x() * s_hat.y())) / cb);
  return api::Rotation::FromRpy(alpha, beta, gamma);
}

api::LanePosition Segment::DoEvalMotionDerivatives(
    const api::LanePosition& position,
    const api::IsoLaneVelocity& velocity) const {

  const double p = p_from_s(position.s());
  const double r = position.r();
  const double h = position.h();

  // TODO(maddog@tri.global)  When s(p) is integrated properly, do this:
  //                          const double g_prime = elevation().fdot_p(p);
  const double g_prime = geometry_->elevation().fake_gprime(p);

  const Rot3 R = Rabg_of_p(p);
  const Vector3<double> W_prime = W_prime_of_prh(p, r, h, R);

  // The definition of path-length of a path along σ yields dσ = |∂W/∂p| dp.
  // Similarly, path-length s along the road at r = 0 is related to the
  // elevation by ds = p_scale * sqrt(1 + g'^2) dp.  Chaining yields ds/dσ:
  const double p_scale = geometry_->length();
  const double ds_dsigma =
      p_scale * std::sqrt(1 + (g_prime * g_prime)) / W_prime.norm();

  return api::LanePosition(ds_dsigma * velocity.sigma_v,
                           velocity.rho_v,
                           velocity.eta_v);
}

api::LanePosition Segment::DoToLanePosition(
    const api::GeoPosition& geo_position,
    const api::RBounds& driveable_bounds,
    const api::HBounds& elevation_bounds) const {
  const std::pair<double, double> lateral_bounds =
      std::make_pair(driveable_bounds.min(), driveable_bounds.max());
  const std::pair<double, double> height_bounds =
      std::make_pair(elevation_bounds.min(), elevation_bounds.max());
  return api::LanePosition::FromSrh(geometry_->ToCurveFrame(geo_position.xyz(),
                                                            lateral_bounds,
                                                            height_bounds));
}

double Segment::p_from_s(double s) const {
  const double p_scale = geometry_->length();
  return geometry_->elevation().p_s(s / p_scale);
}

Rot3 Segment::Rabg_of_p(double p) const {
  const double p_scale = geometry_->length();
  return Rot3(geometry_->superelevation().f_p(p) * p_scale,
              -std::atan(geometry_->elevation().f_dot_p(p)),
              geometry_->heading_of_p(p));
}

Vector3<double> Segment::W_prime_of_prh(const double p, const double r,
    const double h, const Rot3& Rabg) const {
  const Vector2<double> G_prime = geometry_->xy_dot_of_p(p);
  const double g_prime = geometry_->elevation().f_dot_p(p);

  const Rot3& R = Rabg;
  const double alpha = R.roll();
  const double beta = R.pitch();
  const double gamma = R.yaw();

  const double ca = std::cos(alpha);
  const double cb = std::cos(beta);
  const double cg = std::cos(gamma);
  const double sa = std::sin(alpha);
  const double sb = std::sin(beta);
  const double sg = std::sin(gamma);

  // Evaluate dα/dp, dβ/dp, dγ/dp...
  const double p_scale = geometry_->length();
  const double d_alpha = geometry_->superelevation().f_dot_p(p) *
                         p_scale;
  const double d_beta =
      -cb * cb * geometry_->elevation().f_ddot_p(p);
  const double d_gamma = geometry_->heading_dot_of_p(p);

  // Recall that W is the lane-to-world transform, defined by
  //   (x,y,z)  = W(p,r,h) = (G(p), Z(p)) + R_αβγ*(0,r,h)
  // where G is the reference curve, Z is the elevation profile, and R_αβγ is
  // a rotation matrix derived from reference curve (heading), elevation,
  // and superelevation.
  //
  // Thus, ∂W/∂p = (∂G(p)/∂p, ∂Z(p)/∂p) + (∂R_αβγ/∂p)*(0,r,h), where
  //
  //   ∂G(p)/∂p = G'(p)
  //
  //   ∂Z(p)/∂p = p_scale * (z / p_scale) = p_scale * g'(p)
  //
  //   ∂R_αβγ/∂p = (∂R_αβγ/∂α ∂R_αβγ/∂β ∂R_αβγ/∂γ)*(dα/dp, dβ/dp, dγ/dp)
  return
      Vector3<double>(G_prime.x(),
         G_prime.y(),
         p_scale * g_prime) +

      Vector3<double>((((sa*sg)+(ca*sb*cg))*r + ((ca*sg)-(sa*sb*cg))*h),
         (((-sa*cg)+(ca*sb*sg))*r - ((ca*cg)+(sa*sb*sg))*h),
         ((ca*cb)*r + (-sa*cb)*h))
      * d_alpha +

      Vector3<double>(((sa*cb*cg)*r + (ca*cb*cg)*h),
         ((sa*cb*sg)*r + (ca*cb*sg)*h),
         ((-sa*sb)*r - (ca*sb)*h))
      * d_beta +

      Vector3<double>((((-ca*cg)-(sa*sb*sg))*r + ((+sa*cg)-(ca*sb*sg))*h),
         (((-ca*sg)+(sa*sb*cg))*r + ((sa*sg)+(ca*sb*cg))*h),
         0)
      * d_gamma;
}

Vector3<double> Segment::s_hat_of_prh(const double p, const double r,
    const double h, const Rot3& Rabg) const {
  const Vector3<double> W_prime = W_prime_of_prh(p, r, h, Rabg);
  return W_prime * (1.0 / W_prime.norm());
}

Vector3<double> Segment::r_hat_of_Rabg(const Rot3& Rabg) const {
  return Rabg.apply({0., 1., 0.});
}


const api::Junction* Segment::do_junction() const {
  return junction_;
}

const api::Lane* Segment::do_lane(int index) const {
  DRAKE_DEMAND(index == 0);
  return lane_.get();
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
