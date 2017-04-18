#include "drake/automotive/maliput/rndf/spline_lane.h"

#include <algorithm>
#include <tuple>
#include <cmath>
#include <limits>

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace rndf {


SplineLane::SplineLane(const api::LaneId& id, const api::Segment* segment,
  const std::vector<std::tuple<ignition::math::Vector3d,
    ignition::math::Vector3d>> &control_points,
  const api::RBounds& lane_bounds,
  const api::RBounds& driveable_bounds):
    Lane(id,
      segment,
      lane_bounds,
      driveable_bounds,
      ComputeLength(control_points)) {
  std::unique_ptr<ignition::math::Spline> spline =
    std::make_unique<ignition::math::Spline>();
  spline->Tension(SPLINE_TENSION);
  spline->AutoCalculate(true);
  for (const auto &point : control_points) {
    spline->AddPoint(std::get<0>(point), std::get<1>(point));
  }
  spline_ = std::make_unique<ArcLengthParameterizedSpline>(
    spline, control_points.size() - 1);
}

api::LanePosition SplineLane::DoToLanePosition(
  const api::GeoPosition&,
  api::GeoPosition*,
  double*) const {
    DRAKE_ABORT();
}

api::GeoPosition SplineLane::DoToGeoPosition(
  const api::LanePosition& lane_pos) const {
  // RNDF doesn't have any elevation
  const double z = 0.0;
  // Calculate x,y of (s,0,0).
  const V2 xy = xy_of_s(lane_pos.s);
  // Calculate orientation of (s,r,h) basis at (s,0,0).
  const Rot3 ypr = Rabg_of_s(lane_pos.s);
  // Rotate (0,r,h) and sum with mapped (s,0,0).
  const V3 xyz =
      ypr.apply({0., lane_pos.r, lane_pos.h}) + V3(xy.x(), xy.y(), z);
  return {xyz.x(), xyz.y(), xyz.z()};
}

api::Rotation SplineLane::DoGetOrientation(const api::LanePosition& lane_pos) const {
  // Recover linear parameter p from arc-length position s.
  const double s = lane_pos.s;
  const double r = lane_pos.r;
  const double h = lane_pos.h;
  // Calculate orientation of (s,r,h) basis at (s,0,0).
  const Rot3 Rabg = Rabg_of_s(s);

  // Calculate s,r basis vectors at (s,r,h)...
  const V3 s_hat = s_hat_of_srh(s, r, h, Rabg);
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
  return api::Rotation(0.0, 0.0, gamma);
}

api::LanePosition SplineLane::DoEvalMotionDerivatives(
    const api::LanePosition& position,
    const api::IsoLaneVelocity& velocity) const {
  const double s = position.s;
  const double r = position.r;
  const double h = position.h;

  const Rot3 R = Rabg_of_s(s);
  const V3 W_prime = W_prime_of_srh(s, r, h, R);

  // The definition of path-length of a path along σ yields dσ = |∂W/∂p| dp.
  // Similarly, path-length s along the road at r = 0 is related to the
  // elevation by ds = p_scale * sqrt(1 + g'^2) dp.  Chaining yields ds/dσ:
  const double ds_dsigma = p_scale_ / W_prime.norm();

  return api::LanePosition(ds_dsigma * velocity.sigma_v,
                           velocity.rho_v,
                           velocity.eta_v);
}

V3 SplineLane::s_hat_of_srh(const double s, const double r, const double h,
                      const Rot3& Rabg) const {
  const V3 W_prime = W_prime_of_srh(s, r, h, Rabg);
  return W_prime * (1.0 / W_prime.norm());
}
V3 SplineLane::W_prime_of_srh(const double s, const double r, const double h,
                        const Rot3& Rabg) const {
  const V2 G_prime = xy_dot_of_s(s);

  const Rot3& R = Rabg;
  const double alpha = R.roll();
  const double beta = R.pitch();
  const double gamma = R.yaw();

  const double ca = 1.0;
  const double cb = std::cos(beta);
  const double cg = std::cos(gamma);
  const double sa = std::sin(alpha);
  const double sb = std::sin(beta);
  const double sg = std::sin(gamma);

  // Evaluate dα/dp, dβ/dp, dγ/dp...
  const double d_alpha = 0.0;
  const double d_beta = 0.0;
  const double d_gamma = heading_dot_of_s(s);

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
      V3(G_prime.x(), G_prime.y(), 0.0) +

      V3((((sa*sg)+(ca*sb*cg))*r + ((ca*sg)-(sa*sb*cg))*h),
         (((-sa*cg)+(ca*sb*sg))*r - ((ca*cg)+(sa*sb*sg))*h),
         ((ca*cb)*r + (-sa*cb)*h))
      * d_alpha +

      V3(((sa*cb*cg)*r + (ca*cb*cg)*h),
         ((sa*cb*sg)*r + (ca*cb*sg)*h),
         ((-sa*sb)*r - (ca*sb)*h))
      * d_beta +

      V3((((-ca*cg)-(sa*sb*sg))*r + ((+sa*cg)-(ca*sb*sg))*h),
         (((-ca*sg)+(sa*sb*cg))*r + ((sa*sg)+(ca*sb*cg))*h),
         0)
      * d_gamma;
}

V2 SplineLane::xy_of_s(const double s) const {
  // xy_of_p it's called L which is a function
  // R --> R^2. We discard z component right now. We can say
  // L = f(p) = (x(p) ; y(p))
  //const auto &point = spline_->InterpolateMthDerivative(0, p * spline_->BaseSpline()->ArcLength());
  const auto &point = spline_->InterpolateMthDerivative(0, s);
  return {point.X(), point.Y()};
}
V2 SplineLane::xy_dot_of_s(const double s) const {
  // We get here the tangent, which is the first derivative of
  // L --> dL(p) / dp
  //const auto& tangent = spline_->InterpolateMthDerivative(1, p * spline_->BaseSpline()->ArcLength());
  const auto &point = spline_->InterpolateMthDerivative(1, s);
  return {point.X(), point.Y()};
}
double SplineLane::heading_of_s(const double s) const {
  // The tangent of the heading is the function of y(p) / x(p).
  // So, we can say that h(p) = arctg (y(p) / x(p)). This function
  // is a function like: h(p) = R --> R or h(f(x, y)) where f it's
  // a function defined like y / x. y and x are the components
  // of the first derivative of L. Then, we got: f: R^2 --> R
  const auto tangent = xy_dot_of_s(s);
  return std::atan2(tangent.y(), tangent.x());
}

double SplineLane::heading_dot_of_s(const double s) const {
  // Based on the explanation of heading_of_p, we got applying the chain rule:
  // dh / dp = d/dp {arctg (f(x(p), y(p)))}
  //  = 1 / (1 + f(x(p), y(p))^2) * d/dp {f(x(p), y(p))}
  // As x(p) and y(p) and independant polynomials, we can say that:
  // df(x(p), y(p)) / dp = (y' * x - y * x') / x^2
  // Where y and x are the components of the L' and, x' and y' are
  // the components of L'' as they are independant.
  const double heading = heading_of_s(s);
  // const auto first_derivative = spline_->InterpolateMthDerivative(1, p * spline_->BaseSpline()->ArcLength());
  // const auto second_derivative = spline_->InterpolateMthDerivative(2, p * spline_->BaseSpline()->ArcLength());
  const auto &first_derivative = spline_->InterpolateMthDerivative(1, s);
  const auto &second_derivative = spline_->InterpolateMthDerivative(2, s);
  const double m =
    ( second_derivative.Y() * first_derivative.X() -
      first_derivative.Y() * second_derivative.X() ) /
    (first_derivative.X() * first_derivative.X());
  return (1.0 / (1.0 + heading * heading) * m);
}

Rot3 SplineLane::Rabg_of_s(const double s) const {
  return Rot3(0.0, 0.0, heading_of_s(s));
}

double SplineLane::ComputeLength(
  const std::vector<std::tuple<ignition::math::Vector3d,
    ignition::math::Vector3d>> &points) {
  ignition::math::Spline spline;
  spline.AutoCalculate(true);
  spline.Tension(SPLINE_TENSION);
  for (const auto &point : points) {
    spline.AddPoint(std::get<0>(point), std::get<1>(point));
  }
  return spline.ArcLength();
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake