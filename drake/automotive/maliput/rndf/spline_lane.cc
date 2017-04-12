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
  const api::RBounds& driveable_bounds,
  const CubicPolynomial& elevation,
  const CubicPolynomial& superelevation):
    Lane(id,
      segment,
      lane_bounds,
      driveable_bounds,
      ComputeLength(control_points),
      elevation,
      superelevation) {
  std::unique_ptr<ignition::math::Spline> spline =
    std::make_unique<ignition::math::Spline>();
  //spline_->Tension(0.0);
  spline->AutoCalculate(true);
  for (const auto &point : control_points) {
    //std::cout << std::atan2(std::get<1>(point).Y(), std::get<1>(point).X()) << std::endl;
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

V2 SplineLane::xy_of_p(const double p) const {
  // xy_of_p it's called L which is a function
  // R --> R^2. We discard z component right now. We can say
  // L = f(p) = (x(p) ; y(p))
  const auto point = spline_->InterpolateMthDerivative(0, p * spline_->BaseSpline()->ArcLength());
  return {point.X(), point.Y()};
}

V2 SplineLane::xy_dot_of_p(const double p) const {
  // We get here the tangent, which is the first derivative of
  // L --> dL(p) / dp
  const auto& tangent = spline_->InterpolateMthDerivative(1, p * spline_->BaseSpline()->ArcLength());
  return {tangent.X(), tangent.Y()};
}

double SplineLane::heading_of_p(const double p) const {
  // The tangent of the heading is the function of y(p) / x(p).
  // So, we can say that h(p) = arctg (y(p) / x(p)). This function
  // is a function like: h(p) = R --> R or h(f(x, y)) where f it's
  // a function defined like y / x. y and x are the components
  // of the first derivative of L. Then, we got: f: R^2 --> R
  const auto tangent = xy_dot_of_p(p);
  return std::atan2(tangent.y(), tangent.x());
}

double SplineLane::heading_dot_of_p(const double p) const {
  // Based on the explanation of heading_of_p, we got applying the chain rule:
  // dh / dp = d/dp {arctg (f(x(p), y(p)))}
  //  = 1 / (1 + f(x(p), y(p))^2) * d/dp {f(x(p), y(p))}
  // As x(p) and y(p) and independant polynomials, we can say that:
  // df(x(p), y(p)) / dp = (y' * x - y * x') / x^2
  // Where y and x are the components of the L' and, x' and y' are
  // the components of L'' as they are independant.
  const double heading = heading_of_p(p);
  const auto first_derivative = spline_->InterpolateMthDerivative(1, p * spline_->BaseSpline()->ArcLength());
  const auto second_derivative = spline_->InterpolateMthDerivative(2, p * spline_->BaseSpline()->ArcLength());
  const double m =
    ( second_derivative.Y() * first_derivative.X() -
      first_derivative.Y() * second_derivative.X() ) /
    (first_derivative.X() * first_derivative.X());
  // std::cout << "p: " << p << " h: " << heading << " h_dot " << (1.0 / (1.0 + heading * heading) * m) << std::endl;
  return (1.0 / (1.0 + heading * heading) * m);
}

double SplineLane::module_p(const double _p) const {
  double p = std::max(0.0, _p);
  p = std::min(1.0, p);
  return p;
}

double SplineLane::ComputeLength(
  const std::vector<std::tuple<ignition::math::Vector3d,
    ignition::math::Vector3d>> &points) {
  ignition::math::Spline spline;
  spline.AutoCalculate(true);
  for (const auto &point : points) {
    spline.AddPoint(std::get<0>(point), std::get<1>(point));
  }
  return spline.ArcLength();
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake