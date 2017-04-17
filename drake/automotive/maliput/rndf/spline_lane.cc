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
  spline->Tension(0.0);
  spline->AutoCalculate(true);
  for (const auto &point : control_points) {
    spline->AddPoint(std::get<0>(point));
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
  spline.Tension(0.0);
  for (const auto &point : points) {
    spline.AddPoint(std::get<0>(point), std::get<1>(point));
  }
  return spline.ArcLength();
}

void SplineLane::do_test() const {
/*
  // 2 points, over x-axis. 90° difference orientation
{
  std::unique_ptr<ignition::math::Spline> spline =
    std::make_unique<ignition::math::Spline>();

  spline->AutoCalculate(true);
  spline->AddPoint(ignition::math::Vector3d(0.0, 0.0, 0.0), ignition::math::Vector3d(0.0, 1.0, 0.0));
  spline->AddPoint(ignition::math::Vector3d(10.0, 0.0, 0.0), ignition::math::Vector3d(1.0, 0.0, 0.0));
  std::cout << "s_tot: " << spline->ArcLength() << std::endl;
  for (double s = 0.; s <= 1.0; s += 0.01) {
    std::cout << "t: " << s << " | " << spline->Interpolate(s) << " | " << spline->InterpolateTangent(s) << std::endl;
  }
}
std::cout << "-----------------" << std::endl;
*/
  /*
  // 2 points, over x-axis. 90° difference orientation
{
  std::unique_ptr<ignition::math::Spline> spline =
    std::make_unique<ignition::math::Spline>();

  spline->AutoCalculate(true);
  spline->AddPoint(ignition::math::Vector3d(0.0, 0.0, 0.0), ignition::math::Vector3d(0.0, 1.0, 0.0));
  spline->AddPoint(ignition::math::Vector3d(10.0, 10.0, 0.0), ignition::math::Vector3d(1.0, 0.0, 0.0));
  std::cout << "s_tot: " << spline->ArcLength() << std::endl;
  for (double s = 0.; s <= 1.0; s += 0.01) {
    std::cout << "t: " << s << " | " << spline->Interpolate(s) << " | " << spline->InterpolateTangent(s) << std::endl;
  }
}
std::cout << "-----------------" << std::endl;
*/
  /*
  // 2 points, over x-axis. 0° difference orientation
{
  std::unique_ptr<ignition::math::Spline> spline =
    std::make_unique<ignition::math::Spline>();

  spline->AutoCalculate(true);
  spline->AddPoint(ignition::math::Vector3d(0.0, 0.0, 0.0), ignition::math::Vector3d(0.0, 1.0, 0.0));
  spline->AddPoint(ignition::math::Vector3d(10.0, 0.0, 0.0), ignition::math::Vector3d(0.0, 1.0, 0.0));
  std::cout << "s_tot: " << spline->ArcLength() << std::endl;
  for (double s = 0.; s <= 1.0; s += 0.01) {
    std::cout << "t: " << s << " | " << spline->Interpolate(s) << " | " << spline->InterpolateTangent(s) << std::endl;
  }
}
std::cout << "-----------------" << std::endl;
*/

/*
  // 2 points, over 45°-axis. 0° difference orientation
{
  std::unique_ptr<ignition::math::Spline> spline =
    std::make_unique<ignition::math::Spline>();

  spline->AutoCalculate(true);
  spline->AddPoint(ignition::math::Vector3d(0.0, 0.0, 0.0), ignition::math::Vector3d(std::sqrt(2)/2, std::sqrt(2)/2, 0.0));
  spline->AddPoint(ignition::math::Vector3d(10.0, 10.0, 0.0), ignition::math::Vector3d(std::sqrt(2)/2, std::sqrt(2)/2, 0.0));
  std::cout << "s_tot: " << spline->ArcLength() << std::endl;
  for (double s = 0.; s < 1.0; s += 0.01) {
    std::cout << "t: " << s << " | " << spline->Interpolate(s) << " | " << spline->InterpolateTangent(s) << std::endl;
  }
}
std::cout << "-----------------" << std::endl;
*/
/*
  // 2 points, over 45°-axis. 90° difference orientation
{
  std::unique_ptr<ignition::math::Spline> spline =
    std::make_unique<ignition::math::Spline>();

  spline->AutoCalculate(true);
  spline->AddPoint(ignition::math::Vector3d(0.0, 0.0, 0.0), ignition::math::Vector3d(0.0, 1.0, 0.0));
  spline->AddPoint(ignition::math::Vector3d(10.0, 10.0, 0.0), ignition::math::Vector3d(1.0, 0.0, 0.0));
  std::cout << "s_tot: " << spline->ArcLength() << std::endl;
  for (double s = 0.; s < 1.0; s += 0.1) {
    std::cout << "t: " << s << " | " << spline->Interpolate(s) << " | " << spline->InterpolateTangent(s) << std::endl;
  }
}
std::cout << "-----------------" << std::endl;
*/

/*
  // 2 points, over 45°-axis. 90° difference orientation
{
  std::unique_ptr<ignition::math::Spline> spline =
    std::make_unique<ignition::math::Spline>();

  spline->AutoCalculate(true);
  spline->Tension(1.0);
  spline->AddPoint(ignition::math::Vector3d(0.0, 0.0, 0.0), ignition::math::Vector3d(0.0, 10.0, 0.0));
  spline->AddPoint(ignition::math::Vector3d(10.0, 10.0, 0.0));
  spline->AddPoint(ignition::math::Vector3d(20.0, 0.0, 0.0), ignition::math::Vector3d(0.0, -10.0, 0.0));
  std::cout << "s_tot: " << spline->ArcLength() << std::endl;
  for (double s = 0.; s < 1.0; s += 0.01) {
    std::cout << "t: " << s << " | " << spline->Interpolate(s) << " | " << spline->InterpolateTangent(s) << std::endl;
  }
}
std::cout << "-----------------" << std::endl;
{
  std::unique_ptr<ignition::math::Spline> spline =
    std::make_unique<ignition::math::Spline>();

  spline->AutoCalculate(true);
  spline->Tension(0.5);
  spline->AddPoint(ignition::math::Vector3d(0.0, 0.0, 0.0), ignition::math::Vector3d(0.0, 10.0, 0.0));
  spline->AddPoint(ignition::math::Vector3d(10.0, 10.0, 0.0));
  spline->AddPoint(ignition::math::Vector3d(20.0, 0.0, 0.0), ignition::math::Vector3d(0.0, -10.0, 0.0));
  std::cout << "s_tot: " << spline->ArcLength() << std::endl;
  for (double s = 0.; s < 1.0; s += 0.01) {
    std::cout << "t: " << s << " | " << spline->Interpolate(s) << " | " << spline->InterpolateTangent(s) << std::endl;
  }
}
std::cout << "-----------------" << std::endl;
{
  std::unique_ptr<ignition::math::Spline> spline =
    std::make_unique<ignition::math::Spline>();

  spline->AutoCalculate(true);
  spline->Tension(0.0);
  spline->AddPoint(ignition::math::Vector3d(0.0, 0.0, 0.0), ignition::math::Vector3d(0.0, 10.0, 0.0));
  spline->AddPoint(ignition::math::Vector3d(10.0, 10.0, 0.0));
  spline->AddPoint(ignition::math::Vector3d(20.0, 0.0, 0.0), ignition::math::Vector3d(0.0, -10.0, 0.0));
  std::cout << "s_tot: " << spline->ArcLength() << std::endl;
  for (double s = 0.; s < 1.0; s += 0.01) {
    std::cout << "t: " << s << " | " << spline->Interpolate(s) << " | " << spline->InterpolateTangent(s) << std::endl;
  }
}
std::cout << "-----------------" << std::endl;
*/
  /*
{
  std::unique_ptr<ignition::math::Spline> spline =
    std::make_unique<ignition::math::Spline>();

  const ignition::math::Vector3d p1(0,0,0);
  const ignition::math::Vector3d p2(47.8908,52.9149,-0.0004);
  const ignition::math::Vector3d p3(287.807,156.855,-0.008422);
  const ignition::math::Vector3d p4(314.413,143.875,-0.00937);

  spline->AutoCalculate(true);
  spline->Tension(0.0);
  spline->AddPoint(ignition::math::Vector3d(0,0,0), (p2 - p1) * 0.5);
  spline->AddPoint(ignition::math::Vector3d(47.8908,52.9149,-0.0004));
  spline->AddPoint(ignition::math::Vector3d(69.0841,74.103,-0.000806));
  spline->AddPoint(ignition::math::Vector3d(105.782,104.498,-0.001735));
  spline->AddPoint(ignition::math::Vector3d(144.866,128.016,-0.002933));
  spline->AddPoint(ignition::math::Vector3d(188.905,144.655,-0.004441));
  spline->AddPoint(ignition::math::Vector3d(196.244,145.098,-0.004672));
  spline->AddPoint(ignition::math::Vector3d(232.668,155.414,-0.006139));
  spline->AddPoint(ignition::math::Vector3d(287.807,156.855,-0.008422));
  spline->AddPoint(ignition::math::Vector3d(314.413,143.875,-0.00937), (p3 - p4) * 0.5);
  std::cout << "s_tot: " << spline->ArcLength() << std::endl;
  for (double s = 0.; s < 1.0; s += 0.01) {
    std::cout << "t: " << s << " | " << spline->Interpolate(s) << " | " << spline->InterpolateTangent(s) << std::endl;
  }
}
std::cout << "-----------------" << std::endl;
*/

}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake