#include "drake/automotive/maliput/multilane/line_road_curve.h"

#include "drake/math/saturate.h"

namespace drake {
namespace maliput {
namespace multilane {

Vector3<double> LineRoadCurve::ToCurveFrame(
    const Vector3<double>& geo_coordinate,
    const api::RBounds& lateral_bounds,
    const api::HBounds& height_bounds) const {
  // TODO(jadecastro): Lift the zero superelevation and zero elevation gradient
  // restriction.
  const Vector2<double> s_unit_vector = dp_ / dp_.norm();
  const Vector2<double> r_unit_vector{-s_unit_vector(1), s_unit_vector(0)};

  const Vector2<double> p(geo_coordinate.x(), geo_coordinate.y());
  const Vector2<double> lane_origin_to_p = p - p0_;

  // Compute the distance from `p` to the start of the lane.
  const double s_unsaturated = lane_origin_to_p.dot(s_unit_vector);
  const double s = math::saturate(s_unsaturated, 0., length());
  const double r_unsaturated = lane_origin_to_p.dot(r_unit_vector);
  const double r = math::saturate(r_unsaturated, lateral_bounds.min(),
                                  lateral_bounds.max());
  // N.B. h is the geo z-coordinate referenced against the lane elevation (whose
  // `a` coefficient is normalized by lane length).
  const double h_unsaturated = geo_coordinate.z() -
                               elevation().f_p(0) * length();
  const double h = math::saturate(h_unsaturated, height_bounds.min(),
                                  height_bounds.max());
  return Vector3<double>(s, r, h);
}

std::unique_ptr<RoadCurve> LineRoadCurve::Offset(double r) const {
  // Computes the new p0 point.
  const double scale = dp_.norm();
  const Vector2<double> s_unit_vector = dp_ / scale;
  const Vector2<double> r_unit_vector{-s_unit_vector(1), s_unit_vector(0)};
  const Vector2<double> p0 = p0_ + r_unit_vector * r;
  // Creates a new composed elevation and sets the new lateral displacement.
  Elevation<double> composed_elevation(elevation());
  composed_elevation.set_r(r + elevation().r());
  return std::make_unique<LineRoadCurve>(p0, dp_,
                                         composed_elevation,
                                         superelevation());
}


}  // namespace multilane
}  // namespace maliput
}  // namespace drake
