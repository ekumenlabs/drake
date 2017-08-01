#include "drake/automotive/maliput/multilane/line_segment_geometry.h"

#include "drake/math/saturate.h"

namespace drake {
namespace maliput {
namespace multilane {

Vector3<double> LineGeometry::ToCurveFrame(
    const Vector3<double>& geo_coordinate,
    const std::pair<double, double>& lateral_bounds,
    const std::pair<double, double>& height_bounds) const {
  DRAKE_THROW_UNLESS(lateral_bounds.first < lateral_bounds.second);
  DRAKE_THROW_UNLESS(height_bounds.first < height_bounds.second);
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
  const double r = math::saturate(r_unsaturated, lateral_bounds.first,
                                  lateral_bounds.second);
  // N.B. h is the geo z-coordinate referenced against the lane elevation (whose
  // `a` coefficient is normalized by lane length).
  const double h_unsaturated = geo_coordinate.z() - elevation().a() * length();
  const double h = math::saturate(h_unsaturated, height_bounds.first,
                                  height_bounds.second);
  return Vector3<double>(s, r, h);
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
