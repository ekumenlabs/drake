#include <algorithm>

#include <ignition/math/Vector4.hh>
#include <ignition/math/Matrix4.hh>

#include "drake/automotive/maliput/rndf/spline_helpers.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace rndf {

static const int kFunctionPartitionTreeDegree = 10;

static const int kFunctionPartitionTreeMaxDepth = 10;

InverseFunctionInterpolator::InverseFunctionInterpolator(
    const std::function<double(double)> _function, const double _xmin,
    const double _xmax, const double _error_boundary)
    : function_(_function), error_boundary_(_error_boundary) {
  // Make sure the error boundary is attainable.
  DRAKE_THROW_UNLESS(_error_boundary > 0);
  // Instantiate the partition tree root
  this->partition_tree_ = std::make_unique<FunctionGraphPartition>();
  this->partition_tree_->segment.xmin = _xmin;
  this->partition_tree_->segment.xmax = _xmax;
  this->partition_tree_->segment.ymin = _function(_xmin);
  this->partition_tree_->segment.ymax = _function(_xmax);
  this->partition_tree_degree_ = kFunctionPartitionTreeDegree;
  this->partition_tree_max_depth_ = kFunctionPartitionTreeMaxDepth;
}

double
InverseFunctionInterpolator::InterpolateMthDerivative(const int _mth,
                                                      const double _y) const {
  // Make sure that the derivative order is a positive integer or zero.
  DRAKE_THROW_UNLESS(_mth >= 0);
  // Make sure that y is not above x(y) image interval upper bound.
  DRAKE_THROW_UNLESS((_y - this->partition_tree_->segment.ymax) <
                     this->error_boundary_);
  // Make sure that y is not below x(y) image interval low bound.
  DRAKE_THROW_UNLESS((this->partition_tree_->segment.ymin - _y) <
                     this->error_boundary_);

  // Zero out any derivative of higher order than
  // 1, as this is a piecewise linear interpolant.
  if (_mth > 1)
    return 0.0;

  // Get partition tree root.
  FunctionGraphPartition *root = this->partition_tree_.get();

  // Clamp interpolation y
  double iy = std::min(std::max(_y, root->segment.ymin), root->segment.ymax);

  // Traverse down the partition tree until the required
  // error is attained.
  int depth = 0;
  while (true) {
    if (root->partitions.empty()) {
      // Visiting a a leaf node.

      // Linearly interpolate x for the given y.
      double dx = root->segment.xmax - root->segment.xmin;
      double dy = root->segment.ymax - root->segment.ymin;
      double ix = root->segment.xmin + (dx / dy) * (iy - root->segment.ymin);

      // Check if the interpolation suffices error bound requirements.
      if (std::abs(this->function_(ix) - iy) < this->error_boundary_) {
        if (_mth == 1) {
          // Return derivative dx/dy at y.
          return dx / dy;
        }
        // Return interpolated x(y).
        return ix;
      }

      // Safety check before building next tree level.
      DRAKE_THROW_UNLESS(depth <= this->partition_tree_max_depth_);

      // Build sub-partitions for this partition.
      double segmentdx = dx / this->partition_tree_degree_;
      for (int i = 0; i < this->partition_tree_degree_; ++i) {
        double segmentxmin = root->segment.xmin + i * segmentdx;
        double segmentxmax = segmentxmin + segmentdx;

        auto subp = std::make_unique<FunctionGraphPartition>();
        DRAKE_DEMAND(this->partition_tree_ != nullptr);
        subp->segment.xmin = segmentxmin;
        subp->segment.xmax = segmentxmax;
        subp->segment.ymin = this->function_(segmentxmin);
        subp->segment.ymax = this->function_(segmentxmax);
        root->partitions.push_back(std::move(subp));
      }
    }
    // Visiting an inner node.

    // Search subpartition that contains y.
    auto it = std::find_if(
        root->partitions.begin(), root->partitions.end(),
        [&iy](const std::unique_ptr<FunctionGraphPartition> &subp) {
          return (subp->segment.ymin <= iy && iy <= subp->segment.ymax);
        });

    // Use subpartition as new root.
    DRAKE_DEMAND(it != root->partitions.end());
    DRAKE_DEMAND(it->get() != nullptr);

    root = it->get();
    depth++;
  }
}

ArcLengthParameterizedSpline::ArcLengthParameterizedSpline(
    std::unique_ptr<ignition::math::Spline> _spline,
    const double _error_boundary)
    : q_t_(std::move(_spline)) {
  DRAKE_ASSERT(this->q_t_ != nullptr);
  // Instantiate an inverse function interpolator
  // for the given spline arc length function.
  t_s_ = std::make_unique<InverseFunctionInterpolator>([this](double t) {
    return this->q_t_->ArcLength(t);
  }, 0.0, 1.0, _error_boundary);
}

ignition::math::Vector3d
ArcLengthParameterizedSpline::InterpolateMthDerivative(const int _mth,
                                                       const double _s) const {
  if (_mth > 3) {
    // M > 3 => p = 0 (as this is a cubic interpolator)
    return ignition::math::Vector3d(0.0, 0.0, 0.0);
  }
  double t_s = this->t_s_->InterpolateMthDerivative(0, _s);
  if (_mth < 1) {
    // M = 0 => P(s) = Q(t(s))
    return this->q_t_->InterpolateMthDerivative(0, t_s);
  }
  double t_prime_s = this->t_s_->InterpolateMthDerivative(1, _s);
  ignition::math::Vector3d q_prime_t =
      this->q_t_->InterpolateMthDerivative(1, t_s);

  if (_mth < 2) {
    // M = 1 => P'(s) = Q'(t(s)) * t'(s)
    return q_prime_t * t_prime_s;
  }
  double t_prime_s_2 = t_prime_s * t_prime_s;
  double t_prime2_s = this->t_s_->InterpolateMthDerivative(2, _s);
  ignition::math::Vector3d q_prime2_t =
      this->q_t_->InterpolateMthDerivative(2, t_s);
  if (_mth < 3) {
    // M = 2 => P''(s) = Q''(t(s)) * t'(s)^2 + Q'(t(s)) * t''(s)
    return q_prime2_t * t_prime_s_2 + q_prime_t * t_prime2_s;
  }
  double t_prime_s_3 = t_prime_s_2 * t_prime_s;
  double t_prime3_s = this->t_s_->InterpolateMthDerivative(3, _s);
  ignition::math::Vector3d q_prime3_t =
      this->q_t_->InterpolateMthDerivative(3, t_s);
  // M = 3 => P'''(s) = Q'''(t(s)) * t'(s)^3
  ///                   + 3 * Q''(t(s)) * t'(s) * t''(s)
  ///                   + Q'(t(s)) * t'''(s)
  return (q_prime3_t * t_prime_s_3 + 3 * q_prime2_t * t_prime_s * t_prime2_s +
          q_prime_t * t_prime3_s);
}

double ArcLengthParameterizedSpline::FindClosestPointTo(
    const ignition::math::Vector3d &point, const double step) const {
  double closest_s = 0.0;
  double min_distance = std::numeric_limits<double>::max();
  double distance, t_s;
  for (double s = 0.0; s < q_t_->ArcLength(); s += step) {
    t_s = t_s_->InterpolateMthDerivative(0, s);
    distance = (q_t_->InterpolateMthDerivative(0, t_s) - point).Length();
    if (distance < min_distance) {
      min_distance = distance;
      closest_s = s;
    }
  }
  t_s = this->t_s_->InterpolateMthDerivative(0, q_t_->ArcLength());
  distance = (q_t_->InterpolateMthDerivative(0, t_s) - point).Length();
  if (distance < min_distance) {
    min_distance = distance;
    closest_s = q_t_->ArcLength();
  }

  return closest_s;
}

std::vector<ignition::math::Vector3d> SplineToBezier(
    const ignition::math::Vector3d& p0,
    const ignition::math::Vector3d& t0,
    const ignition::math::Vector3d& p1,
    const ignition::math::Vector3d& t1) {
  // This is the Bezier coefficient matrix.
  const ignition::math::Matrix4d bezier_matrix(1.0, 0.0, 0.0, 0.0,
                                               -3.0, 3.0, 0.0, 0.0,
                                               3.0, -6.0, 3.0, 0.0,
                                               -1.0, 3.0, -3.0, 1.0);
  // This is the Hermite coefficient matrix.
  const ignition::math::Matrix4d hermite_matrix(1.0, 0.0, 0.0, 0.0,
                                                0.0, 1.0, 0.0, 0.0,
                                                -3.0, -2.0, 3.0, -1.0,
                                                2.0, 1.0, -2.0, 1.0);
  // These are the control points arranged by coordinate.
  const ignition::math::Matrix4d hermite_points(p0.X(), p0.Y(), p0.Z(), 0.0,
                                                t0.X(), t0.Y(), t0.Z(), 0.0,
                                                p1.X(), p1.Y(), p1.Z(), 0.0,
                                                t1.X(), t1.Y(), t1.Z(), 0.0);
  // Given a function F_B(t): ℝ --> ℝ^3 that represents a cubic Bezier curve,
  // and F_H(t) : ℝ --> ℝ^3 that represents a cubic Hermite Spline curve, we can
  // define them like:
  // F_B(t) = [1 t t^2 t^3] * [bezier_matrix] * [bezier_points]
  // F_H(t) = [1 t t^2 t^3] * [hermite_matrix] * [hermite_points]
  // If both functions have the same image: F_B(t) = F_H(t), we can say:
  // [bezier_points] = [bezier_matrix]^(-1) * [hermite_matrix] * [hermite_points]
  // [hermite_points] = [hermite_matrix]^(-1) * [bezier_matrix] * [bezier_points]
  const ignition::math::Matrix4d bezier_points =
      bezier_matrix.Inverse() * hermite_matrix * hermite_points;
  std::vector<ignition::math::Vector3d> result;
  result.push_back(ignition::math::Vector3d(
      bezier_points(0,0), bezier_points(0,1), bezier_points(0,2)));
  result.push_back(ignition::math::Vector3d(
      bezier_points(1,0), bezier_points(1,1), bezier_points(1,2)));
  result.push_back(ignition::math::Vector3d(
      bezier_points(2,0), bezier_points(2,1), bezier_points(2,2)));
  result.push_back(ignition::math::Vector3d(
      bezier_points(3,0), bezier_points(3,1), bezier_points(3,2)));
  return result;
}


std::vector<ignition::math::Vector3d> BezierToSpline(
    const ignition::math::Vector3d& p0,
    const ignition::math::Vector3d& p1,
    const ignition::math::Vector3d& p2,
    const ignition::math::Vector3d& p3) {
  // This is the Bezier coefficient matrix.
  const ignition::math::Matrix4d bezier_matrix(1.0, 0.0, 0.0, 0.0,
                                               -3.0, 3.0, 0.0, 0.0,
                                               3.0, -6.0, 3.0, 0.0,
                                               -1.0, 3.0, -3.0, 1.0);
  // This is the Hermite coefficient matrix.
  const ignition::math::Matrix4d hermite_matrix(1.0, 0.0, 0.0, 0.0,
                                                0.0, 1.0, 0.0, 0.0,
                                                -3.0, -2.0, 3.0, -1.0,
                                                2.0, 1.0, -2.0, 1.0);
  // These are the control points arranged by coordinate.
  const ignition::math::Matrix4d bezier_points(p0.X(), p0.Y(), p0.Z(), 0.0,
                                               p1.X(), p1.Y(), p1.Z(), 0.0,
                                               p2.X(), p2.Y(), p2.Z(), 0.0,
                                               p3.X(), p3.Y(), p3.Z(), 0.0);
  // Given a function F_B(t): ℝ --> ℝ^3 that represents a cubic Bezier curve,
  // and F_H(t) : ℝ --> ℝ^3 that represents a cubic Hermite Spline curve, we can
  // define them like:
  // F_B(t) = [1 t t^2 t^3] * [bezier_matrix] * [bezier_points]
  // F_H(t) = [1 t t^2 t^3] * [hermite_matrix] * [hermite_points]
  // If both functions have the same image: F_B(t) = F_H(t), we can say:
  // [bezier_points] = [bezier_matrix]^(-1) * [hermite_matrix] * [hermite_points]
  // [hermite_points] = [hermite_matrix]^(-1) * [bezier_matrix] * [bezier_points]
  ignition::math::Matrix4d hermite_points =
      hermite_matrix.Inverse() * bezier_matrix * bezier_points;
  std::vector<ignition::math::Vector3d> result;
  result.push_back(ignition::math::Vector3d(
      hermite_points(0,0), hermite_points(0,1), hermite_points(0,2)));
  result.push_back(ignition::math::Vector3d(
      hermite_points(1,0), hermite_points(1,1), hermite_points(1,2)));
  result.push_back(ignition::math::Vector3d(
      hermite_points(2,0), hermite_points(2,1), hermite_points(2,2)));
  result.push_back(ignition::math::Vector3d(
      hermite_points(3,0), hermite_points(3,1), hermite_points(3,2)));
  return result;
}


std::vector<ignition::math::Vector3d> RemoveLoopsInBezier(
    const std::vector<ignition::math::Vector3d>& cp,
    double tension = 1.0) {
  const double kAlmostZero = 1e-3;
  const ignition::math::Vector3d& p0 = cp[0];
  const ignition::math::Vector3d& p1 = cp[1];
  const ignition::math::Vector3d& p2 = cp[2];
  const ignition::math::Vector3d& p3 = cp[3];

  // Tangents unit vectors.
  const ignition::math::Vector3d norm_t0 = (p1 - p0).Normalize();
  const ignition::math::Vector3d norm_t3 = (p3 - p2).Normalize();
  // A vector that joins both extents.
  const ignition::math::Vector3d r = p3 - p0;
  // Cross products to determine if the lines will intersect or not.
  const ignition::math::Vector3d norm_t3_x_r = norm_t3.Cross(r);
  const ignition::math::Vector3d norm_t3_x_norm_t0 = norm_t3.Cross(norm_t0);

  std::vector<ignition::math::Vector3d> result;
  if (norm_t3_x_r.Length() < kAlmostZero ||
      norm_t3_x_norm_t0.Length() < kAlmostZero) {
    // Here we won't have intersection as the lines are not coplanar or are
    // parallel. As we currently do not use any other z coordinate different
    // from 0.0, we assume they are coplanar and parallel. From this assumption
    // we build the control points for the Spline that match the transition.
    result.push_back(p0);
    double r_dot_norm_t0 = norm_t0.Dot(r);
    result.push_back(p0 + (0.5 * r_dot_norm_t0) * norm_t0);
    result.push_back(p3 + (-0.5 * r_dot_norm_t0) * norm_t3);
    result.push_back(p3);
    return result;
  }
  // The vector to move from p0 towards the control point.
  const ignition::math::Vector3d l =
    (norm_t3_x_r.Length() / norm_t3_x_norm_t0.Length()) * norm_t0;
  // Check how we need to add l.
  double projection = norm_t3_x_r.Dot(norm_t3_x_norm_t0);
  ignition::math::Vector3d critical_point;
  if (projection >= kAlmostZero) {
    critical_point = p0 + l;
  } else {
    critical_point = p0 - l;
  }

  // We compute if the resulting Bezier curve will preserve convexity.
  const ignition::math::Vector3d diff_to_p0 = (critical_point - p0);
  const ignition::math::Vector3d diff_to_p3 = (p3 - critical_point);

  if (diff_to_p3.Normalized().Dot(norm_t3) < kAlmostZero) {
    result.push_back(p0);
    result.push_back(p0 + 0.1 * (critical_point - p0));
    result.push_back(p3 + (-0.1) * (critical_point - p3));
    result.push_back(p3);
  }
  else if (diff_to_p0.Normalized().Dot(norm_t0) < kAlmostZero) {
    result.push_back(p0);
    result.push_back(p0 + (-0.1) * (critical_point - p0));
    result.push_back(p3 + 0.1 * (critical_point - p3));
    result.push_back(p3);

  }
  else {
    result.push_back(p0);
    result.push_back(p0 + tension * (critical_point - p0));
    result.push_back(p3 + tension * (critical_point - p3));
    result.push_back(p3);

  }


  return result;
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
