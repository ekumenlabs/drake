#include <cmath>
#include <vector>
#include <iostream>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector4.hh>
#include <ignition/math/Matrix4.hh>

namespace drake {
namespace examples {
namespace bezier {
namespace {

class CubicBezier {
 public:
  static CubicBezier FromPointAndTangent(const ignition::math::Vector3d& p0,
                                         const ignition::math::Vector3d& t0,
                                         const ignition::math::Vector3d& p1,
                                         const ignition::math::Vector3d& t1) {
    const ignition::math::Vector3d pa = (1.0 / 3.0) * t0 + p0;
    const ignition::math::Vector3d pb = (-1.0 / 3.0) * t1 + p1;
    return CubicBezier(p0, pa, pb, p1);
  }

  explicit CubicBezier(const ignition::math::Vector3d& p0,
                       const ignition::math::Vector3d& p1,
                       const ignition::math::Vector3d& p2,
                       const ignition::math::Vector3d& p3) :
      p0_(p0), p1_(p1), p2_(p2), p3_(p3) {
  }

  ignition::math::Vector3d InterpolateMthDerivative(int derivative_order,
                                                      double t) const {
    t = std::max(0.0, std::min(t, 1.0));
    switch(derivative_order) {
      case 0:
        return std::pow(1.0 - t, 3.0) * p0_ +
               3.0 * std::pow(1.0 - t, 2.0) * t * p1_ +
               3.0 * (1.0 - t) * std::pow(t, 2.0) * p2_ +
               std::pow(t, 3.0) * p3_;
      break;
      case 1:
        return 3.0 * std::pow(1.0 - t, 2.0) * (p1_ - p0_) +
               6.0 * (1.0 - t) * t * (p2_ - p1_) +
               3.0 * std::pow(t, 2.0) * (p3_ - p2_);
      break;
      case 2:
        return 6.0 * (t - 1.0) * (p1_ - p0_) +
               6.0 * (1.0 - 2.0 * t) * (p2_ - p1_) +
               6.0 * t * (p3_ - p2_);
      break;
      case 3:
        return 6.0 * (p1_ - p0_) +
               - 12.0 * (p2_ - p1_) +
               6.0 * (p3_ - p2_);
        break;
      default:
        return ignition::math::Vector3d(0.0, 0.0, 0.0);
        break;
    }
  }

 private:
  const ignition::math::Vector3d p0_;
  const ignition::math::Vector3d p1_;
  const ignition::math::Vector3d p2_;
  const ignition::math::Vector3d p3_;
};

class MatrixCubicBezier {
 public:

  explicit MatrixCubicBezier(const ignition::math::Vector3d& p0,
                   const ignition::math::Vector3d& p1,
                   const ignition::math::Vector3d& p2,
                   const ignition::math::Vector3d& p3) :
      p0_(p0), p1_(p1), p2_(p2), p3_(p3) {
  }

  ignition::math::Vector3d InterpolateMthDerivative(int derivative_order,
                                                      double t) const {
    t = std::max(0.0, std::min(t, 1.0));
    switch(derivative_order) {
      case 0:
        ignition::math::Matrix4d matrix(1.0, 0.0, 0.0, 0.0,
                                        -3.0, 3.0, 0.0, 0.0,
                                        3.0, -6.0, 3.0, 0.0,
                                        -1.0, 3.0, -3.0, 1.0);
        ignition::math::Matrix4d p_vector(p0_.X(), p0_.Y(), p0_.Z(), 0.0,
                                          p1_.X(), p1_.Y(), p1_.Z(), 0.0,
                                          p2_.X(), p2_.Y(), p2_.Z(), 0.0,
                                          p3_.X(), p3_.Y(), p3_.Z(), 0.0);
        ignition::math::Vector4d t_vector(1, t, t * t, t * t * t);
        ignition::math::Vector4d result = t_vector * (matrix * p_vector);
        return ignition::math::Vector3d(result.X(), result.Y(), result.Z());
      break;
    }
    return ignition::math::Vector3d();
  }

 private:
  const ignition::math::Vector3d p0_;
  const ignition::math::Vector3d p1_;
  const ignition::math::Vector3d p2_;
  const ignition::math::Vector3d p3_;
};
/*
class MatrixHermiteSpline {
 public:

  explicit MatrixHermiteSpline(const ignition::math::Vector3d& p0,
                   const ignition::math::Vector3d& p1,
                   const ignition::math::Vector3d& p2,
                   const ignition::math::Vector3d& p3) :
      p0_(p0), p1_(p1), p2_(p2), p3_(p3) {
  }

  ignition::math::Vector3d InterpolateMthDerivative(int derivative_order,
                                                      double t) const {
    t = std::max(0.0, std::min(t, 1.0));
    switch(derivative_order) {
      case 0:
        ignition::math::Matrix4d matrix(1.0, 0.0, 0.0, 0.0,
                                        0.0, 1.0, 0.0, 0.0,
                                        -3.0, -2.0, 3.0, -1.0,
                                        2.0, 1.0, -2.0, 1.0);
        ignition::math::Matrix4d p_vector(p0_.X(), p0_.Y(), p0_.Z(), 0.0,
                                          p1_.X(), p1_.Y(), p1_.Z(), 0.0,
                                          p2_.X(), p2_.Y(), p2_.Z(), 0.0,
                                          p3_.X(), p3_.Y(), p3_.Z(), 0.0);
        ignition::math::Vector4d t_vector(1, t, t * t, t * t * t);
        ignition::math::Vector4d result = t_vector * (matrix * p_vector);
        return ignition::math::Vector3d(result.X(), result.Y(), result.Z());
      break;
    }
    return ignition::math::Vector3d();
  }

 private:
  const ignition::math::Vector3d p0_;
  const ignition::math::Vector3d p1_;
  const ignition::math::Vector3d p2_;
  const ignition::math::Vector3d p3_;
};
*/
/*
class MatrixBSpline {
 public:

  explicit MatrixBSpline(const ignition::math::Vector3d& p0,
                   const ignition::math::Vector3d& p1,
                   const ignition::math::Vector3d& p2,
                   const ignition::math::Vector3d& p3) :
      p0_(p0), p1_(p1), p2_(p2), p3_(p3) {
  }

  ignition::math::Vector3d InterpolateMthDerivative(int derivative_order,
                                                      double t) const {
    t = std::max(0.0, std::min(t, 1.0));
    switch(derivative_order) {
      case 0:
        ignition::math::Matrix4d matrix(1.0 / 6.0, 3.0/ 6.0, -3.0/ 6.0, 1.0/ 6.0,
                                        3.0/ 6.0, -6.0/ 6.0, 3.0/ 6.0, 0.0/ 6.0,
                                        -3.0/ 6.0, 0.0/ 6.0, 3.0/ 6.0, 0.0/ 6.0,
                                        1.0/ 6.0, 4.0/ 6.0, 1.0/ 6.0, 0.0/ 6.0);
        ignition::math::Matrix4d p_vector(p0_.X(), p0_.Y(), p0_.Z(), 0.0,
                                          p1_.X(), p1_.Y(), p1_.Z(), 0.0,
                                          p2_.X(), p2_.Y(), p2_.Z(), 0.0,
                                          p3_.X(), p3_.Y(), p3_.Z(), 0.0);
        ignition::math::Vector4d t_vector(1, t, t * t, t * t * t);
        ignition::math::Vector4d result = t_vector * (matrix * p_vector);
        return ignition::math::Vector3d(result.X(), result.Y(), result.Z());
      break;
    }
    return ignition::math::Vector3d();
  }

 private:
  const ignition::math::Vector3d p0_;
  const ignition::math::Vector3d p1_;
  const ignition::math::Vector3d p2_;
  const ignition::math::Vector3d p3_;
};
*/
/*
std::vector<ignition::math::Vector3d> SplineToBezier(
    const std::vector<ignition::math::Vector3d>& cp) {
    ignition::math::Matrix4d bmatrix(1.0, 0.0, 0.0, 0.0,
                                -3.0, 3.0, 0.0, 0.0,
                                3.0, -6.0, 3.0, 0.0,
                                -1.0, 3.0, -3.0, 1.0);
    ignition::math::Matrix4d hmatrix(1.0, 0.0, 0.0, 0.0,
                                0.0, 1.0, 0.0, 0.0,
                                -3.0, -2.0, 3.0, -1.0,
                                2.0, 1.0, -2.0, 1.0);
    ignition::math::Matrix4d points(cp[0].X(), cp[0].Y(), cp[0].Z(), 0.0,
        cp[1].X(), cp[1].Y(), cp[1].Z(), 0.0,
        cp[2].X(), cp[2].Y(), cp[2].Z(), 0.0,
        cp[3].X(), cp[3].Y(), cp[3].Z(), 0.0);
    ignition::math::Matrix4d rm = bmatrix.Inverse() * hmatrix * points;
    std::vector<ignition::math::Vector3d> result;
    result.push_back(ignition::math::Vector3d(rm(0,0), rm(0,1), rm(0,2)));
    result.push_back(ignition::math::Vector3d(rm(1,0), rm(1,1), rm(1,2)));
    result.push_back(ignition::math::Vector3d(rm(2,0), rm(2,1), rm(2,2)));
    result.push_back(ignition::math::Vector3d(rm(3,0), rm(3,1), rm(3,2)));
    return result;
}
*/
/*
std::vector<ignition::math::Vector3d> BezierToSpline(
    const std::vector<ignition::math::Vector3d>& cp) {
    ignition::math::Matrix4d bmatrix(1.0, 0.0, 0.0, 0.0,
                                -3.0, 3.0, 0.0, 0.0,
                                3.0, -6.0, 3.0, 0.0,
                                -1.0, 3.0, -3.0, 1.0);
    ignition::math::Matrix4d hmatrix(1.0, 0.0, 0.0, 0.0,
                                0.0, 1.0, 0.0, 0.0,
                                -3.0, -2.0, 3.0, -1.0,
                                2.0, 1.0, -2.0, 1.0);
    ignition::math::Matrix4d points(cp[0].X(), cp[0].Y(), cp[0].Z(), 0.0,
        cp[1].X(), cp[1].Y(), cp[1].Z(), 0.0,
        cp[2].X(), cp[2].Y(), cp[2].Z(), 0.0,
        cp[3].X(), cp[3].Y(), cp[3].Z(), 0.0);
    ignition::math::Matrix4d rm = hmatrix.Inverse() * bmatrix * points;
    std::vector<ignition::math::Vector3d> result;
    result.push_back(ignition::math::Vector3d(rm(0,0), rm(0,1), rm(0,2)));
    result.push_back(ignition::math::Vector3d(rm(1,0), rm(1,1), rm(1,2)));
    result.push_back(ignition::math::Vector3d(rm(2,0), rm(2,1), rm(2,2)));
    result.push_back(ignition::math::Vector3d(rm(3,0), rm(3,1), rm(3,2)));
    return result;
}
*/
/*
std::vector<ignition::math::Vector3d> RemoveLoopsInBezier(
    const std::vector<ignition::math::Vector3d>& cp) {
  const ignition::math::Vector3d& p0 = cp[0];
  const ignition::math::Vector3d& p1 = cp[1];
  const ignition::math::Vector3d& p2 = cp[2];
  const ignition::math::Vector3d& p3 = cp[3];

  const ignition::math::Vector3d norm_t0 = (p1 - p0).Normalize();
  const ignition::math::Vector3d norm_t3 = (p3 - p2).Normalize();
  const ignition::math::Vector3d r = p3 - p0;

  const double original_k1 = (p1 - p0).Length();
  const double original_k3 = (p3 - p2).Length();

  const ignition::math::Vector3d norm_t3_x_r = norm_t3.Cross(r);
  const ignition::math::Vector3d norm_t3_x_norm_t0 = norm_t3.Cross(norm_t0);

  // Check if we have intersection between t0 and t3
  if (norm_t3_x_r.Length() == 0 || norm_t3_x_norm_t0.Length() == 0) {
    // Here we won't have intersection
    return cp;
  }

  const ignition::math::Vector3d l =
    (norm_t3_x_r.Length() / norm_t3_x_norm_t0.Length()) * norm_t0;

  double projection = norm_t3_x_r.Dot(norm_t3_x_norm_t0);
  ignition::math::Vector3d critical_point;
  if (projection >= 0.) {
    critical_point = p0 + l;
  } else {
    critical_point = p0 - l;
  }

  const double critical_k1 = (critical_point - p0).Length();
  const double critical_k2 = (critical_point - p3).Length();

  std::vector<ignition::math::Vector3d> result;
  if (std::abs(critical_k1) < std::abs(original_k1) ||
    std::abs(critical_k2) < std::abs(original_k3)) {
    result.push_back(p0);
    result.push_back(critical_point);
    result.push_back(critical_point);
    result.push_back(p3);
  } else {
    result.insert(result.end(), cp.begin(), cp.end());
  }
  return result;
}
*/
int main(int argc, char* argv[]) {
  /*
  ignition::math::Vector3d p0(0.0, 0.0, 0.0);
  ignition::math::Vector3d p1(0.0, 10.0, 0.0);
  ignition::math::Vector3d p2(0.0, 10.0, 0.0);
  ignition::math::Vector3d p3(10.0, 10.0, 0.0);
  ignition::math::Vector3d t0(0.0, 50.0, 0.0);
  ignition::math::Vector3d t3(50.0, 0.0, 0.0);
  */
  ignition::math::Vector3d p0(28.8984, 40.9639, 0);
  ignition::math::Vector3d p1(29.1009, 28.3639, 0);
  ignition::math::Vector3d p2(29.0969, 36.9484, 0);
  ignition::math::Vector3d p3(28.9852, 36.6344, 0);
  /*
  // We generate a SplineLane and then print all the values from it.
  {
    MatrixHermiteSpline curve(p0, t0, p3, t3);

    // Interpolation
    for (double t = 0.0;  t <= 1.0; t += 0.01) {
      std::cout << curve.InterpolateMthDerivative(0, t) << std::endl;
    }
    std::cout << curve.InterpolateMthDerivative(0, 1.0) << std::endl;

    std::cout << "-----------" << std::endl;
  }

  std::vector<ignition::math::Vector3d> points =
    SplineToBezier(std::vector<ignition::math::Vector3d>{p0, t0, p3, t3});
    */
  // Now we generate the same curve but as Bezier
  {
    // MatrixCubicBezier curve(points[0], points[1], points[2], points[3]);
    MatrixCubicBezier curve(p0, p1, p2, p3);

    // Interpolation
    for (double t = 0.0;  t <= 1.0; t += 0.01) {
      std::cout << curve.InterpolateMthDerivative(0, t) << std::endl;
    }
    std::cout << curve.InterpolateMthDerivative(0, 1.0) << std::endl;

    std::cout << "-----------" << std::endl;
  }
  /*
  points = RemoveLoopsInBezier(points);
  // Now we generate a Bezier curve but without loops or change in curvature.
  {
    MatrixCubicBezier curve(points[0], points[1], points[2], points[3]);

    // Interpolation
    for (double t = 0.0;  t <= 1.0; t += 0.01) {
      std::cout << curve.InterpolateMthDerivative(0, t) << std::endl;
    }
    std::cout << curve.InterpolateMthDerivative(0, 1.0) << std::endl;

    std::cout << "-----------" << std::endl;
  }

  points = BezierToSpline(points);
  // Now we generate a Spline without cusps.
  {
    MatrixHermiteSpline curve(points[0], points[1], points[2], points[3]);

    // Interpolation
    for (double t = 0.0;  t <= 1.0; t += 0.01) {
      std::cout << curve.InterpolateMthDerivative(0, t) << std::endl;
    }
    std::cout << curve.InterpolateMthDerivative(0, 1.0) << std::endl;

    std::cout << "-----------" << std::endl;
  }
  */

  return 0;
}

}
}
}
}

int main (int argc, char* argv[]) {
  return drake::examples::bezier::main(argc, argv);
}

