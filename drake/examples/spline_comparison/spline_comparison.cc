#include <iostream>
#include <vector>
#include <tuple>

#include "ignition/math/Vector3.hh"
#include "ignition/math/Spline.hh"

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace examples {
namespace splines_comparison {
namespace {

typedef Vector2<double> V2;
typedef Vector3<double> V3;

// std::vector<std::tuple<V3, V3>> BuildControlPointsForLine() {
//   std::vector<std::tuple<V3, V3>> control_points;

//   // Straight line
//   control_points.push_back(
//       std::make_tuple<V3, V3>(V3(0., 0., 0.), V3(20., 0., 0.)));
//   control_points.push_back(
//       std::make_tuple<V3, V3>(V3(20., 0., 0.), V3(20., 0., 0.)));

//   return control_points;
// }

// std::vector<V3> ReadControlPointsFromFile() {
//   std::ifstream file("/home/agalbachicar/Downloads/MapNodes.csv");
//   std::string line;
//   char sep = ',';
//   std::vector<V3> points;
//   while (file.good()) {
//     // Read the line from the file
//     std::getline(file, line,'\n');
//     // Split the line
//     std::vector<double> values;
//     for(size_t p=0, q=0, i = 0; p != line.npos; p=q, i++) {
//       std::string part = line.substr(p + (p!=0),
//                                        (q = line.find(sep, p + 1)) - p - (p != 0));
//       if (part.empty())
//         return points;
//       values.push_back(std::stod(part));
//     }
//     points.push_back(V3(values[0], values[1], values[2]));
//   }
//   return points;
// }

// std::vector<std::tuple<V3, V3>> BuildControlPointsForFreeFormCurve() {
//   std::vector<std::tuple<V3, V3>> control_points;

//   // Straight line
//   control_points.push_back(
//       std::make_tuple<V3, V3>(V3(0., 0., 0.), V3(0., 20., 0.)));
//   control_points.push_back(
//       std::make_tuple<V3, V3>(V3(20., 20., 0.), V3(20., 0., 0.)));
//   control_points.push_back(
//       std::make_tuple<V3, V3>(V3(40., 0., 0.), V3(0., -20., 0.)));

//   return control_points;
// }

// std::vector<std::tuple<V3, V3>> BuildControlPointsForLoopCurve() {
//   std::vector<std::tuple<V3, V3>> control_points;

//   // Straight line
//   control_points.push_back(
//       std::make_tuple<V3, V3>(V3(0., 0., 0.), V3(0., 30., 0.)));
//   control_points.push_back(
//       std::make_tuple<V3, V3>(V3(0, 1, 0), V3(0, 3, 0)));
//   control_points.push_back(
//       std::make_tuple<V3, V3>(V3(0, 10, 0), V3(0, 3, 0)));

//   return control_points;
// }

ignition::math::Vector3d FromV3(const V3& point) {
  return ignition::math::Vector3d(point.x(), point.y(), point.z());
}
V3 FromVector3d(const ignition::math::Vector3d& point) {
  return V3(point.X(), point.Y(), point.Z());
}

// std::vector<V3> DoInterpolateIgnitionSpline(
//     const std::vector<std::tuple<V3, V3>>& control_points, double step,
//     int derivative_order) {
//   // Create the Spline and then add the points + tangents
//   ignition::math::Spline spline;
//   spline.AutoCalculate(true);
//   for (const auto& point : control_points) {
//     spline.AddPoint(FromV3(std::get<0>(point)), FromV3(std::get<1>(point)));
//   }
//   // Iterate and run from 0 --> 1 at constant time step
//   std::vector<V3> interpolated_points;
//   double t{0.0};
//   while (t < 1.0) {
//     interpolated_points.push_back(
//         FromVector3d(spline.InterpolateMthDerivative(derivative_order, t)));
//     t += step;
//   }
//   if (t > 1.0) {
//     interpolated_points.push_back(
//         FromVector3d(spline.InterpolateMthDerivative(derivative_order, 1.0)));
//   }
//   return interpolated_points;
// }

// std::vector<V3> DoInterpolateDrakeSpline(
//     const std::vector<std::tuple<V3, V3>>& control_points, double step,
//     int derivative_order) {
//   std::vector<V3> interpolated_points;
//   std::vector<double> breaks;
//   std::vector<MatrixX<double>> knots(control_points.size(),
//                                        MatrixX<double>::Zero(3, 1));
//   std::vector<MatrixX<double>> knots_dot(control_points.size(),
//                                        MatrixX<double>::Zero(3, 1));
//   double t = 0.0;
//   for (int i = 0; i < static_cast<int>(control_points.size()); i++) {
//     const V3 point = std::get<0>(control_points[i]);
//     knots[i] << point.x(), point.y(), point.z();
//     const V3 tangent = std::get<1>(control_points[i]);
//     knots_dot[i] << tangent.x(), tangent.y(), tangent.z();
//     breaks.push_back(t);
//     t += 1.0;
//   }

//   PiecewisePolynomial<double> polynomial =
//       PiecewisePolynomial<double>::Pchip(breaks, knots, true);

//   PiecewisePolynomial<double> derivated_polynomial =
//       polynomial.derivative(derivative_order);

//   const double new_step = (t - 1.0) * step;
//   double tt {0.0};
//   while(tt < (t - 1.0)) {
//     auto result = derivated_polynomial.value(tt);
//     interpolated_points.push_back(derivated_polynomial.value(tt));
//     tt += new_step;
//   }
//   if (tt > (t - 1.0)) {
//     auto result = derivated_polynomial.value(t - 1.0);
//     interpolated_points.push_back(derivated_polynomial.value(t - 1.0));
//   }

//   return interpolated_points;
// }

// std::vector<V3> DoInterpolateDrakePChip(
//     const std::vector<std::tuple<V3, V3>>& control_points, double step,
//     int derivative_order) {
//   std::vector<V3> interpolated_points;
//   std::vector<double> breaks;
//   std::vector<MatrixX<double>> knots(control_points.size(),
//                                        MatrixX<double>::Zero(3, 1));
//   double t = 0.0;
//   for (int i = 0; i < static_cast<int>(control_points.size()); i++) {
//     const V3 point = std::get<0>(control_points[i]);
//     knots[i] << point.x(), point.y(), point.z();
//     breaks.push_back(t);
//     t += 1.0;
//   }

//   PiecewisePolynomial<double> polynomial =
//       PiecewisePolynomial<double>::Pchip(breaks, knots, true);
//   PiecewisePolynomial<double> derivated_polynomial = polynomial.derivative(
//       derivative_order);
//   const double new_step = (t - 1.0) * step;
//   double tt {0.0};
//   while(tt < (t - 1.0)) {
//     auto result = derivated_polynomial.value(tt);
//     interpolated_points.push_back(derivated_polynomial.value(tt));
//     tt += new_step;
//   }
//   if (tt > (t - 1.0)) {
//     auto result = derivated_polynomial.value(t - 1.0);
//     interpolated_points.push_back(derivated_polynomial.value(t - 1.0));
//   }

//   return interpolated_points;
// }


// std::vector<V3> DoInterpolateDrakePChip(
//     const std::vector<V3>& control_points, double step,
//     int derivative_order) {
//   std::vector<V3> interpolated_points;
//   std::vector<double> breaks;
//   std::vector<MatrixX<double>> knots(control_points.size(),
//                                        MatrixX<double>::Zero(3, 1));
//   double t = 0.0;
//   for (int i = 0; i < static_cast<int>(control_points.size()); i++) {
//     const V3 point = control_points[i];
//     knots[i] << point.x(), point.y(), point.z();
//     breaks.push_back(t);
//     t += 1.0;
//   }

//   PiecewisePolynomial<double> polynomial =
//       PiecewisePolynomial<double>::Pchip(breaks, knots, true);
//   PiecewisePolynomial<double> derivated_polynomial = polynomial.derivative(
//       derivative_order);
//   const double new_step = (t - 1.0) * step;
//   double tt {0.0};
//   while(tt < (t - 1.0)) {
//     auto result = derivated_polynomial.value(tt);
//     interpolated_points.push_back(derivated_polynomial.value(tt));
//     tt += new_step;
//   }
//   if (tt > (t - 1.0)) {
//     auto result = derivated_polynomial.value(t - 1.0);
//     interpolated_points.push_back(derivated_polynomial.value(t - 1.0));
//   }

//   return interpolated_points;
// }

PiecewisePolynomial<double> ConstructPChip(const std::vector<V3>& control_points) {
  std::vector<V3> interpolated_points;
  std::vector<double> breaks;
  std::vector<MatrixX<double>> knots(control_points.size(),
                                       MatrixX<double>::Zero(3, 1));
  double t = 0.0;
  for (int i = 0; i < static_cast<int>(control_points.size()); i++) {
    const V3 point = control_points[i];
    knots[i] << point.x(), point.y(), point.z();
    breaks.push_back(t);
    t += 1.0;
  }

  return PiecewisePolynomial<double>::Pchip(breaks, knots, true);
}

void PrintPoint(const V3& p) {
  std::cout.precision(20);
  std::cout << p.x() << "," << p.y() << "," << p.z() << std::endl;
}

void PrintVector(const std::string& prefix, const std::string& suffix,
    const std::vector<V3>& v) {
  std::cout.precision(20);
  std::cout << prefix << std::endl;
  for (const auto &p : v) {
    PrintPoint(p);
  }
  std::cout << suffix << std::endl;
}

// std::vector<V3> CalculateDifference(const std::vector<V3>& v1,
//                                     const std::vector<V3>& v2) {
//   std::vector<V3> difference;
//   for (int i = 0; i < static_cast<int>(v1.size()); i++) {
//     difference.push_back(v1[i] - v2[i]);
//   }
//   return difference;
// }


int main(int, char **) {
  const double kInterpolationStep{1e-2};
  /*
  for (int i = 0; i <= 3; i++) {
    std::vector<std::tuple<V3, V3>> control_points = BuildControlPointsForLine();
    std::vector<V3> ignition_interpolation = DoInterpolateIgnitionSpline(
        control_points, kInterpolationStep, i);
    std::vector<V3> drake_interpolation = DoInterpolateDrakeSpline(
        control_points, kInterpolationStep, i);
    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "[Linear]: derivative_order: " << i << std::endl;
    PrintVector(std::string("ignition_v:["), std::string("]"),
                ignition_interpolation);
    PrintVector(std::string("drake_v:["), std::string("]"),
                drake_interpolation);
    PrintVector(std::string("difference:["), std::string("]"),
                CalculateDifference(ignition_interpolation,
                                    drake_interpolation));
    std::cout << "---------------------------------------------" << std::endl;
  }

  for (int i = 0; i <= 3; i++) {
    std::vector<std::tuple<V3, V3>> control_points =
      BuildControlPointsForFreeFormCurve();
    std::vector<V3> ignition_interpolation = DoInterpolateIgnitionSpline(
        control_points, kInterpolationStep, i);
    std::vector<V3> drake_interpolation = DoInterpolateDrakeSpline(
        control_points, kInterpolationStep, i);
    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "[Free form curve]: derivative_order: " << i << std::endl;
    PrintVector(std::string("ignition_v:["), std::string("]"),
        ignition_interpolation);
    PrintVector(std::string("drake_v:["), std::string("]"),
        drake_interpolation);
    PrintVector(std::string("difference:["), std::string("]"),
                CalculateDifference(ignition_interpolation,
                                    drake_interpolation));
    std::cout << "---------------------------------------------" << std::endl;
  }

  for (int i = 0; i <= 3; i++) {
    std::vector<std::tuple<V3, V3>> control_points =
      BuildControlPointsForLoopCurve();
    std::vector<V3> ignition_interpolation = DoInterpolateIgnitionSpline(
        control_points, kInterpolationStep, i);
    std::vector<V3> drake_interpolation = DoInterpolateDrakeSpline(
        control_points, kInterpolationStep, i);
    std::vector<V3> pchip_drake_interpolation = DoInterpolateDrakePChip(
        control_points, kInterpolationStep, i);
    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "[Loop]: derivative_order: " << i << std::endl;

    PrintVector(std::string("ignition_v:["), std::string("]"),
        ignition_interpolation);
    PrintVector(std::string("drake_v:["), std::string("]"),
        drake_interpolation);
    PrintVector(std::string("pchip_v:["), std::string("]"),
        pchip_drake_interpolation);
    std::cout << "---------------------------------------------" << std::endl;
  }
  */
  std::vector<V3> control_points;
  /*
  control_points.push_back(V3(0.0, 0.0, 0.0));
  control_points.push_back(V3(10.0, 10.0, 0.0));
  control_points.push_back(V3(20.0, 0.0, 0.0));
  */
  control_points.push_back(V3(0.0, 0.0, 0.0));
  control_points.push_back(V3(199.982, 0.000553, 0.));
  auto pchip = ConstructPChip(control_points);
  auto pchip_derivative = pchip.derivative(1);

  const auto start = pchip.value(0);
  const auto middle = pchip.value(1);
  // const auto end = pchip.value(2);
  const auto start_derivative = pchip_derivative.value(0);
  const auto middle_derivative = pchip_derivative.value(1);
  // const auto end_derivative = pchip_derivative.value(2);

  // Create the Spline and then add the points + tangents
  ignition::math::Spline spline;
  spline.AutoCalculate(true);
  spline.AddPoint(FromV3(start), FromV3(start_derivative));
  spline.AddPoint(FromV3(middle), FromV3(middle_derivative));
  // spline.AddPoint(FromV3(end), FromV3(end_derivative));

  // Interpolate both curves
  double step = kInterpolationStep;
  std::vector<V3> pchip_points, pchip_derivative_points;
  std::vector<V3> spline_points, spline_derivative_points;
  for (double t = 0; t < 1; t+= step) {
    pchip_points.push_back(pchip.value(t));
    pchip_derivative_points.push_back(pchip_derivative.value(t));
  }
  step = kInterpolationStep;
  for (double t = 0; t < 1; t+= step) {
    spline_points.push_back(FromVector3d(spline.InterpolateMthDerivative(0, t)));
    spline_derivative_points.push_back(FromVector3d(spline.InterpolateMthDerivative(1, t)));
  }

  std::cout << "---------------------------------------------" << std::endl;
  PrintVector(std::string("pchip_v:["), std::string("]"), pchip_points);
  PrintVector(std::string("spline_v:["), std::string("]"), spline_points);
  std::cout << "---------------------------------------------" << std::endl;
  std::cout << "---------------------------------------------" << std::endl;
  PrintVector(std::string("pchip_der_v:["), std::string("]"), pchip_derivative_points);
  PrintVector(std::string("spline_der_v:["), std::string("]"), spline_derivative_points);
  std::cout << "---------------------------------------------" << std::endl;
  return 0;
}

}  // manespace
}  // namespace splines_comparison
}  // namespace examples
}  // namespace drake

int main(int argc, char **argv) {
  return drake::examples::splines_comparison::main(argc, argv);
}