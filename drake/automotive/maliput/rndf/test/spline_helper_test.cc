#include <cmath>
#include <iostream>
#include <memory>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>


#include "ignition/math/Spline.hh"
#include "ignition/math/Vector3.hh"

#include "drake/automotive/maliput/rndf/spline_helpers.h"

namespace drake {
namespace maliput {
namespace rndf {

const double kLinearTolerance = 1e-4;
const double kLinearToleranceProportionForModule = 0.05;

#define EXPECT_IGN_VECTOR_NEAR(actual, expected, tolerance)         \
  do {                                                       \
    const ignition::math::Vector3d _actual(actual);                  \
    const ignition::math::Vector3d _expected(expected);               \
    const double _tolerance(tolerance);                   	\
    EXPECT_NEAR(_actual.X(), _expected.X(), _tolerance);         \
    EXPECT_NEAR(_actual.Y(), _expected.Y(), _tolerance);         \
    EXPECT_NEAR(_actual.Z(), _expected.Z(), _tolerance);         \
  } while (0)

std::unique_ptr<ignition::math::Spline>
CreateSpline(
  const std::vector<
    std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>> &points) {
  std::unique_ptr<ignition::math::Spline> spline =
    std::make_unique<ignition::math::Spline>();
  spline->Tension(1.0);
  spline->AutoCalculate(true);
  for (const auto &point : points) {
    spline->AddPoint(std::get<0>(point), std::get<1>(point));
  }
  return spline;
}

GTEST_TEST(RNDFSplineHelperTest, StraightLine) {
  std::vector<
    std::tuple<ignition::math::Vector3d,ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(0.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(20.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));

  std::unique_ptr<ignition::math::Spline> spline =
  	CreateSpline(control_points);
  std::unique_ptr<ArcLengthParameterizedSpline> arc_lenght_param_spline =
  	std::make_unique<ArcLengthParameterizedSpline>(std::move(spline), 500);

  const double length = arc_lenght_param_spline->BaseSpline()->ArcLength();
  ignition::math::Vector3d p(0.0, 0.0, 0.0);
  for (double l = 0.0; l < length; l += 1.0) {
  	p.X() = l;
    EXPECT_IGN_VECTOR_NEAR(arc_lenght_param_spline->InterpolateMthDerivative(0, l), p, kLinearTolerance);
  }
}

GTEST_TEST(RNDFSplineHelperTest, ConstantTangentModule) {
  std::vector<
    std::tuple<ignition::math::Vector3d,ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(0.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(20.0, 0.0, 0.0),
      ignition::math::Vector3d(-10.0, 0.0, 0.0)));

  std::unique_ptr<ignition::math::Spline> spline =
  	CreateSpline(control_points);
  std::unique_ptr<ArcLengthParameterizedSpline> arc_lenght_param_spline =
  	std::make_unique<ArcLengthParameterizedSpline>(std::move(spline), 100);

  const double length = arc_lenght_param_spline->BaseSpline()->ArcLength();
  const double tangent_module = arc_lenght_param_spline->InterpolateMthDerivative(1, length/2.0).Length();
  const double tangent_module_tolerance = tangent_module * kLinearToleranceProportionForModule;
  ignition::math::Vector3d p;
  for (double l = 0.0; l < length; l += 1.0) {
  	p.X() = l;
  	const auto tangent = arc_lenght_param_spline->InterpolateMthDerivative(1, l);
    EXPECT_NEAR(tangent.Length(), tangent_module, tangent_module_tolerance);
  }
}


} // rndf
} // maliput
} // drake