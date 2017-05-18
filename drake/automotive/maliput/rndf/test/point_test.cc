#include <memory>
#include <string>
#include <tuple>
#include <vector>
#include <iostream>

#include <gtest/gtest.h>

#include <ignition/math/Vector3.hh>


ignition::math::Vector3d Average(
  const std::vector<ignition::math::Vector3d> &points) {
  ignition::math::Vector3d sum;
  if (points.size() == 0)
    return ignition::math::Zero;
  for (const auto &point : points) {
    sum += point;
  }
  return sum / static_cast<double>(point.size());
}

GTEST_TEST(RNDFBuilder, FindPoint) {
  ignition::math::Vector3d g_l_1(0., 5., 0.);
  ignition::math::Vector3d t_1 = ignition::math::Vector3d(0., 10., 0.) -
    ignition::math::Vector3d(0., 0., 0.);
  t_1.Normalize();
  ignition::math::Vector3d n_1(t_1.Y(), t_1.X(), 0.);

  ignition::math::Vector3d g_l_0_a(5., 0., 0.);
  ignition::math::Vector3d g_l_0_b(10., 10., 0.);
  ignition::math::Vector3d t_0 = (g_l_0_b - g_l_0_a);
  t_0.Normalize();

  double lambda_l_1 =
    ( g_l_1.X() - g_l_0_a.X() - (g_l_1.Y() - g_l_0_a.Y()) / t_0.Y() * t_0.X() ) /
    ( t_0.X() * n_1.Y() / t_0.Y() - n_1.X());

  double lambda_l_0 = (g_l_1.Y() - g_l_0_a.Y()) / t_0.Y() + lambda_l_1 * n_1.Y() / t_0.Y();

  std::cout << "Point we are looking for is: " << g_l_0_a + t_0 * lambda_l_0 << std::endl;
}

GTEST_TEST(RNDFBuilder, FindPoint2) {
  ignition::math::Vector3d g_l_1(-2.5, 5., 0.);
  ignition::math::Vector3d t_1 = ignition::math::Vector3d(-5., 10., 0.) -
    ignition::math::Vector3d(0., 0., 0.);
  t_1.Normalize();
  ignition::math::Vector3d n_1(t_1.Y(), t_1.X(), 0.);

  ignition::math::Vector3d g_l_0_a(5., 0., 0.);
  ignition::math::Vector3d g_l_0_b(10., 10., 0.);
  ignition::math::Vector3d t_0 = (g_l_0_b - g_l_0_a);
  t_0.Normalize();

  double lambda_l_1 =
    ( g_l_1.X() - g_l_0_a.X() - (g_l_1.Y() - g_l_0_a.Y()) / t_0.Y() * t_0.X() ) /
    ( t_0.X() * n_1.Y() / t_0.Y() - n_1.X());

  double lambda_l_0 = (g_l_1.Y() - g_l_0_a.Y()) / t_0.Y() + lambda_l_1 * n_1.Y() / t_0.Y();

  std::cout << "Point we are looking for is: " << g_l_0_a + t_0 * lambda_l_0 << std::endl;
}