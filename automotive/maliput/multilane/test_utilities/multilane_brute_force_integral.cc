#include "drake/automotive/maliput/multilane/test_utilities/multilane_brute_force_integral.h"

#include <cmath>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace test {

double PathLengthIntegral(const RoadCurve& rc, double p_0, double p_1, double r,
                          double h, int k_order) {
  DRAKE_DEMAND(p_0 >= 0. && p_0 <= 1.);
  DRAKE_DEMAND(p_1 >= 0. && p_1 <= 1.);
  DRAKE_DEMAND(p_0 <= p_1);
  DRAKE_DEMAND(k_order >= 0);
  const int iterations = std::pow(2, k_order);
  double length{0.};
  Vector3<double> old_geo_position = rc.W_of_prh(p_0, r, h);
  const double d_p = p_1 - p_0;
  for (int i = 1; i <= iterations; ++i) {
    const double p =
        p_0 + d_p * static_cast<double>(i) / static_cast<double>(iterations);
    const Vector3<double> geo_position_p = rc.W_of_prh(p, r, h);
    length += (geo_position_p - old_geo_position).norm();
    old_geo_position = geo_position_p;
  }
  return length;
}

int MinimumKOrder(const RoadCurve& rc, double p_0, double p_1, double r,
                  double h, double tolerance, int k_order_hint) {
  DRAKE_DEMAND(tolerance >= 0.);
  DRAKE_DEMAND(k_order_hint >= 0);
  int k_order{k_order_hint};
  while (1) {
    const double difference =
      std::abs(PathLengthIntegral(rc, p_0, p_1, r, h, k_order) -
               PathLengthIntegral(rc, p_0, p_1, r, h, k_order + 1));
    if (difference < tolerance) { break; }
    ++k_order;
  }
  return k_order;
}

}  // namespace test
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
