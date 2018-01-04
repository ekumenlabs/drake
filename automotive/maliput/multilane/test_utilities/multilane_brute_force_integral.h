#pragma once

#include <tuple>

#include "drake/automotive/maliput/multilane/road_curve.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace test {

double PathLengthIntegral(const RoadCurve& rc, double p_0, double p_1, double r,
                          double h, int k_order);

int MinimumKOrder(const RoadCurve& rc, double p_0, double p_1, double r,
                  double h, double tolerance, int k_order_hint);

}  // namespace test
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
