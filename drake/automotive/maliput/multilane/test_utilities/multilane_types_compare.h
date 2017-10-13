#pragma once

#include <gtest/gtest.h>

#include "drake/automotive/maliput/multilane/builder.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace test {

::testing::AssertionResult IsEndpointXyClose(const EndpointXy& xy1,
                                             const EndpointXy& xy2,
                                             double tolerance);

::testing::AssertionResult IsEndpointZClose(const EndpointZ& z1,
                                            const EndpointZ& z2,
                                            double tolerance);

::testing::AssertionResult IsEndpointClose(const Endpoint& pos1,
                                           const Endpoint& pos2,
                                           double tolerance);
}  // namespace test
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
