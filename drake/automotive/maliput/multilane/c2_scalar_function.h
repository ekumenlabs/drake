#pragma once

#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace multilane {

template<typename T> class C2ScalarFunction {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(C2ScalarFunction)
  C2ScalarFunction() = default;
  virtual ~C2ScalarFunction() = default;
  virtual T f_p(T p) const = 0;
  virtual T f_dot_p(T p) const = 0;
  virtual T f_dot_dot_p(T p) const = 0;
  virtual T s_p(T p) const = 0;
  virtual T p_s(T s) const = 0;
  virtual void scale(T scale_0, T scale_1) = 0;
};


}  // namespace multilane
}  // namespace maliput
}  // namespace drake
