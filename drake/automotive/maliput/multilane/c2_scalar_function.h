#pragma once

#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace multilane {

/// Abstract class to define the interface of a scalar function up to the second
/// derivative. p is the time parameter and s represents the path length.
/// Conversion (inverse function) between p and s should be provided by the
/// implementation.
template<typename T> class C2ScalarFunction {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(C2ScalarFunction)
  /// Constructor.
  C2ScalarFunction() = default;

  /// Destructor.
  virtual ~C2ScalarFunction() = default;

  /// Interpolates the function at @p p parameter.
  /// @param p The parameter value of the function.
  /// @return The image of the function at @p p.
  virtual T f_p(T p) const = 0;

  /// Interpolates the function's first derivative at @p p parameter.
  /// @param p The parameter value of the function.
  /// @return The image of the function's first derivative at @p p.
  virtual T f_dot_p(T p) const = 0;

  /// Interpolates the function's second derivative at @p p parameter.
  /// @param p The parameter value of the function.
  /// @return The image of the function's second derivative at @p p.
  virtual T f_dot_dot_p(T p) const = 0;

  /// Path length integral from @p p = 0 to @p p.
  /// @param p The parameter value to define the integration limit.
  /// @return The image of the function's second derivative at @p p.
  virtual T s_p(T p) const = 0;

  /// Path length integral from p = 0 to @p p.
  /// @param p The parameter value to define the integration limit.
  /// @return The function's path length integral from p = 0 to @p p.
  virtual T p_s(T s) const = 0;
};


}  // namespace multilane
}  // namespace maliput
}  // namespace drake
