#pragma once

namespace drake {
namespace systems {

/// An interface for continuous extension of scalar ODE and DAE solutions.
///
/// See ContinuousExtension class documentation for further details.
///
/// @tparam T A valid Eigen scalar type.
template <typename T>
class ScalarContinuousExtension {
 public:
  virtual ~ScalarContinuousExtension() {}

  /// Evaluates extension at the given time @p t.
  /// @param t Time to evaluate extension at.
  /// @return Extension scalar value.
  /// @pre Extension is not empty i.e. is_empty() is false.
  /// @throw std::logic_error if any of the preconditions is not met.
  /// @throw std::runtime_error if the extension is not defined for the
  ///                           given @p t.
  virtual T Evaluate(const T& t) const = 0;

  /// Checks whether the extension is empty or not.
  virtual bool is_empty() const = 0;

  /// Returns extension's start time i.e. the oldest time `t`
  /// that it can be evaluated at e.g. via Evaluate().
  /// @pre Extension is not empty i.e. is_empty() is false..
  /// @throw std::logic_error if any of the preconditions is not met.
  virtual const T& get_start_time() const = 0;

  /// Returns extension's end time i.e. the newest time `t`
  /// that it can be evaluated at e.g. via Evaluate().
  /// @pre Extension is not empty.
  /// @throw std::logic_error if any of the preconditions is not met.
  virtual const T& get_end_time() const = 0;
};


}  // namespace systems
}  // namespace drake
