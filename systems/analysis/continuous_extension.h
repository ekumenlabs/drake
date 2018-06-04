#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/// An interface for continuous extension of ODE and DAE solutions, to
/// efficiently approximate them in between integration steps when solving
/// them numerically (see IntegratorBase class documentation).
///
/// This continuous extension concept can be formally stated as follows: given
/// a solution 𝐱(t) ∈ ℝⁿ to an ODE or DAE system that is known at a discrete
/// set of points {t₁ ... tₚ} where tₚ ∈ ℝ (e.g. as a result of numerical
/// integration), a continuous extension of 𝐱(t) is another function  𝐰(t) ∈ ℝⁿ
/// defined for t ∈ [t₁, tₚ] such that 𝐰(tᵢ) = 𝐱(tᵢ) for every
/// tᵢ ∈ {t₁ ... tₚ} and that approximates 𝐱(t) for the value in between.
///
/// @tparam T A valid Eigen scalar type.
template <typename T>
class ContinuousExtension {
 public:
  virtual ~ContinuousExtension() {}

  /// Evaluates extension at the given time @p t.
  /// @param t Time to evaluate extension at.
  /// @return Extension vector value.
  /// @pre Extension is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions are not met.
  /// @throw std::runtime_error if the extension is not defined for the
  ///                           given @p t.
  virtual VectorX<T> Evaluate(const T& t) const = 0;

  /// Evaluates extension at the given time @p t.
  /// @param t Time to evaluate extension at.
  /// @param dimension Dimension to evaluate.
  /// @return Extension scalar value.
  /// @pre Extension is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions is not met.
  /// @throw std::runtime_error if the extension is not defined for the
  ///                           given @p t.
  /// @throw std::runtime_error if given @p dimension is not valid
  ///                           i.e. 0 <= @p dimension < get_dimensions().
  virtual T Evaluate(const T& t, int dimension) const = 0;

  /// Returns the extension dimensions i.e. of its vector value.
  /// @pre Extension is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions is not met.
  virtual int get_dimensions() const = 0;

  /// Checks whether the extension is empty or not.
  virtual bool is_empty() const = 0;

  /// Returns extension's start time i.e. the oldest time `t`
  /// that it can be evaluated at e.g. via Evaluate().
  /// @pre Extension is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions is not met.
  virtual const T& get_start_time() const = 0;

  /// Returns extension's end time i.e. the newest time `t`
  /// that it can be evaluated at e.g. via Evaluate().
  /// @pre Extension is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions is not met.
  virtual const T& get_end_time() const = 0;
};

}  // namespace systems
}  // namespace drake
