#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/analysis/stepwise_continuous_extension.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

namespace detail {

/// Copies a potentially non flat @p vector into a flat column matrix.
/// @param vector A vector to be copied.
/// @return The copied column matrix.
/// @tparam T Vector elements' type, which must be a valid Eigen scalar type.
template <typename T>
MatrixX<T> CopyToColumnMatrix(const VectorBase<T>& vector) {
  MatrixX<T> column_matrix(vector.size(), 1);
  for (int i = 0 ; i < vector.size() ; ++i) {
    column_matrix(i, 0) = vector.GetAtIndex(i);
  }
  return std::move(column_matrix);
}

/// A class providing a basic set of operations to convert to/from
/// `scalar` type values from/to a `double` floating point type values.
///
/// @tparam T A valid Eigen scalar type.
template <typename T>
struct ScalarBaseConverter {
  /// Converts a given scalar type value to its
  /// double floating point type value equivalent.
  /// @param value Scalar value to be converted.
  /// @return Given @p value as a double.
  double ToDouble(const T& value) const {
    return double{value};
  }

  /// Converts a double floating point type value
  /// to its equivalent scalar type value.
  /// @param value Floating point value to be converted.
  /// @return Given @p value as a scalar type value.
  T FromDouble(double value) const {
    return T{value};
  }

  /// Converts a matrix using scalar type elements to its equivalent
  /// matrix using double floating point type elements.
  /// @param matrix Matrix to be converted.
  /// @return Given @p matrix using double floating type elements.
  MatrixX<double> ToDoubleMatrix(const MatrixX<T>& matrix) const {
    return matrix.template cast<double>();
  }

  /// Converts a matrix using double floating point type elements to
  /// its equivalent matrix using scalar type elements.
  /// @param matrix Matrix to be converted.
  /// @return Given @p matrix using scalar type elements.
  MatrixX<T> FromDoubleMatrix(const MatrixX<double>& matrix) const {
    return matrix.template cast<T>();
  }
};

/// A ScalarBaseConverter explicit specialization to deal
/// with Eigen::AutoDiffScalar values.
template <typename S>
struct ScalarBaseConverter<Eigen::AutoDiffScalar<S>> {
  double ToDouble(const Eigen::AutoDiffScalar<S>& scalar) const {
    return double{scalar.value()};
  }

  Eigen::AutoDiffScalar<S> FromDouble(double scalar) const {
    return Eigen::AutoDiffScalar<S>{scalar};
  }

  MatrixX<double> ToDoubleMatrix(
      const MatrixX<Eigen::AutoDiffScalar<S>>& matrix) const {
    MatrixX<double> result(matrix.rows(), matrix.cols());
    for (Eigen::Index i = 0; i < matrix.rows(); ++i) {
      for (Eigen::Index j = 0; j < matrix.cols(); ++j) {
        result(i, j) = matrix(i, j).value();
      }
    }
    return result;
  }

  MatrixX<Eigen::AutoDiffScalar<S>> FromDoubleMatrix(
      const MatrixX<double>& matrix) const {
    MatrixX<Eigen::AutoDiffScalar<S>> result(
        matrix.rows(), matrix.cols());
    for (Eigen::Index i = 0; i < matrix.rows(); ++i) {
      for (Eigen::Index j = 0; j < matrix.cols(); ++j) {
        result(i, j) = matrix(i, j);
      }
    }
    return result;
  }
};

/// A ScalarBaseConverter extension to deal with scalars in
/// common use data structures.
///
/// @tparam T A valid Eigen scalar type.
template <typename T>
struct ScalarConverter : public ScalarBaseConverter<T> {
  /// Converts a given vector of scalar type values to its
  /// equivalent vector of double floating point type values.
  /// @param scalar_vector A vector of scalar type values.
  /// @return Given @p scalar_vector using double floating point type values.
  std::vector<double> ToDoubleVector(
      const std::vector<T>& scalar_vector) const {
    using std::placeholders::_1;
    std::vector<double> scalar_vector_using_doubles{};
    scalar_vector_using_doubles.reserve(scalar_vector.size());
    std::transform(scalar_vector.begin(), scalar_vector.end(),
                   std::back_inserter(scalar_vector_using_doubles),
                   std::bind(&ScalarConverter<T>::ToDouble, this, _1));
    return scalar_vector_using_doubles;
  }

  /// Converts a given vector of matrices using scalar type elements to its
  /// equivalent vector of matrices using double floating point type elements.
  /// @param matrix_vector A vector of matrices using scalar type elements.
  /// @return Given @p matrix_vector using double floating point type elements.
  std::vector<MatrixX<double>> ToDoubleMatrixVector(
      const std::vector<MatrixX<T>>& matrix_vector) const {
    using std::placeholders::_1;
    std::vector<MatrixX<double>> matrix_vector_using_doubles{};
    matrix_vector_using_doubles.reserve(matrix_vector.size());
    std::transform(matrix_vector.begin(), matrix_vector.end(),
                   std::back_inserter(matrix_vector_using_doubles),
                   std::bind(&ScalarConverter<T>::ToDoubleMatrix, this, _1));
    return matrix_vector_using_doubles;
  }
};

/// A ScalarConverter explicit specialization to work as a passthrough
/// for double scalars.
template <>
struct ScalarConverter<double> : public ScalarBaseConverter<double> {
  std::vector<double> ToDoubleVector(
      const std::vector<double>& scalar_vector) const {
    return scalar_vector;
  }

  std::vector<MatrixX<double>> ToDoubleMatrixVector(
      const std::vector<MatrixX<double>>& matrix_vector) const {
    return matrix_vector;
  }
};

}  // namespace detail

/// An StepwiseContinuousExtension class implementation using Hermitian
/// interpolators. For every integration step, state 𝐱 and state time
/// derivative d𝐱/dt are required at both ends. Hermite cubic polynomials
/// are then constructed for each, yielding a C1 extension to the solution 𝐱(t).
///
/// Hermitian continuous extensions show the same error bound as that of the
/// integration scheme being used for up to 3rd order schemes (see
/// [Hairer, 1993]).
///
/// - [Hairer, 1993] E. Hairer, S. Nørsett and G. Wanner. Solving Ordinary
///                  Differential Equations I (Nonstiff Problems), p.190,
///                  Springer, 1993.
/// @tparam T A valid Eigen scalar type.
// TODO(hidmic): Better support AutoDiff scalars when PiecewisePolynomial
// fully supports them.
template <typename T>
class HermitianContinuousExtension : public StepwiseContinuousExtension<T> {
 public:
  /// An integration step representation class, holding just enough
  /// for hermitian interpolation i.e. three (3) related sets containing
  /// step times {t₀, ..., tᵢ₋₁, tᵢ} where tᵢ ∈ ℝ , step states
  /// {𝐱₀, ..., 𝐱ᵢ₋₁, 𝐱ᵢ} where 𝐱ᵢ∈ ℝⁿ,  and state derivatives
  /// {d𝐱/dt₀, ..., d𝐱/dtᵢ₋₁, d𝐱/dtᵢ}.where d𝐱/dtᵢ ∈ ℝⁿ.
  class IntegrationStep {
   public:
    /// Constructs a zero length step by copy.
    ///
    /// @param initial_time Initial time t₀ where the step starts.
    /// @param initial_state Initial state vector 𝐱₀ at @p initial_time
    ///                      as a column matrix.
    /// @param initial_state_derivative Initial state derivative vector
    ///                                 d𝐱/dt₀ at @p initial_time as a
    ///                                 column matrix.
    /// @throw std::runtime_error if:
    ///   - given @p initial_state 𝐱₀ is not a column matrix.
    ///   - given @p initial_state_derivative d𝐱/t₀ is not a column matrix.
    ///   - given @p initial_state 𝐱₀ and @p initial_state_derivative d𝐱/dt₀
    ///     do not match each other's dimension.
    explicit IntegrationStep(const T& initial_time,
                             const MatrixX<T>& initial_state,
                             const MatrixX<T>& initial_state_derivative) {
      ValidateOrThrow(initial_time, initial_state,
                      initial_state_derivative);
      times_.push_back(initial_time);
      states_.push_back(initial_state);
      state_derivatives_.push_back(initial_state_derivative);
    }

    /// Constructs a zero length step by move.
    ///
    /// @param initial_time Initial time t₀ where the step starts.
    /// @param initial_state Initial state vector 𝐱₀ at @p initial_time
    ///                      as a column matrix.
    /// @param initial_state_derivative Initial state derivative vector
    ///                                 d𝐱/dt₀ at @p initial_time as a
    ///                                 column matrix.
    /// @throw std::runtime_error if:
    ///   - given @p initial_state 𝐱₀ is not a column matrix.
    ///   - given @p initial_state_derivative d𝐱/t₀ is not a column matrix.
    ///   - given @p initial_state 𝐱₀ and @p initial_state_derivative d𝐱/dt₀
    ///     do not match each other's dimension.
    explicit IntegrationStep(const T& initial_time,
                             MatrixX<T>&& initial_state,
                             MatrixX<T>&& initial_state_derivative) {
      ValidateOrThrow(initial_time, initial_state,
                      initial_state_derivative);
      times_.push_back(std::move(initial_time));
      states_.push_back(std::move(initial_state));
      state_derivatives_.push_back(std::move(initial_state_derivative));
    }

    /// Constructs a zero length step by copy from system vectors.
    ///
    /// @param initial_time Initial time t₀ where the step starts.
    /// @param initial_state Initial state vector 𝐱₀ at @p initial_time
    /// @param initial_state_derivative Initial state derivative vector
    ///                                 d𝐱/dt₀ at @p initial_time.
    /// @throw std::runtime_error if given @p initial_state 𝐱₀ and
    ///                           @p initial_state_derivative d𝐱/dt₀
    ///                           do not match each other's dimension.
    /// @throw std::logic_error if any of the preconditions is not met.
    explicit IntegrationStep(const T& initial_time,
                             const VectorBase<T>& initial_state,
                             const VectorBase<T>& initial_state_derivative)
        : IntegrationStep(
              initial_time, detail::CopyToColumnMatrix(initial_state),
              detail::CopyToColumnMatrix(initial_state_derivative)) {}

    /// Extends the step forward in time by copy.
    ///
    /// @param time Time tᵢto extend the step to.
    /// @param state State vector 𝐱ᵢ at @p time tᵢ as a column matrix.
    /// @param state_derivative State derivative vector d𝐱/dtᵢ at @p time tᵢ
    ///                         as a column matrix.
    /// @throw std::runtime_error if:
    ///   - given @p time tᵢ is not greater than the previous time tᵢ₋₁. in
    ///     the step.
    ///   - given @p state 𝐱ᵢ dimension does not match the dimension of the
    ///     previous state 𝐱ᵢ₋₁.
    ///   - given @p state 𝐱ᵢand @p state_derivative d𝐱/dtᵢ do not match each
    ///     other's dimension.
    void Extend(const T& time, const MatrixX<T>& state,
                const MatrixX<T>& state_derivative) {
      ValidateOrThrow(time, state, state_derivative);
      times_.push_back(time);
      states_.push_back(state);
      state_derivatives_.push_back(state_derivative);
    }

    /// Extends the step forward in time by move.
    ///
    /// @param time Time tᵢto extend the step to.
    /// @param state State vector 𝐱ᵢ at @p time tᵢ as a column matrix.
    /// @param state_derivative State derivative vector d𝐱/dtᵢ at @p time tᵢ
    ///                         as a column matrix.
    /// @throw std::runtime_error if:
    ///   - given @p time tᵢ is not greater than the previous time tᵢ₋₁. in
    ///     the step.
    ///   - given @p state 𝐱ᵢ dimension does not match the dimension of the
    ///     previous state 𝐱ᵢ₋₁.
    ///   - given @p state 𝐱ᵢand @p state_derivative d𝐱/dtᵢ do not match each
    ///     other's dimension.
    void Extend(const T& time, MatrixX<T>&& state,
                MatrixX<T>&& state_derivative) {
      ValidateOrThrow(time, state, state_derivative);
      times_.push_back(time);
      states_.push_back(std::move(state));
      state_derivatives_.push_back(std::move(state_derivative));
    }

    /// Extends the step forward in time by copy from system vectors.
    ///
    /// @param time Time tᵢto extend the step to.
    /// @param state State vector 𝐱ᵢ at @p time tᵢ.
    /// @param state_derivative State derivative vector d𝐱/dtᵢ at @p time tᵢ.
    /// @throw std::runtime_error if:
    ///   - given @p time tᵢ is not greater than the previous time tᵢ₋₁. in
    ///     the step.
    ///   - given @p state 𝐱ᵢ dimension does not match the dimension of the
    ///     previous state 𝐱ᵢ₋₁.
    ///   - given @p state 𝐱ᵢand @p state_derivative d𝐱/dtᵢ do not match each
    ///     other's dimension.
    void Extend(const T& time, const VectorBase<T>& state,
                const VectorBase<T>& state_derivative) {
      this->Extend(time, detail::CopyToColumnMatrix(state),
                   detail::CopyToColumnMatrix(state_derivative));
    }

    /// Returns step start time t₀, which may coincide with its
    /// end time tᵢ if the step has zero length.
    const T& get_start_time() const { return times_.front(); }

    /// Returns step end time tᵢ, which may coincide with its
    /// start time t₀ if the step has zero length.
    const T& get_end_time() const { return times_.back(); }

    /// Returns the step state 𝐱 dimensions.
    int get_dimensions() const {
      return states_.back().rows();
    }

    /// Returns step times {t₀, ..., tᵢ₋₁, tᵢ}.
    const std::vector<T>& get_times() const { return times_; }

    /// Returns step states {𝐱₀, ..., 𝐱ᵢ₋₁, 𝐱ᵢ} as column matrices.
    const std::vector<MatrixX<T>>& get_states() const { return states_; }

    /// Gets step state derivatives {d𝐱/dt₀, ..., d𝐱/dtᵢ₋₁, d𝐱/dtᵢ}
    /// as column matrices.
    const std::vector<MatrixX<T>>& get_state_derivatives() const {
      return state_derivatives_;
    }

   private:
    // Validates step update triplet for consistency among them
    // and with step content.
    //
    // @param time Time tᵢto extend the step to.
    // @param state State vector 𝐱ᵢ at @p time tᵢ as a column matrix.
    // @param state_derivative State derivative vector d𝐱/dtᵢ at @p time tᵢ
    //                         as a column matrix.
    /// @throw std::runtime_error if
    ///   - given @p time tᵢ is not greater than the previous time tᵢ₋₁. in
    ///     the step.
    ///   - given @p state 𝐱ᵢ is not a column matrix.
    ///   - given @p state 𝐱ᵢ dimension does not match the dimension of the
    ///     previous state 𝐱ᵢ₋₁, if any.
    ///   - given @p state_derivative d𝐱/dtᵢ is not a column matrix.
    ///   - given @p state 𝐱ᵢand @p state_derivative d𝐱/dtᵢ do not match each
    ///     other's dimension.
    void ValidateOrThrow(const T& time, const MatrixX<T>& state,
                         const MatrixX<T>& state_derivative) {
      if (state.cols() != 1) {
        throw std::runtime_error("Provided state for step is "
                               "not a column matrix.");
      }
      if (state_derivative.cols() != 1) {
        throw std::runtime_error("Provided state derivative for "
                               " step is not a column matrix.");
      }
      if (!times_.empty()) {
        if (time < times_.front()) {
          throw std::runtime_error("Step cannot be extended"
                                   " backwards in time.");
        }
        if (time <= times_.back()) {
          throw std::runtime_error("Step already extends up"
                                   " to the given time.");
        }
      }
      if (!states_.empty() && states_.back().rows() != state.rows()) {
          throw std::runtime_error("Provided state dimensions do not "
                                   "match that of the states in the step.");
      }
      if (state.rows() != state_derivative.rows()) {
        throw std::runtime_error("Provided state and state derivative "
                                 "dimensions do not match.");
      }
    }
    // Step times, ordered in increasing order.
    std::vector<T> times_{};
    // Step states, ordered as to match its
    // corresponding time in `times_`.
    std::vector<MatrixX<T>> states_{};
    // Step state derivatives, ordered as to match
    // its corresponding time in `times_`.
    std::vector<MatrixX<T>> state_derivatives_{};
  };

  VectorX<T> Evaluate(const T& t) const override {
    if (is_empty()) {
      throw std::logic_error("Empty continuous extension"
                             " cannot be evaluated.");
    }
    if (t < get_start_time() || t > get_end_time()) {
      throw std::runtime_error("Continuous extension is not "
                               "defined for given time.");
    }
    return scalar_converter_.FromDoubleMatrix(
        continuous_trajectory_.value(
            scalar_converter_.ToDouble(t)).col(0));
  }

  T Evaluate(const T& t, const int dimension) const override {
    if (is_empty()) {
      throw std::logic_error("Empty continuous extension"
                             " cannot be evaluated.");
    }
    if (dimension < 0 || get_dimensions() <= dimension) {
      throw std::runtime_error("Invalid dimension evaluated for "
                               "continuous extension.");
    }
    if (t < get_start_time() || t > get_end_time()) {
      throw std::runtime_error("Continuous extension is not "
                               "defined for given time.");
    }
    return scalar_converter_.FromDouble(
        continuous_trajectory_.scalarValue(
            scalar_converter_.ToDouble(t), dimension, 0));
  }

  int get_dimensions() const override {
    if (is_empty()) {
      throw std::logic_error("Dimension is not defined for an"
                             " empty continuous extension.");
    }
    return continuous_trajectory_.rows();
  }

  bool is_empty() const override {
    return continuous_trajectory_.empty();
  }

  const T& get_end_time() const override {
    if (is_empty()) {
      throw std::logic_error("End time is not defined for an"
                             " empty continuous extension.");
    }
    return end_time_;
  }

  const T& get_start_time() const override {
    if (is_empty()) {
      throw std::logic_error("Start time is not defined for an"
                             " empty continuous extension.");
    }
    return start_time_;
  }

  /// Update extension with the given @p step by copy.
  /// @param step Integration step to update this extension with.
  /// @throw std::runtime_error if:
  ///   - given @p step has zero length.
  ///   - given @p step does not ensure C1 continuity at the end of
  ///     this continuous extension.
  ///   - given @p step dimension does not match this continuous
  ///     extension dimension.
  void Update(const IntegrationStep& step) {
    ValidateOrThrow(step);
    raw_steps_.push_back(step);
  }

  /// Update extension with the given @p step by move.
  /// @param step Integration step to update this extension with.
  /// @throw std::runtime_error if:
  ///   - given @p step has zero length.
  ///   - given @p step does not ensure C1 continuity at the end of
  ///     this continuous extension.
  ///   - given @p step dimension does not match this continuous
  ///     extension dimension.
  void Update(IntegrationStep&& step) {
    ValidateOrThrow(step);
    raw_steps_.push_back(step);
  }

  void Rollback() override {
    if (raw_steps_.empty()) {
      throw std::logic_error("No updates to rollback.");
    }
    raw_steps_.pop_back();
  }

  void Consolidate() override {
    if (raw_steps_.empty()) {
      throw std::logic_error("No updates to consolidate.");
    }
    for (const IntegrationStep& step : raw_steps_) {
      const std::vector<double> step_times =
          scalar_converter_.ToDoubleVector(step.get_times());
      const std::vector<MatrixX<double>> step_states =
          scalar_converter_.ToDoubleMatrixVector(step.get_states());
      const std::vector<MatrixX<double>> step_state_derivatives =
          scalar_converter_.ToDoubleMatrixVector(step.get_state_derivatives());
      continuous_trajectory_.concatenate(
          trajectories::PiecewisePolynomial<double>::Cubic(
              step_times, step_states, step_state_derivatives),
          kAllowedTimeMisalignment);
    }
    // TODO(hidmic): Remove state keeping members when
    // PiecewisePolynomial better supports non-double types.
    start_time_ = continuous_trajectory_.start_time();
    end_time_ = continuous_trajectory_.end_time();

    const IntegrationStep& last_step = raw_steps_.back();
    end_state_ = last_step.get_states().back();
    end_state_derivative_ = last_step.get_state_derivatives().back();

    raw_steps_.clear();
  }

 private:
  // Validates that the provided @p step can be consolidated
  // into this continuous extension.
  // @param step Integration step to be taken.
  // @throw std::runtime_error if:
  //   - given @p step has zero length.
  //   - given @p step does not ensure C1 continuity at the end
  //     of this continuous extension.
  //   - given @p step dimension does not match this continuous
  //     extension dimension.
  void ValidateOrThrow(const IntegrationStep& step) {
    if (step.get_start_time() == step.get_end_time()) {
      throw std::runtime_error("Provided step has zero length "
                               "i.e. start time and end time "
                               "are equal.");
    }
    if (!raw_steps_.empty()) {
      const IntegrationStep& last_raw_step = raw_steps_.back();
      const T& last_raw_step_end_time = last_raw_step.get_end_time();
      const MatrixX<T>& last_raw_step_end_state =
          last_raw_step.get_states().back();
      const MatrixX<T>& last_raw_step_end_state_derivative =
          last_raw_step.get_state_derivatives().back();
      EnsureConsistencyOrThrow(step, last_raw_step_end_time,
                               last_raw_step_end_state,
                               last_raw_step_end_state_derivative);
    } else if (!continuous_trajectory_.empty()) {
      EnsureConsistencyOrThrow(step, end_time_, end_state_,
                               end_state_derivative_);
    }
  }

  // Ensures the provided @p step is consistent with the given
  // continuous extension quantities evaluated at its end.
  // @param step Integration step to be taken.
  // @param end_time Continuous extension end time.
  // @param end_state Continuous extension end state.
  // @param end_state Continuous extension end state derivative.
  // @throw std::runtime_error if:
  //   - given @p step does not ensure C1 continuity at the end
  //     of this continuous extension.
  //   - given @p step dimension does not match this continuous
  //     extension dimension.
  void EnsureConsistencyOrThrow(const IntegrationStep& step,
                                const T& end_time, const MatrixX<T>& end_state,
                                const MatrixX<T>& end_state_derivative) {
    if (end_state.rows() != step.get_dimensions()) {
      throw std::runtime_error("Provided step dimension and continuous"
                               " extension (inferred) dimension do not match.");
    }
    const double time_misalignment = std::abs(
        scalar_converter_.ToDouble(end_time) -
        scalar_converter_.ToDouble(step.get_start_time()));
    if (time_misalignment > kAllowedTimeMisalignment) {
      throw std::runtime_error("Provided step start time and"
                               " continuous extension end time"
                               " differ.");
    }
    if (!end_state.isApprox(step.get_states().front())) {
      throw std::runtime_error("Provided step start state and"
                               " continuous extension end state"
                               " differ. Cannot ensure C0 continuity.");
    }
    if (!end_state_derivative.isApprox(step.get_state_derivatives().front())) {
      throw std::runtime_error("Provided step start state derivative and"
                               " continuous extension end state derivative"
                               " differ. Cannot ensure C1 continuity.");
    }
  }

  // TODO(hidmic): Remove state keeping members when
  // PiecewisePolynomial better supports non-double types.

  // The smallest time at which the extension is defined.
  T start_time_{};
  // The largest time at which the extension is defined.
  T end_time_{};
  // The value of the extension at the `end_time`.
  MatrixX<T> end_state_{};
  // The value of the extension's first time derivative at the `end_time`.
  MatrixX<T> end_state_derivative_{};

  // The integration steps taken but not consolidated yet (via Consolidate()).
  std::vector<IntegrationStep> raw_steps_{};
  // The underlying PiecewisePolynomial continuous trajectory.
  trajectories::PiecewisePolynomial<double> continuous_trajectory_{};
  // Maximum time misalignment between steps that it is safe to align
  // to ensure continuity.
  static constexpr double kAllowedTimeMisalignment{
    5. * std::numeric_limits<T>::epsilon()};
  // Conversion mechanisms to deal with non floating point types.
  const detail::ScalarConverter<T> scalar_converter_{};
};

}  // namespace systems
}  // namespace drake
