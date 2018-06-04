#pragma once

#include <memory>
#include <utility>

#include "drake/systems/analysis/continuous_extension.h"
#include "drake/systems/analysis/scalar_continuous_extension.h"

namespace drake {
namespace systems {

/// A ScalarContinuousExtension class implementation that wraps a
/// ContinuousExtension class instance and behaves as a view to one of
/// its dimensions.
///
/// @tparam T A valid Eigen scalar.
template <typename T>
class ScalarViewContinuousExtension : public ScalarContinuousExtension<T> {
 public:
  /// Constructs a view of another ContinuousExtension instance.
  /// @param base_extension Base continuous extension to operate with.
  /// @param dimension The dimension to view.
  /// @throw std::logic_error if specified @p dimension is not a
  ///                         valid dimension of @p base_extension.
  explicit ScalarViewContinuousExtension(
      std::unique_ptr<ContinuousExtension<T>> base_extension, int dimension)
      : base_extension_(std::move(base_extension)),
        dimension_(dimension) {
    if (dimension < 0 || dimension >= base_extension_->get_dimensions()) {
      throw std::logic_error("Dimension out of range for the "
                             "given base continuous extension.");
    }
  }

  T Evaluate(const T& t) const override {
    return base_extension_->Evaluate(t, dimension_);
  }

  bool is_empty() const override {
    return base_extension_->is_empty();
  }

  const T& get_start_time() const override {
    return base_extension_->get_start_time();
  }

  const T& get_end_time() const override {
    return base_extension_->get_end_time();
  }

  /// Returns the base continuous extension upon which the
  /// view operates.
  const ContinuousExtension<T>* get_base_extension() const {
    return base_extension_.get();
  }

 private:
  // The base (vector) continuous extension wrapped.
  const std::unique_ptr<ContinuousExtension<T>> base_extension_;
  // The dimension of interest for the view.
  const int dimension_;
};

}  // namespace systems
}  // namespace drake
