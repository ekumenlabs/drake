#include "drake/systems/analysis/scalar_view_continuous_extension.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/hermitian_continuous_extension.h"

namespace drake {
namespace systems {
namespace analysis {
namespace {

template <typename T>
class ScalarViewContinuousExtensionTest : public ::testing::Test {
 protected:
  std::unique_ptr<ContinuousExtension<T>> CreateDummyContinuousExtension() {
    auto continuous_extension =
        std::make_unique<HermitianContinuousExtension<T>>();
    typename HermitianContinuousExtension<T>::IntegrationStep step(
        this->kInitialTime, this->kInitialState, this->kInitialStateDerivative);
    step.Extend(this->kFinalTime, this->kFinalState,
                this->kFinalStateDerivative);
    continuous_extension->Update(std::move(step));
    continuous_extension->Consolidate();
    return std::move(continuous_extension);
  }

  const T kInvalidTime{-1.0};
  const T kInitialTime{0.0};
  const T kMidTime{0.5};
  const T kFinalTime{1.0};
  const T kTimeStep{0.1};
  const MatrixX<T> kInitialState{
    (MatrixX<T>(3, 1) << 0., 0., 0.).finished()};
  const MatrixX<T> kMidState{
    (MatrixX<T>(3, 1) << 0.5, 5., 50.).finished()};
  const MatrixX<T> kFinalState{
    (MatrixX<T>(3, 1) << 1., 10., 100.).finished()};
  const MatrixX<T> kFinalStateWithFewerDimensions{
    (MatrixX<T>(2, 1) << 1., 10.).finished()};
  const MatrixX<T> kFinalStateWithMoreDimensions{
    (MatrixX<T>(4, 1) << 1., 10., 100., 1000.).finished()};
  const MatrixX<T> kFinalStateNotAVector{
    (MatrixX<T>(2, 2) << 1., 10., 100., 1000.).finished()};
  const MatrixX<T> kInitialStateDerivative{
    (MatrixX<T>(3, 1) << 0., 1., 0.).finished()};
  const MatrixX<T> kMidStateDerivative{
    (MatrixX<T>(3, 1) << 0.5, 0.5, 0.5).finished()};
  const MatrixX<T> kFinalStateDerivative{
    (MatrixX<T>(3, 1) << 1., 0., 1.).finished()};
  const MatrixX<T> kFinalStateDerivativeWithFewerDimensions{
    (MatrixX<T>(2, 1) << 1., 0.).finished()};
  const MatrixX<T> kFinalStateDerivativeWithMoreDimensions{
    (MatrixX<T>(4, 1) << 1., 0., 1., 0.).finished()};
  const MatrixX<T> kFinalStateDerivativeNotAVector{
    (MatrixX<T>(2, 2) << 0., 1., 0., 1.).finished()};
  const int kValidDimension{0};
  const int kInvalidDimension{10};
};

typedef ::testing::Types<double, AutoDiffXd> ExtensionTypes;

TYPED_TEST_CASE(ScalarViewContinuousExtensionTest, ExtensionTypes);

// Checks that ScalarViewContinuousExtension properly wraps a
// ContinuousExtension instance.
TYPED_TEST(ScalarViewContinuousExtensionTest, ExtensionConsistency) {
  // Verifies that views to invalid dimensions result in an error.
  EXPECT_THROW(
      ScalarViewContinuousExtension<TypeParam> continuous_extension(
          this->CreateDummyContinuousExtension(), this->kInvalidDimension),
      std::logic_error);

  EXPECT_THROW(
      ScalarViewContinuousExtension<TypeParam> continuous_extension(
          this->CreateDummyContinuousExtension(), -this->kInvalidDimension),
      std::logic_error);

  // Instantiates scalar continuous extension properly.
  ScalarViewContinuousExtension<TypeParam> continuous_extension(
      this->CreateDummyContinuousExtension(), this->kValidDimension);

  // Retrieves dummy base continuous extension.
  const ContinuousExtension<TypeParam>* base_extension =
      continuous_extension.get_base_extension();

  // Checks basic getters for consistency.
  EXPECT_EQ(continuous_extension.is_empty(),
            base_extension->is_empty());
  EXPECT_EQ(continuous_extension.get_start_time(),
            base_extension->get_start_time());
  EXPECT_EQ(continuous_extension.get_end_time(),
            base_extension->get_end_time());

  // Compares evaluations for consistency.
  EXPECT_THROW(
      continuous_extension.Evaluate(this->kInvalidTime),
      std::runtime_error);
  EXPECT_EQ(
      continuous_extension.Evaluate(this->kInitialTime),
      base_extension->Evaluate(this->kInitialTime, this->kValidDimension));
  EXPECT_EQ(
      continuous_extension.Evaluate(this->kMidTime),
      base_extension->Evaluate(this->kMidTime, this->kValidDimension));
  EXPECT_EQ(
      continuous_extension.Evaluate(this->kFinalTime),
      base_extension->Evaluate(this->kFinalTime, this->kValidDimension));
}

}  // namespace
}  // namespace analysis
}  // namespace systems
}  // namespace drake
