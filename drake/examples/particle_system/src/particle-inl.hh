#ifndef PARTICLE_INL_H
#define PARTICLE_INL_H

#include "drake/examples/particle_system/src/particle.hh"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace particle {

template <typename T>
Particle<T>::Particle() {
  this->DeclareContinuousState(1, 1, 0);
  this->DeclareOutputPort(systems::kVectorValued, 2);
}

template <typename T>
void Particle<T>::DoCalcOutput(const systems::Context<T>& context,
                           systems::SystemOutput<T>* output) const {
  // Obtain the structure we need to write into.
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

  output_vector->get_mutable_value() =
      context.get_continuous_state()->CopyToVector();
}

template <typename T>
void Particle<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Obtain the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const new_derivatives =
      derivatives->get_mutable_vector();
  DRAKE_ASSERT(new_derivatives != nullptr);

  new_derivatives->SetAtIndex(0, state.GetAtIndex(1));
  new_derivatives->SetAtIndex(1, T(get_acceleration()));
}

template <typename T>
void Particle<T>::SetDefaultState(const systems::Context<T>& context,
                              systems::State<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  Vector2<T> x0;
  x0 << 10.0, 0.0;  // initial state values.
  state->get_mutable_continuous_state()->SetFromVector(x0);
}

}
}
#endif