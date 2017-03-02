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
  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  // Obtain the structure we need to write into.
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);

  T currentTime = context.get_time();
  T dt = currentTime - lastTime;
  output_vector->SetAtIndex(0, T(T(1/2) * get_acceleration() * dt * dt + context_state.GetAtIndex(1) * dt + context_state.GetAtIndex(0)));
  output_vector->SetAtIndex(1, T(get_acceleration() * dt + context_state.GetAtIndex(1)));
}

template <typename T>
void Particle<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Obtain the structure we need to write into.
  systems::VectorBase<T>* const new_derivatives =
      derivatives->get_mutable_vector();
  // Set the derivatives. The first one is the speed
  // and the other one is the acceleration
  new_derivatives->SetAtIndex(0,
    context.get_continuous_state_vector().GetAtIndex(1));
  new_derivatives->SetAtIndex(1, T(get_acceleration()));
}

template <typename T>
void Particle<T>::SetDefaultSgit tate(const systems::Context<T>& context,
                              systems::State<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  Vector2<T> x0;
  x0 << 0.0, 0.0;
  state->get_mutable_continuous_state()->SetFromVector(x0);
}

}
}
#endif