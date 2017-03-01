#include <iostream>
#include <memory>

#include "drake/examples/particle_system/src/particle.hh"
#include "drake/examples/particle_system/src/particle-inl.hh"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"

namespace drake {
namespace particle {

int main (int argc, char **argv) {
	std::unique_ptr<systems::System<double>> particle;
	std::unique_ptr<systems::Context<double>> particleContext;
	std::unique_ptr<systems::SystemOutput<double>> particleOutput;
	std::unique_ptr<systems::ContinuousState<double>> particleDerivatives;

	particle.reset(new drake::particle::Particle<double>());
	particleContext = particle->CreateDefaultContext();
	particleOutput = particle->AllocateOutput(*particleContext);
	particleDerivatives = particle->AllocateTimeDerivatives();

	const double t_final = 1.0;
	const double x0 = 10.0;
	const double v0 = 0.0;
	// Prepare to integrate.
	drake::systems::Simulator<double> simulator(*particle, std::move(particleContext));
	simulator.reset_integrator<systems::RungeKutta3Integrator<double>>(
		*particle,
		simulator.get_mutable_context());
	simulator.get_mutable_integrator()->set_fixed_step_mode(true);
	simulator.get_mutable_integrator()->set_minimum_step_size(1e-3);
	simulator.get_mutable_integrator()->set_maximum_step_size(1e-3);
	simulator.Initialize();

	// Set the initial state for the particle
	systems::VectorBase<double>* xc = simulator.get_mutable_context()->
		get_mutable_continuous_state_vector();
	xc->SetAtIndex(0, x0);
	xc->SetAtIndex(1, v0);
  // Integrate.
  simulator.StepTo(t_final);

  std::cout << "Last position: " << x0 << std::endl;
  std::cout << "Last speed: " << v0 << std::endl;

  return 0;
}

}
}

int main (int argc, char **argv) {
	return drake::particle::main(argc, argv);
}