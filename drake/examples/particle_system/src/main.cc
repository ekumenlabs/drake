#include <iostream>
#include <memory>
#include <stdlib.h>

#include "drake/examples/particle_system/src/particle.hh"
#include "drake/examples/particle_system/src/particle-inl.hh"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"

namespace drake {
namespace particle {

void PrintHelp() {
	std::cerr << "Run this program the following way:" << std::endl;
	std::cerr << "bazel run drake/examples/particle_system:sample [t_final] [x_init] [v_init] [accel]" << std::endl;
	std::cerr << "All the values are doubles." << std::endl;
}

void PrintValues(const double t_final,
	const double xi,
	const double vi,
	const double a) {
	std::cout << "t_final\txi\tvi\ta" << std::endl;
	std::cout << t_final << xi << vi << a << std::endl;
}

int main (int argc, char **argv) {
	if (argc < 5) {
		PrintHelp();
	}
	const double t_final = atof(argv[1]);
	const double xi = atof(argv[2]);
	const double vi = atof(argv[3]);
	const double a = atof(argv[4]);
	PrintValues(t_final, xi, vi, a);

	std::unique_ptr<drake::particle::Particle<double>> particle;
	particle.reset(new drake::particle::Particle<double>());
    std::unique_ptr<systems::Context<double>> particleContext;
	particleContext = particle->CreateDefaultContext();

	particle->SetAcceleration(a);
	// Prepare to integrate.
	drake::systems::Simulator<double> simulator(
		*particle,
		std::move(particleContext));
	simulator.reset_integrator<systems::RungeKutta3Integrator<double>>(
		*particle,
		simulator.get_mutable_context());
	simulator.get_mutable_integrator()->set_fixed_step_mode(true);
	simulator.get_mutable_integrator()->set_minimum_step_size(10e-3);
	simulator.get_mutable_integrator()->set_maximum_step_size(10e-3);
	simulator.Initialize();

	// Set the initial state for the particle
	systems::VectorBase<double>* xc = simulator.get_mutable_context()->
		get_mutable_continuous_state_vector();
	xc->SetAtIndex(0, xi);
	xc->SetAtIndex(1, vi);
  // Integrate.
  simulator.StepTo(t_final);

  const drake::systems::Context<double> &context = simulator.get_context();
  const drake::systems::VectorBase<double> &csVector = context.get_continuous_state_vector();
  std::cout << "Last position: " << csVector.GetAtIndex(0) << std::endl;
  std::cout << "Last speed: " << csVector.GetAtIndex(1) << std::endl;

  return 0;
}

}
}

int main (int argc, char **argv) {
	return drake::particle::main(argc, argv);
}