
#include "drake/examples/particle_system/src/particle_simulator.hh"

namespace drake {
namespace particle {


  template <typename T>
  ParticleSimulator<T>::ParticleSimulator( 
    double target_realtime_rate)
    : lcm_(std::make_unique<lcm::DrakeLcm>())
  {
    simulator_
  }
  
  template <typename T>
  ParticleSimulator<T>::~ParticleSimulator()
  {
    lcm_.reset();
  }
    
  template <typename T>
  void ParticleSimulator<T>::Initialize(const T acceleration, double target_realtime_rate)
  {
    DRAKE_DEMAND(!initialized_);
    
    systems::DiagramBuilder<T> builder;
    auto accel_source = builder_->template AddSystem<systems::ConstantValueSource<T>>(
      std::make_unique(Value<T>(acceleration))
    );
    auto particle = builder_->template AddSystem<Particle<T>>();
    // Acceleration constant -> Particle input
    builder->Connect(*accel_source, *particle);

    auto visualizer = builder_->template AddSystem<systems::DrakeVisualizer>(
	*rigid_body_tree_, *lcm_
    );
    // Particle state -> Visualizer inputs
    builder_->Connect(*particle, *visualizer);

    systems::Diagram diagram = builder_->Build();
    simulator_ = std::make_unique<systems::Simulator<T>>(*diagram_);
  
    lcm_->StartReceiveThread();
  
    simulator_->set_target_realtime_rate(target_realtime_rate_);
    simulator_->reset_integrator<systems::RungeKutta3Integrator<double>>(
      simulator_->get_system(), simulator_->get_mutable_context()
    );
    simulator->get_mutable_integrator()->set_fixed_step_mode(true);
    simulator->get_mutable_integrator()->set_minimum_step_size(10e-3);
    simulator->get_mutable_integrator()->set_maximum_step_size(10e-3);
    simulator_->Initialize();
  }

  template <typename T>
  void ParticleSimulator<T>::Reset(T initial_position,
				   T initial_velocity)
  {
    DRAKE_DEMAND(initialized_);    

    // Initialize the state of the particle.
    auto context = diagram_->AllocateContext();
    auto state = context->get_mutable_continuous_state_vector();
    state.SetAtIndex(0, initial_position);
    state.SetAtIndex(1, initial_velocity);
    simulator_.reset_context(context);
  }

  template <typename T>
  void ParticleSimulator<T>::StepBy(const T& time_step)
  {
    auto context = simulator_->get_context();
    SPDLOG_TRACE(drake::log(), "Simulation started at time {}", context.get_time());
    auto state = context.get_continuous_state_vector();
    SPDLOG_TRACE(drake::log(), "Initial state is position = {} m, velocity = {}",
		 state.GetAtIndex(0), state.GetAtIndex(1));
    simulator_->StepTo(context.get_time() + time_step);
    SPDLOG_TRACE(drake::log(), "Simulation finished at time {}", context.get_time());
    state = context.get_continuous_state_vector();  
    SPDLOG_TRACE(drake::log(), "Final state is position = {} m, velocity = {}",
		 state.GetAtIndex(0), state.GetAtIndex(1));
  }

}
} // namespace drake
