#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/multiplexer.h"



namespace drake {
  namespace particles {

    template <typename T>
    class SimpleBillards : public Diagram<T> {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleBillards)

      SimpleBillards();
      
    private:
      multibody::RigidBodyPlant<T>* plant_;
      systems::DrakeVisualizer* visualizer_;
    };

    template <typename T>
    SimpleBillards<T>::SimpleBillards() {
      static const std::string box_sdf =
	GetDrakePath() + "/examples/particle_system/models/box.sdf";
      static const std::string particle_sdf =
	GetDrakePath() + "/examples/particle_system/models/particle.sdf";
      
      parsers::sdf::AddModelInstancesFromSdfFileToWorld(
	box_sdf, multibody::joints::kFixed, &rigid_body_tree);

      parsers::sdf::AddModelInstancesFromSdfFileToWorld(
	particle_sdf, multibody::joints::kRollPitchYaw, &rigid_body_tree);
      
      systems::DiagramBuilder<T> builder;
      auto plant = builder.template AddSystem< multibody::RigidBodyPlant<T> >(
	rigid_body_tree);
      auto vis = builder.template AddSystem < systems::DrakeVisualizer<T> >(
	rigid_body_tree, lcm)
      builder.Connect(plant, vis);
      builder.BuildInto(this);
    }

    int main (int argc, char* argv[]) {
      gflags::ParseCommandLineFlags(&argc, &argv, true);
      logging::HandleSpdlogGflags();

      // instantiate diagram
      SimpleBillards<double> diagram;
      // instantiate simulator
      systems::Simulator<double> simulator(diagram);
      // configure simulator
      simulator.Initialize();
      simulator.set_target_realtime_rate(FLAGS_realtime_rate);
      // set initial conditions
      auto plant = diagram.get_plant();
      plant.
      systems::Context<double> context = simulator.get_mutable_context();
      systems::VectorBase<double> state = context->get_mutable_continuous_state_vector();
      
      state->SetAtIndex(0, FLAGS_initial_pos);
      state->SetAtIndex(1, FLAGS_initial_vel);
      // run simulation
      simulator.StepTo(FLAGS_simulation_time);
      
      return 0;
    }    
    
  }  // namespace particles
}  // namespace drake

int main (int argc, char **argv) {
  return drake::particles::main(argc, argv);
}
