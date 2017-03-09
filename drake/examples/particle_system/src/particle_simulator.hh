
#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace particle {

  template <typename T>
  class ParticleSimulator {
  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ParticleSimulator)

    ParticleSimulator(const T acceleration_setpoint,
		      double target_realtime_rate);
    ~ParticleSimulator();

    void Start(const T initial_position,
	       const T initial_velocity);
    void StepBy(double simulation_time);
  private:
    std::unique_ptr<RigidBodyTree<T>> rigid_body_tree_{
      std::make_unique<RigidBodyTree<T>>()};
    std::unique_ptr<lcm::DrakeLcmInterface> lcm_{
      std::make_unique<DrakeLcm>()};
    std::unique_ptr<systems::Simulator<T>> simulator_;
    bool initialized_{false};
  };
  
} // namespace particle
} // namespace drake
