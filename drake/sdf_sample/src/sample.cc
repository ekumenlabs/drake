#include <iostream>
#include <string>
#include <memory>
#include <limits>
#include <stdexcept>
#include <algorithm>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/common/eigen_types.h"

#include "drake/lcm/drake_lcm.h"

#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

#include "drake/systems/controllers/pid_controlled_system.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/multiplexer.h"

#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"

#include "drake/common/drake_path.h"


namespace drake {
namespace sdf_sample {

/**
 * \brief It loads a SDF file into the parser and gives you a RigidBodyPlant from it.
 * \param[in] filePath The path to the SDF file.
 * \return A pointer to a drake::systems::RigidBodyPlant<T> that handles the SDF structure.
 */
template<typename T>
std::unique_ptr<drake::systems::RigidBodyPlant<T>>
CreateSDFSystem(const std::string &filePath) {
  auto rigid_body_tree = std::make_unique<RigidBodyTree<T>>();
  drake::parsers::sdf::AddModelInstancesFromSdfFile(filePath,
      multibody::joints::kQuaternion,
      nullptr, //weld to frame
      rigid_body_tree.get());
  return std::make_unique<drake::systems::RigidBodyPlant<T>>(
      std::move(rigid_body_tree));
}

/**
 * \brief It does the minimum steps to load a static SDF file description into Drake's visualizer
 * \details You have to do the following so as to get a static SDF:
 *   1.- Create the RigidBodyPlant from the SDF file
 *   2.- Create a DiagramBuilder and a DrakeLcm object
 *   3.- Add a system with the RigidBodyTree (from the RigidBodyPlant) and the DrakeLcm
 *   4.- Connect the output port from the RigidBodyPlant to the input port of the LCMDrakeLcm
 *   5.- Export the output port from the RigidBodyPlant
 *   6.- Build the diagram and create a bare default Context for the simulator.
 *   7.- Create the simulator and then configure it
 * \param[in] filePath It is the path to the SDF file.
 */
void LoadSDFSample(const std::string &filePath) {
  systems::DiagramBuilder<double> builder;
  const systems::RigidBodyPlant<double>* rigidBodyPlant =
      builder.AddSystem<systems::RigidBodyPlant<double>>(
          CreateSDFSystem<double>(filePath));

  drake::lcm::DrakeLcm lcm;

  const auto viz_publisher =
    builder.template AddSystem<systems::DrakeVisualizer>(
      rigidBodyPlant->get_rigid_body_tree(), &lcm);
  builder.Connect(rigidBodyPlant->get_output_port(0),
                  viz_publisher->get_input_port(0));
  builder.ExportOutput(rigidBodyPlant->get_output_port(0));

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> context = diagram->AllocateContext();
  diagram->SetDefaultState(*context, context->get_mutable_state());
  systems::Simulator<double> simulator(*diagram, std::move(context));

  simulator.Initialize();
  simulator.StepTo(0.1);
}

}
}

int main (int argc, char **argv) {
  // Get the drake base path
  std::string filePath(drake::GetDrakePath());
  // Load a sample file or load a file path passed by args
  if (argc == 1) {
    filePath += "/sdf_sample/models/darpa_box.sdf";
  }
  else {
    filePath = std::string(argv[1]);
  }
  std::cout << "[sdf_sample]: SDF file path: " << filePath << std::endl;

  // Load the sdf file into memory
  drake::sdf_sample::LoadSDFSample(filePath);

  return 0;
}