#include <iostream>
#include <string> 
#include <memory>
#include <limits>
#include <stdexcept>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"

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

template<typename T>
std::unique_ptr<drake::systems::RigidBodyPlant<T>>
CreateSystem(const std::string &sdfFilePath) {
  auto rigid_body_tree = std::make_unique<RigidBodyTree<T>>();
  drake::parsers::sdf::AddModelInstancesFromSdfFile(sdfFilePath,
      multibody::joints::kFixed,
      nullptr /* weld to frame */,
      rigid_body_tree.get());
  return std::make_unique<drake::systems::RigidBodyPlant<T>>(
      std::move(rigid_body_tree));
}

void LoadSDFSample(const std::string &sdfFilePath) {
  systems::DiagramBuilder<double> builder;
  const systems::RigidBodyPlant<double>* rigidBodyPlant =
      builder.AddSystem<systems::RigidBodyPlant<double>>(
          CreateSystem<double>(sdfFilePath));
  
  const RigidBodyTree<double> &tree = rigidBodyPlant->get_rigid_body_tree();
  
  drake::lcm::DrakeLcm lcm;

  const auto viz_publisher =
    builder.template AddSystem<systems::DrakeVisualizer>(
      rigidBodyPlant->get_rigid_body_tree(), &lcm);

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  std::cout << "Before simulator.Initialize()" << std::endl;
  simulator.Initialize();
  std::cout << "After simulator.Initialize()" << std::endl;
  simulator.StepTo(0.1);
  std::cout << "After simulator.Initialize()" << std::endl;
}

}
}

int main (int argc, char **argv) {
  std::cout << "[sdf_sample]: Program started" << std::endl;
  std::string sdfFilePath(drake::GetDrakePath() + "/sdf_sample/models/box.sdf");
  std::cout << "[sdf_sample]: SDF file path: " << sdfFilePath << std::endl;
  // Load the sdf file into memory
  drake::sdf_sample::LoadSDFSample(sdfFilePath);

  return 0;
}