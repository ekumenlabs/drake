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
//#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/multiplexer.h"


namespace drake {
namespace sdf_sample {


void LoadSDFSample(const std::string &sdfFilePath) {

  auto rigid_body_tree = std::make_unique<RigidBodyTreed>();
  
  parsers::sdf::AddModelInstancesFromSdfFile(
      sdfFilePath,
      multibody::joints::kQuaternion,
      nullptr /* weld to frame */,
      rigid_body_tree.get());
  
  lcm::DrakeLcm lcm;

  systems::DiagramBuilder<double> builder;

  builder.template AddSystem<systems::DrakeVisualizer>(*rigid_body_tree, &lcm);
  
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  lcm.StartReceiveThread();
  
  systems::Simulator<double> simulator(*diagram);
  
  simulator.Initialize();
  
  simulator.StepTo(std::numeric_limits<double>::infinity());
}

}
}

void PrintHelp() {
  std::cerr << "Error calling sdf_sample." << std::endl;
  std::cerr << "You should call it: ./sdf_sample <sdf_file_path>" << std::endl;
}

int main (int argc, char **argv) {
  std::cout << "[sdf_sample]: Program started" << std::endl;

  std::cout << "[sdf_sample]: Program invoked: ";
  for (int i = 0; i < argc; i++) {
    std::cout << argv[i] << " ";
  }
  std::cout << std::endl;

  // Check arguments
  if (argc != 2) {
    PrintHelp();
    exit(-1);
  }
  // Get the sdf file path
  std::string sdfFilePath(reinterpret_cast< char const* >(argv[1]));
  std::cout << "[sdf_sample]: SDF file path: " << sdfFilePath << std::endl;
  // Load the sdf file into memory
  drake::sdf_sample::LoadSDFSample(sdfFilePath);

  return 0;
}