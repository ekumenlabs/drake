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

void LoadSDFSample(const std::string &filePath) {
  systems::DiagramBuilder<double> builder;
  const systems::RigidBodyPlant<double>* rigidBodyPlant =
      builder.AddSystem<systems::RigidBodyPlant<double>>(
          CreateSDFSystem<double>(filePath));
  
  const RigidBodyTree<double> &tree = rigidBodyPlant->get_rigid_body_tree();
  
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

template<typename T>
std::unique_ptr<drake::systems::RigidBodyPlant<T>>
CreateURDFSystem(const std::string &filePath) {
  auto rigid_body_tree = std::make_unique<RigidBodyTree<T>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      filePath,
      multibody::joints::kFixed,
      rigid_body_tree.get());
  return std::make_unique<drake::systems::RigidBodyPlant<T>>(
      std::move(rigid_body_tree));
}

void LoadURDFSample(const std::string &filePath) {
  systems::DiagramBuilder<double> builder;
  const systems::RigidBodyPlant<double>* rigidBodyPlant =
      builder.AddSystem<systems::RigidBodyPlant<double>>(
          CreateURDFSystem<double>(filePath));
  
  const RigidBodyTree<double> &tree = rigidBodyPlant->get_rigid_body_tree();
  
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

bool endsWith(const std::string &suffix, const std::string &str) {
  return std::mismatch(suffix.rbegin(), suffix.rend(), str.rbegin()).first != suffix.rend();
}

bool isSDFFile(const std::string &filePath) {
  std::string suffix = ".sdf";
  return endsWith(suffix, filePath);
}


bool isURDFFile(const std::string &filePath) {
  std::string suffix = ".urdf";
  return endsWith(suffix, filePath);
}


int main (int argc, char **argv) {
  std::cout << "[sdf_sample]: Program started" << std::endl;

  std::string filePath(drake::GetDrakePath());

  if (argc == 1) {
    filePath += "/sdf_sample/models/box_with_mesh.sdf";
  }
  else {
    filePath = std::string(argv[1]);
  }
  std::cout << "[sdf_sample]: SDF file path: " << filePath << std::endl;

  // Load the sdf file into memory
  if (isSDFFile(filePath))
    drake::sdf_sample::LoadSDFSample(filePath);
  else if (isURDFFile(filePath))
    drake::sdf_sample::LoadURDFSample(filePath);
  else
  {
    std::cerr << "[sdf_sample]: File path provided [" << filePath << "] is not supported." << std::endl;
    return -1;
  }

  return 0;
}