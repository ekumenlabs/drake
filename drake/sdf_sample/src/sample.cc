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


void LoadSDFSample(const std::string &sdfFilePath) {

  auto rigid_body_tree = std::make_unique<RigidBodyTreed>();
  
  parsers::sdf::AddModelInstancesFromSdfFile(
      sdfFilePath,
      multibody::joints::kQuaternion,
      nullptr /* weld to frame */,
      rigid_body_tree.get());
  
  lcm::DrakeLcm lcm;

  systems::DiagramBuilder<double> builder;

  auto plant = std::make_unique<systems::RigidBodyPlant<double>>(std::move(rigid_body_tree));
  /*const Vector3<double> Kp(100,   0,   0);  // Units: Nm / radians
  const Vector3<double> Ki(0,     0,   0);  // Units: Nm / radians
  const Vector3<double> Kd(100,  250, 250);  // Units: Nm / (radians / sec).

  MatrixX<double> feedback_selector_matrix;
  feedback_selector_matrix.setZero(plant->get_input_size() * 2,
                                  plant->get_output_size());*/

  // DRAKE_DEMAND(feedback_selector_matrix.rows() == 6);
  // const int kFeedbackIndexSteeringAnglePosition = 0;
  // const int kFeedbackIndexLeftWheelPosition     = 1;
  // const int kFeedbackIndexRightWheelPosition    = 2;
  // const int kFeedbackIndexSteeringAngleSpeed = 3;
  // const int kFeedbackIndexLeftWheelSpeed     = 4;
  // const int kFeedbackIndexRightWheelSpeed    = 5;

  // DRAKE_DEMAND(feedback_selector_matrix.cols() == 27);
  // const int kStateIndexSteeringAnglePosition = 7;
  // const int kStateIndexLeftWheelPosition     = 9;
  // const int kStateIndexRightWheelPosition    = 11;
  // const int kStateIndexSteeringAngleSpeed = 20;
  // const int kStateIndexLeftWheelSpeed     = 22;
  // const int kStateIndexRightWheelSpeed    = 24;
  // feedback_selector_matrix(kFeedbackIndexSteeringAnglePosition,
  //                          kStateIndexSteeringAnglePosition) = 1;
  // feedback_selector_matrix(kFeedbackIndexLeftWheelPosition,
  //                          kStateIndexLeftWheelPosition) = 1;
  // feedback_selector_matrix(kFeedbackIndexRightWheelPosition,
  //                          kStateIndexRightWheelPosition) = 1;
  // feedback_selector_matrix(kFeedbackIndexSteeringAngleSpeed,
  //                          kStateIndexSteeringAngleSpeed) = 1;
  // feedback_selector_matrix(kFeedbackIndexLeftWheelSpeed,
  //                          kStateIndexLeftWheelSpeed) = 1;
  // feedback_selector_matrix(kFeedbackIndexRightWheelSpeed,
  //                          kStateIndexRightWheelSpeed) = 1;
  /*auto feedback_selector =
      std::make_unique<systems::MatrixGain<double>>(feedback_selector_matrix);

  auto controller = builder.AddSystem<systems::PidControlledSystem>(
      std::move(plant), std::move(feedback_selector), Kp, Ki, Kd);*/

  // Instantiates a system for visualizing the model.
  /*const RigidBodyTreed& tree_ptr =
      dynamic_cast<const systems::RigidBodyPlant<double>*>(controller->plant())->
          get_rigid_body_tree();*/
  /*const RigidBodyTreed& tree_ptr =
      dynamic_cast<const systems::RigidBodyPlant<double>*>(std::move(plant))->
          get_rigid_body_tree();
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(tree_ptr, &lcm);

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  lcm.StartReceiveThread();
  
  systems::Simulator<double> simulator(*diagram);
  
  simulator.Initialize();
  
  simulator.StepTo(std::numeric_limits<double>::infinity());*/
}

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

void LoadSDFSample2(const std::string &sdfFilePath) {
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
  simulator.Initialize();
  simulator.StepTo(0.1);
}

}
}

void PrintHelp() {
  std::cerr << "Error calling sdf_sample." << std::endl;
  std::cerr << "You should call it: ./sdf_sample <sdf_file_path>" << std::endl;
}

int main (int argc, char **argv) {
  std::cout << "[sdf_sample]: Program started" << std::endl;

  /*std::cout << "[sdf_sample]: Program invoked: ";
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
  std::string sdfFilePath(reinterpret_cast< char const* >(argv[1]));*/
  std::string sdfFilePath(drake::GetDrakePath() + "/sdf_sample/models/box.sdf");
  std::cout << "[sdf_sample]: SDF file path: " << sdfFilePath << std::endl;
  // Load the sdf file into memory
  drake::sdf_sample::LoadSDFSample2(sdfFilePath);

  return 0;
}