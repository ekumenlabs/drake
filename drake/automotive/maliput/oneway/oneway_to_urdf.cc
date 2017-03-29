/**
 @file Instantiates a dragway with a user-specified number of lanes and outputs
 a URDF model of it.
 **/
#include <gflags/gflags.h>

#include "drake/automotive/maliput/oneway/road_geometry.h"
#include "drake/automotive/maliput/utility/generate_urdf.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "spruce.hh"

DEFINE_double(length, 10, "The length of the dragway in meters.");
// By default, each lane is 3.7m (12 feet) wide, which is the standard used by
// the U.S. interstate highway system.
DEFINE_double(lane_width, 3.7, "The width of each lane in meters.");
DEFINE_string(dirpath, ".",
    "The path to where the URDF and OBJ files should be saved. If this path "
    " does not exist, it is created.");
DEFINE_string(file_name_root, "oneway",
    "The root name of the files to create. For example, if the value of this "
    "parameter is \"foo\", the following files will be created: \"foo.urdf\", "
    "\"foo.obj\", and \"foo.mtl\". These files will be placed in the path "
    "specified by parameter 'dirpath'.");

namespace drake {
namespace maliput {
namespace oneway {
namespace {

int exec(int argc, char* argv[]) {
  std::cout << "Stating" << std::endl;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::cout << "Parsing" << std::endl;
  logging::HandleSpdlogGflags();
  std::cout << "Logging" << std::endl;

  RoadGeometry road_geometry(
      {"Oneway road"},
      FLAGS_length,
      FLAGS_lane_width);
  std::cout << "RoadGeometry created." << std::endl;
  utility::ObjFeatures features;

  // Creates the destination directory if it does not already exist.
  spruce::path directory;
  directory.setStr(FLAGS_dirpath);
  if (!directory.exists()) {
    spruce::dir::mkdirAll(directory);
  }
  std::cout << "Directory: " << directory.getStr() << std::endl;
  DRAKE_DEMAND(directory.exists());

  // The following is necessary for users to know where to find the resulting
  // files when this program is executed in a sandbox. This occurs, for example
  // when using `bazel run //drake/automotive/maliput/dragway:dragway_to_urdf`.
  spruce::path my_path;
  my_path.setAsCurrent();

  drake::log()->info("Creating Oneway URDF in {}.", my_path.getStr());
  std::cout << "Creating Oneway URDF in " <<  my_path.getStr() << std::endl;
  utility::GenerateUrdfFile(&road_geometry, directory.getStr(),
      FLAGS_file_name_root, features);
  return 0;
}

}  // namespace
}  // namespace oneway
}  // namespace maliput
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::maliput::oneway::exec(argc, argv);
}
