#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include <gflags/gflags.h>

#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/utility/generate_obj.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"

DEFINE_string(obj_dir, ".", "Directory to contain rendered road surface");
DEFINE_string(obj_file, "",
              "Basename for output Wavefront OBJ and MTL files");
DEFINE_double(max_grid_unit,
              drake::maliput::utility::ObjFeatures().max_grid_unit,
              "Maximum size of a grid unit in the rendered mesh covering the"
              " road surface");
DEFINE_double(min_grid_resolution,
              drake::maliput::utility::ObjFeatures().min_grid_resolution,
              "Minimum number of grid-units in either lateral or longitudinal"
              " direction in the rendered mesh covering the road surface");

namespace drake {
namespace maliput {
namespace  {

std::unique_ptr<const api::RoadGeometry> BuildStairLanes() {
  const double width = 5.;
  const double height = 10.0;
  std::unique_ptr<multilane::Builder> rb(
      new multilane::Builder(api::RBounds(-width, width),
                               api::RBounds(-width, width),
                             api::HBounds(0., height),
                             0.01, /* linear tolerance */
                             0.01 * M_PI /* angular tolerance */));
  /*
  auto four_lines = [&]() {
    // Initialize the road from the origin.
    const multilane::EndpointXy kOriginXy{0., 0., 0.};
    const multilane::EndpointZ kFlatZ{0., 0., 0., 0.};
    const multilane::Endpoint kRoadOrigin{kOriginXy, kFlatZ};
    rb->Connect("lane0", kRoadOrigin, 100.0, kFlatZ);
    rb->Connect("lane1", kRoadOrigin, 100.0, kFlatZ);
    rb->Connect("lane2", kRoadOrigin, 100.0, kFlatZ);
    rb->Connect("lane3", kRoadOrigin, 100.0, kFlatZ);
  };
  */
  /*
  auto four_lines_elevation = [&]() {
    // Initialize the road from the origin.
    const multilane::EndpointXy kOriginXy{0., 0., 0.};
    const multilane::EndpointZ kFlatZ{0., 0., 0., 0.};
    const multilane::EndpointZ kEndZ{10., 0., 0., 0.};
    const multilane::Endpoint kRoadOrigin{kOriginXy, kEndZ};
    rb->Connect("lane0", kRoadOrigin, 100.0, kFlatZ);
    rb->Connect("lane1", kRoadOrigin, 100.0, kFlatZ);
    rb->Connect("lane2", kRoadOrigin, 100.0, kFlatZ);
    rb->Connect("lane3", kRoadOrigin, 100.0, kFlatZ);
  };
  */
  auto four_lines_elevation_superelevation = [&]() {
    // Initialize the road from the origin.
    const multilane::EndpointXy kOriginXy{0., 0., 0.};
    const multilane::EndpointZ kFlatZ{0., 0., M_PI / 4.0, 0.};
    const multilane::EndpointZ kEndZ{10., 0., M_PI / 4.0, 0.};
    const multilane::Endpoint kRoadOrigin{kOriginXy, kEndZ};
    rb->Connect("lane0", kRoadOrigin, 100.0, kFlatZ);
    rb->Connect("lane1", kRoadOrigin, 100.0, kFlatZ);
    rb->Connect("lane2", kRoadOrigin, 100.0, kFlatZ);
    rb->Connect("lane3", kRoadOrigin, 100.0, kFlatZ);
  };
  /*
  auto four_arcs = [&]() {
    // Initialize the road from the origin.
    const multilane::EndpointXy kOriginXy{100., 100., 0.};
    const multilane::EndpointZ kFlatZ{0., 0., 0., 0.};
    const multilane::Endpoint kRoadOrigin{kOriginXy, kFlatZ};
    rb->Connect("lane0", kRoadOrigin, multilane::ArcOffset(100., M_PI / 2.), kFlatZ);
    rb->Connect("lane1", kRoadOrigin, multilane::ArcOffset(100., M_PI / 2.), kFlatZ);
    rb->Connect("lane2", kRoadOrigin, multilane::ArcOffset(100., M_PI / 2.), kFlatZ);
    rb->Connect("lane3", kRoadOrigin, multilane::ArcOffset(100., M_PI / 2.), kFlatZ);
  };
  */
  /*
  auto four_arcs_elevation = [&]() {
    // Initialize the road from the origin.
    const multilane::EndpointXy kOriginXy{100., 100., 0.};
    const multilane::EndpointZ kFlatZ{0., 0., 0., 0.};
    const multilane::EndpointZ kEndZ{10., 0., 0., 0.};
    const multilane::Endpoint kRoadOrigin{kOriginXy, kFlatZ};
    rb->Connect("lane0", kRoadOrigin, multilane::ArcOffset(100., M_PI / 2.), kEndZ);
    rb->Connect("lane1", kRoadOrigin, multilane::ArcOffset(100., M_PI / 2.), kEndZ);
    rb->Connect("lane2", kRoadOrigin, multilane::ArcOffset(100., M_PI / 2.), kEndZ);
    rb->Connect("lane3", kRoadOrigin, multilane::ArcOffset(100., M_PI / 2.), kEndZ);
  };
  */
  /*
  auto four_arcs_elevation_superelevation = [&]() {
    // Initialize the road from the origin.
    const multilane::EndpointXy kOriginXy{100., 100., 0.};
    const multilane::EndpointZ kFlatZ{0., 0., M_PI / 4., 0.};
    const multilane::EndpointZ kEndZ{10., 0., M_PI / 4., 0.};
    const multilane::Endpoint kRoadOrigin{kOriginXy, kFlatZ};
    rb->Connect("lane0", kRoadOrigin, multilane::ArcOffset(100., M_PI / 2.), kEndZ);
    rb->Connect("lane1", kRoadOrigin, multilane::ArcOffset(100., M_PI / 2.), kEndZ);
    rb->Connect("lane2", kRoadOrigin, multilane::ArcOffset(100., M_PI / 2.), kEndZ);
    rb->Connect("lane3", kRoadOrigin, multilane::ArcOffset(100., M_PI / 2.), kEndZ);
  };
  */
  /*
  four_arcs();
  four_lines();
  four_lines_elevation();
  four_arcs_elevation();
  four_arcs_elevation_superelevation();
  */
  four_lines_elevation_superelevation();
  return rb->Build(api::RoadGeometryId{"test_multilane"});
}

}
}
}


int main(int argc, char* argv[]) {
  drake::log()->debug("main()");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();

  if (FLAGS_obj_file.empty()) {
    drake::log()->critical("No output file specified.");
    return 1;
  }

  drake::log()->info("Loading road geometry.");

  std::unique_ptr<const drake::maliput::api::RoadGeometry> road_geometry =
      drake::maliput::BuildStairLanes();

  drake::maliput::utility::ObjFeatures features;
  features.max_grid_unit = FLAGS_max_grid_unit;
  features.min_grid_resolution = FLAGS_min_grid_resolution;

  drake::log()->info("Generating OBJ.");
  drake::maliput::utility::GenerateObjFile(road_geometry.get(), FLAGS_obj_dir,
                                    FLAGS_obj_file, features);
  return 0;
}