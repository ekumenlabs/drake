#include <string>

#include <gflags/gflags.h>

#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/utility/generate_obj.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"

namespace utility = drake::maliput::utility;

DEFINE_string(yaml_file, "",
              "yaml input file defining a monolane road geometry");
DEFINE_string(obj_dir, ".", "Directory to contain rendered road surface");
DEFINE_string(obj_file, "",
              "Basename for output Wavefront OBJ and MTL files");
DEFINE_double(max_grid_unit, utility::ObjFeatures().max_grid_unit,
              "Maximum size of a grid unit in the rendered mesh covering the"
              " road surface");
DEFINE_double(min_grid_resolution, utility::ObjFeatures().min_grid_resolution,
              "Minimum number of grid-units in either lateral or longitudinal"
              " direction in the rendered mesh covering the road surface");

namespace drake {
namespace maliput {
namespace multilane {

std::unique_ptr<const api::RoadGeometry> CreateRoadGeometry() {
  const double kRSpacing{4.};
  const double kLeftShoulder{1.};
  const double kRightShoulder{1.};
  const api::HBounds kElevationBounds{0., 20.};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.01 * M_PI};
  const int kTwoLanes{2};
  const int kThreeLanes{3};
  const EndpointZ kLowFlatZ{0., 0., 0., 0.};

  const Endpoint endpoint_a{{0., 0., 0.}, kLowFlatZ};
  const Endpoint endpoint_b{{50., 0., 0.}, kLowFlatZ};
  const Endpoint endpoint_c{{70., 0., 0.}, kLowFlatZ};
  const Endpoint endpoint_d{{50., 50., -M_PI / 2.}, kLowFlatZ};
  const Endpoint endpoint_e{{50., 14., -M_PI / 2.}, kLowFlatZ};
  const Endpoint endpoint_f{{60., 14., -M_PI / 2.}, kLowFlatZ};
  const Endpoint endpoint_g{{50., -6., -M_PI / 2.}, kLowFlatZ};

  Builder b(kRSpacing, kElevationBounds, kLinearTolerance, kAngularTolerance);
  // Creates connections.
  b.Connect("c1", kThreeLanes, 0, kLeftShoulder, kRightShoulder, endpoint_a,
            50., kLowFlatZ);
  auto c2 = b.Connect("c2", kTwoLanes, 4., kLeftShoulder, kRightShoulder,
                      endpoint_b, 20., kLowFlatZ);
  b.Connect("c3", kTwoLanes, 4., kLeftShoulder, kRightShoulder, endpoint_c, 30.,
            kLowFlatZ);
  auto c4 = b.Connect("c4", kThreeLanes, 0., kLeftShoulder, kRightShoulder,
                      endpoint_b, ArcOffset(6., -M_PI / 2.), kLowFlatZ);
  b.Connect("c5", kTwoLanes, 10., kLeftShoulder, kRightShoulder, endpoint_d,
            36., kLowFlatZ);
  auto c6 = b.Connect("c6", kTwoLanes, 0., kLeftShoulder, kRightShoulder,
                      endpoint_f, ArcOffset(10., M_PI / 2.), kLowFlatZ);
  auto c7 = b.Connect("c7", kTwoLanes, 10., kLeftShoulder, kRightShoulder,
                      endpoint_e, 20., kLowFlatZ);
  b.Connect("c8", kThreeLanes, 6., kLeftShoulder, kRightShoulder, endpoint_g,
            44., kLowFlatZ);
  // Creates the crossing junction.
  std::vector<const Connection*> connections{c2, c4, c6, c7};
  b.MakeGroup("cross", connections);

  return b.Build(api::RoadGeometryId{"multilane-arc-segment"});
/*
  const double kRSpacing = 4.;
  const double kLeftShoulder = 2.;
  const double kRightShoulder = 2.;
  const api::HBounds kElevationBounds(0., 5.);
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.01 * M_PI;
  Builder b(kRSpacing, kElevationBounds, kLinearTolerance, kAngularTolerance);

  const EndpointZ kLowFlatZ(0., 0., 0., 0.);
  const EndpointZ kMidFlatZ(3., 0., 0., 0.);
  const EndpointZ kMidTiltLeftZ(3., 0., -0.4, 0.);
  const EndpointZ kMidTiltRightZ(3., 0., 0.4, 0.);
  const EndpointZ kHighFlatZ(6., 0., 0., 0.);

  const ArcOffset kCounterClockwiseArc(50., 0.75 * M_PI);  // 135deg, 50m radius
  const ArcOffset kClockwiseArc(50., -0.75 * M_PI);  // 135deg, 50m radius

  Endpoint start {{0., 0., -M_PI / 4.}, kLowFlatZ};
  auto c0 = b.Connect("0", 1, 0., kLeftShoulder, kRightShoulder, start, 50.,
                      kMidFlatZ);

  auto c1 = b.Connect("1", 1, 0., kLeftShoulder, kRightShoulder, c0->end(),
                      kCounterClockwiseArc, kMidTiltLeftZ);
  auto c2 = b.Connect("2", 1, 0., kLeftShoulder, kRightShoulder, c1->end(),
                      kCounterClockwiseArc, kMidFlatZ);

  auto c3 = b.Connect("3", 1, 0., kLeftShoulder, kRightShoulder, c2->end(), 50.,
                      kHighFlatZ);
  auto c4 = b.Connect("4", 1, 0., kLeftShoulder, kRightShoulder, c3->end(), 50.,
                      kMidFlatZ);

  auto c5 = b.Connect("5", 1, 0., kLeftShoulder, kRightShoulder, c4->end(),
                      kClockwiseArc, kMidTiltRightZ);
  auto c6 = b.Connect("6", 1, 0., kLeftShoulder, kRightShoulder, c5->end(),
                      kClockwiseArc, kMidFlatZ);

  // Tweak ends to check if fuzzy-matching is working.
  Endpoint c6end = c6->end();
  c6end = Endpoint(EndpointXy(c6end.xy().x() + kLinearTolerance * 0.5,
                              c6end.xy().y() - kLinearTolerance * 0.5,
                              c6end.xy().heading()),
                   EndpointZ(c6end.z().z() + kLinearTolerance * 0.5,
                             c6end.z().z_dot(),
                             c6end.z().theta(), c6end.z().theta_dot()));
  EndpointZ c0start_z = c0->start().z();
  c0start_z = EndpointZ(c0start_z.z() - kLinearTolerance * 0.5,
                        c0start_z.z_dot(),
                        c0start_z.theta(), c0start_z.theta_dot());

  b.Connect("7", 1, 0., kLeftShoulder, kRightShoulder, c6end, 50., c0start_z);

  return b.Build(api::RoadGeometryId{"figure-eight"});
  */
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
  auto rg = drake::maliput::multilane::CreateRoadGeometry();


  utility::ObjFeatures features;
  features.max_grid_unit = FLAGS_max_grid_unit;
  features.min_grid_resolution = FLAGS_min_grid_resolution;

  drake::log()->info("Generating OBJ.");
  utility::GenerateObjFile(rg.get(), FLAGS_obj_dir, FLAGS_obj_file, features);

  return 0;
}
