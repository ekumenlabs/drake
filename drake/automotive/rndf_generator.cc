#include "drake/automotive/rndf_generator.h"

#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace automotive {

namespace rndf = maliput::rndf;

std::unique_ptr<const maliput::api::RoadGeometry>
RNDFTBuilder::Build() {
  std::unique_ptr<maliput::rndf::Builder> rb(
      new maliput::rndf::Builder(rc_.lane_bounds, rc_.driveable_bounds,
                                     linear_tolerance_, angular_tolerance_));

  // Initialize the road from the origin.
  const rndf::EndpointXy kOriginXy{0., 0., 0.};
  const rndf::EndpointZ kFlatZ{0., 0., 0., 0.};
  const rndf::EndpointZ kFlatZ90{0., 0., 0., 0.};
  const rndf::Endpoint kRoadOrigin{kOriginXy, kFlatZ};

  // //const double& kLength50 = 50.;
  // //const double& kLength20 = 20.;
  // //const double& kRadius10 = 10.;

  std::vector<rndf::Endpoint> endpoints;

  // endpoints.push_back(rndf::Endpoint(
  //   rndf::EndpointXy(0.0, 0.0, 0.0),
  //   kFlatZ));
  // endpoints.push_back(rndf::Endpoint(
  //   rndf::EndpointXy(50.0, 0.0, 0.0),
  //   kFlatZ));
  // /*const auto& s1l1 =*/ rb->Connect(
  //   "s1l1",
  //   endpoints);
  // endpoints.clear();

  // endpoints.push_back(rndf::Endpoint(
  //   rndf::EndpointXy(50.0, 0.0, 0.0),
  //   kFlatZ));
  // endpoints.push_back(rndf::Endpoint(
  //   rndf::EndpointXy(100.0, 0.0, 0.0),
  //   kFlatZ));
  // /*const auto& s2l1 = */rb->Connect(
  //   "s2l1",
  //   endpoints);
  // endpoints.clear();

  // endpoints.push_back(rndf::Endpoint(
  //   rndf::EndpointXy(50.0, 0.0, 0.0),
  //   kFlatZ));
  // endpoints.push_back(rndf::Endpoint(
  //   rndf::EndpointXy(60.0, 10.0, 0.0),
  //   kFlatZ));
  // /*const auto& s1l1tos3l1 = */rb->Connect(
  //   "s1l1tos3l1",
  //   endpoints);
  // endpoints.clear();


  // endpoints.push_back(rndf::Endpoint(
  //   rndf::EndpointXy(60.0, 10.0, 0.0),
  //   kFlatZ));
  // endpoints.push_back(rndf::Endpoint(
  //   rndf::EndpointXy(60.0, 60.0, 0.0),
  //   kFlatZ));
  // /*const auto& s3l1 = */rb->Connect(
  //   "s3l1",
  //   endpoints);
  // endpoints.clear();

  // endpoints.push_back(rndf::Endpoint(
  //   rndf::EndpointXy(60.0, 60.0, 0.0),
  //   kFlatZ));
  // // endpoints.push_back(rndf::Endpoint(
  // //   rndf::EndpointXy(61.801, 63.7987, 0.0),
  // //   kFlatZ));
  // // endpoints.push_back(rndf::Endpoint(
  // //   rndf::EndpointXy(65.5141, 69.6438, 0.0),
  // //   kFlatZ));
  // // endpoints.push_back(rndf::Endpoint(
  // //   rndf::EndpointXy(72.9029, 75.2812, 0.0),
  // //   kFlatZ));
  // // endpoints.push_back(rndf::Endpoint(
  // //   rndf::EndpointXy(77.5213, 79.2248, 0.0),
  // //   kFlatZ));
  // // endpoints.push_back(rndf::Endpoint(
  // //   rndf::EndpointXy(79.6613, 79.9177, 0.0),
  // //   kFlatZ));
  // endpoints.push_back(rndf::Endpoint(
  //   rndf::EndpointXy(80.0, 80.0, 0.0),
  //   kFlatZ));
  // /*const auto& s3l1tos4l1 = */rb->Connect(
  //   "s3l1tos4l1",
  //   endpoints);

  // endpoints.push_back(rndf::Endpoint(
  //   rndf::EndpointXy(80.0, 80.0, 0.0),
  //   kFlatZ));
  // endpoints.push_back(rndf::Endpoint(
  //   rndf::EndpointXy(100.0, 80.0, 0.0),
  //   kFlatZ));
  // /*const auto& s4l1 = */rb->Connect(
  //   "s4l1",
  //   endpoints);
  // endpoints.clear();

  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(0.0, 0.0, 0.0),
    kFlatZ));
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(0.0, 100.0, 0.0),
    kFlatZ));
  /*const auto& s4l1 = */rb->Connect(
    "s5l1",
    endpoints);
  endpoints.clear();

  return rb->Build({"rndf-T-example"});
}

}  // namespace automotive
}  // namespace drake