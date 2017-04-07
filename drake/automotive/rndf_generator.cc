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
  const rndf::EndpointZ kFlatZ{0., 0., 0., 0.};
  std::vector<rndf::Endpoint> endpoints;

  // |
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(0.0, 0.0, 0.0),
    kFlatZ));
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(50.0, 0.0, 0.0),
    kFlatZ));
  rb->Connect("s1l1", endpoints);
  endpoints.clear();

  // |
  // |
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(50.0, 0.0, 0.0),
    kFlatZ));
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(100.0, 0.0, 0.0),
    kFlatZ));
  rb->Connect("s2l1", endpoints);
  endpoints.clear();

  // |
  //  -
  // |
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(50.0, 0.0, 0.0),
    kFlatZ));
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(55, 1.34, 30),
    kFlatZ));
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(58.66, 5.0, 60),
    kFlatZ));
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(60.0, 10.0, 90),
    kFlatZ));
  rb->Connect("s1l1tos3l1", endpoints);
  endpoints.clear();

  // |
  //  - --
  // |
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(60.0, 10.0, 90.0),
    kFlatZ));
  endpoints.push_back(rndf::Endpoint(
    rndf::EndpointXy(60.0, 60.0, 90.0),
    kFlatZ));
  rb->Connect("s3l1", endpoints);
  endpoints.clear();

  return rb->Build({"rndf-T-example"});
}

}  // namespace automotive
}  // namespace drake