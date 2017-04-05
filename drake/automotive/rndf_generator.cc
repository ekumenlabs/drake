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

  const double& kLength = 50.;

  const auto& s1l1 = rb->Connect(
  	"s1l1",
  	kRoadOrigin,
  	kLength,
  	kFlatZ);
  /*const auto& s2l1 = */rb->Connect(
  	"s2l1",
  	s1l1->end(),
  	kLength,
  	kFlatZ);

  const double& kRadius = 10.;
  const auto& s1l1tos3l1 = rb->Connect(
  	"s1l1tos3l1",
  	s1l1->end(),
  	rndf::ArcOffset(kRadius, M_PI/2.0),
  	kFlatZ);

  /*const auto& s2l1 = */rb->Connect(
  	"s3l1",
  	s1l1tos3l1->end(),
  	kLength,
  	kFlatZ);

  return rb->Build({"rndf-T-example"});
}

}  // namespace automotive
}  // namespace drake