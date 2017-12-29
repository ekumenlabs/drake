/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/lane.h"
/* clang-format on */

#include <cmath>
#include <random>

#include <gtest/gtest.h>
#include <benchmark/benchmark.h>

#include "drake/automotive/maliput/multilane/arc_road_curve.h"
#include "drake/automotive/maliput/multilane/junction.h"
#include "drake/automotive/maliput/multilane/lane.h"
#include "drake/automotive/maliput/multilane/line_road_curve.h"
#include "drake/automotive/maliput/multilane/road_curve.h"
#include "drake/automotive/maliput/multilane/road_geometry.h"
#include "drake/automotive/maliput/multilane/segment.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace benchmarks {

namespace {

void LaneEvalMotionDerivativesBenchmark(
    benchmark::State& state, const api::Lane* lane,
    const api::LanePosition& position) {
  const api::IsoLaneVelocity velocity{1., 1., 1.};
  for (auto _ : state) {
    benchmark::DoNotOptimize(lane->EvalMotionDerivatives(position, velocity));
  }
}

void LaneToGeoPositionBenchmark(
    benchmark::State& state, const api::Lane* lane,
    const api::LanePosition& position) {
  for (auto _ : state) {
    benchmark::DoNotOptimize(lane->ToGeoPosition(position));
  }
}

void LaneGetOrientationBenchmark(
    benchmark::State& state, const api::Lane* lane,
    const api::LanePosition& position) {
  for (auto _ : state) {
    benchmark::DoNotOptimize(lane->GetOrientation(position));
  }
}

class Vehicle {
 public:
  Vehicle(const api::Lane* lane,
          const api::LanePosition& position,
          const api::IsoLaneVelocity& velocity)
      : lane_(lane),
        lane_length_(lane_->length()),
        position_(position),
        velocity_(velocity) {
    DRAKE_ASSERT(velocity_.rho_v == 0.0);
    DRAKE_ASSERT(velocity_.eta_v == 0.0);
  }

  void move(double dt) {
    const api::LanePosition motion_derivatives =
        lane_->EvalMotionDerivatives(position_, velocity_);
    double next_s = position_.s() + motion_derivatives.s() * dt;
    if (next_s > lane_length_) next_s = std::remainder(next_s, lane_length_);
    position_.set_s(next_s);
  }

 private:
  const api::Lane* lane_;
  const double lane_length_;
  api::LanePosition position_;
  const api::IsoLaneVelocity velocity_;
};

std::vector<Vehicle> PopulateRoadWithTraffic(
    const std::vector<const api::Lane*> road_lanes,
    int number_of_vehicles, int traffic_distribution_key) {
  std::vector<Vehicle> vehicles;

  std::mt19937 random_engine(traffic_distribution_key);
  std::uniform_int_distribution<int> nth_lane_chooser(0, road_lanes.size() - 1);
  const api::IsoLaneVelocity velocity{1., 0., 0.};
  for (int i = 0 ; i < number_of_vehicles ; ++i) {
    const int lane_index = nth_lane_chooser(random_engine);
    const api::Lane* chosen_lane = road_lanes[lane_index];
    std::uniform_real_distribution<double>
        lane_s_coord_chooser(0., chosen_lane->length());
    const api::LanePosition chosen_position{
      lane_s_coord_chooser(random_engine), 0., 0.};
    vehicles.emplace_back(chosen_lane, chosen_position, velocity);
  }

  return vehicles;
}

void RoadTrafficBenchmark(benchmark::State& state,
                          const std::vector<const api::Lane*>& road_lanes,
                          int traffic_distribution_key) {
  const double dt = 1./25;
  int motion_calls = 0;
  std::vector<Vehicle> vehicles = PopulateRoadWithTraffic(
      road_lanes, state.range(0), traffic_distribution_key);
  for (auto _ : state) {
    for (Vehicle& vehicle : vehicles) {
      vehicle.move(dt);
      motion_calls++;
    }
  }
  state.counters["motion_calls"] = benchmark::Counter(
      motion_calls, benchmark::Counter::kIsRate);
}

benchmark::internal::Benchmark* RegisterLaneBenchmark(
    const char* name, std::function<void(
        benchmark::State& st, const api::Lane*, const api::LanePosition&)> fn,
    const api::Lane* lane, int ssteps, int rsteps, int hsteps) {
  benchmark::internal::Benchmark* b = benchmark::RegisterBenchmark(
      name, [fn, lane, ssteps, rsteps, hsteps](benchmark::State& st) {
        const double s = st.range(0) * lane->length() / (ssteps - 1);
        const api::RBounds rbounds = lane->lane_bounds(s);
        const double r = st.range(1) * (
            rbounds.max() - rbounds.min()) / (rsteps - 1) + rbounds.min();
        const api::HBounds hbounds = lane->elevation_bounds(s, r);
        const double h = st.range(2) * (
            hbounds.max() - hbounds.min()) / (hsteps - 1) + hbounds.min();
        fn(st, lane, {s, r, h});
      });
  for (int is = 0 ; is < ssteps; ++is) {
    for (int ir = 0 ; ir < rsteps ; ++ir) {
      for (int ih = 0 ; ih < hsteps ; ++ih) {
        b->Args({is, ir, ih});
      }
    }
  }
  return b;
}

const double kLinearTolerance = 1e-6;
const double kAngularTolerance = 1e-6;

std::unique_ptr<api::RoadGeometry> MakeRollercoasterRoad() {
  const double max_height = 5.;
  const double half_lane_width = 5.;
  const double half_segment_width = 15.;

  const int tunnel_turns = 10;
  const double tunnel_length = 100.;
  const CubicPolynomial tunnelp(0., 2. * M_PI * tunnel_turns / tunnel_length,
                                0., 0.);
  const CubicPolynomial zp(0., 0., 0., 0.);

  auto road_geometry = std::make_unique<RoadGeometry>(
      api::RoadGeometryId{"RollercoasterRoad"},
      kLinearTolerance, kAngularTolerance);

  Junction* j1 = road_geometry->NewJunction(api::JunctionId{"j1"});

  auto road_curve = std::make_unique<LineRoadCurve>(
      Vector2<double>(0., 0.), Vector2<double>(tunnel_length, 0.), zp, tunnelp);
  Segment* s1 = j1->NewSegment(api::SegmentId{"s1"}, std::move(road_curve),
                               -half_segment_width, half_segment_width,
                               {0., max_height});

  s1->NewLane(api::LaneId{"RightHandLane"}, -10.0,
              {-half_lane_width, half_lane_width});
  s1->NewLane(api::LaneId{"CenterLane"}, 0.0,
              {-half_lane_width, half_lane_width});
  s1->NewLane(api::LaneId{"LeftHandLane"}, 10.0,
              {-half_lane_width, half_lane_width});

  return std::move(road_geometry);
}

std::unique_ptr<api::RoadGeometry> MakeHillRoad() {
  const double z0 = 0.;
  const double z1 = 20.;
  const double radius = 100.;
  const double theta0 = 0.25 * M_PI;
  const double d_theta = 0.5 * M_PI;
  const double p_scale = radius * d_theta;

  const double half_segment_width = 15.;
  const double half_lane_width = 5.;
  const double max_height = 5.;
  // A cubic polynomial such that:
  //   f(0) = (z0 / p_scale), f(1) = (z1 / p_scale), and f'(0) = f'(1) = 0.
  const CubicPolynomial hillp(z0 / p_scale, 0.,
                              (3. * (z1 - z0) / p_scale),
                              (-2. * (z1 - z0) / p_scale));
  const CubicPolynomial zp(0., 0., 0., 0.);

  auto road_geometry = std::make_unique<RoadGeometry>(
      api::RoadGeometryId{"HillRoad"},
      kLinearTolerance, kAngularTolerance);

  Junction* j1 = road_geometry->NewJunction(api::JunctionId{"j1"});

  auto road_curve =
      std::make_unique<ArcRoadCurve>(Vector2<double>(-100., -100.), radius,
                                     theta0, d_theta, hillp, zp);
  Segment* s1 = j1->NewSegment(api::SegmentId{"s1"}, std::move(road_curve),
                               -half_segment_width, half_segment_width,
                               {0., max_height});

  s1->NewLane(api::LaneId{"RightHandLane"}, -10.0,
              {-half_lane_width, half_lane_width});
  s1->NewLane(api::LaneId{"CenterLane"}, 0.0,
              {-half_lane_width, half_lane_width});
  s1->NewLane(api::LaneId{"LeftHandLane"}, 10.0,
              {-half_lane_width, half_lane_width});

  return std::move(road_geometry);
}

}  // namespace

const int kStepsInS = 25;
const int kStepsInR = 10;
const int kStepsInH = 5;
const int kMaxNumberOfVehicles = 120;

int main(int argc, char** argv) {
  std::vector<std::unique_ptr<api::RoadGeometry>> road_geometries;
  road_geometries.push_back(MakeHillRoad());
  road_geometries.push_back(MakeRollercoasterRoad());
  for (const auto& road_geometry : road_geometries) {
    std::vector<const api::Lane*> road_lanes;
    for (int i = 0 ; i < road_geometry->num_junctions() ; ++i) {
      const api::Junction* junction = road_geometry->junction(i);
      for (int j = 0 ; j < junction->num_segments() ; ++j) {
        const api::Segment* segment = junction->segment(j);
        for (int k = 0 ; k < segment->num_lanes() ; ++k) {
          const api::Lane* lane = segment->lane(k);
          RegisterLaneBenchmark(
              (road_geometry->id().string()
               + "_" + lane->id().string()
               + "_EvalMotionDerivatives_Benchmark").c_str(),
              LaneEvalMotionDerivativesBenchmark,
              lane, kStepsInS, kStepsInR, kStepsInH)->Iterations(1);
          RegisterLaneBenchmark(
              (road_geometry->id().string()
               + "_" + lane->id().string()
               + "_ToGeoPosition_Benchmark").c_str(),
              LaneToGeoPositionBenchmark,
              lane, kStepsInS, kStepsInR, kStepsInH)->Iterations(1);
          RegisterLaneBenchmark(
              (road_geometry->id().string()
               + "_" + lane->id().string()
               + "_GetOrientation_Benchmark").c_str(),
              LaneGetOrientationBenchmark,
              lane, kStepsInS, kStepsInR, kStepsInH)->Iterations(1);
          road_lanes.push_back(lane);
        }
      }
    }
    RegisterBenchmark((
        road_geometry->id().string() +
        "_Traffic_Distribution1_Benchmark").c_str(),
        RoadTrafficBenchmark, road_lanes, 1)->DenseRange(
            1, kMaxNumberOfVehicles)->Unit(benchmark::kMicrosecond);
    RegisterBenchmark((
        road_geometry->id().string() +
        "_Traffic_Distribution2_Benchmark").c_str(),
        RoadTrafficBenchmark, road_lanes, 2)->DenseRange(
            1, kMaxNumberOfVehicles)->Unit(benchmark::kMicrosecond);
    RegisterBenchmark((
        road_geometry->id().string() +
        "_Traffic_Distribution3_Benchmark").c_str(),
        RoadTrafficBenchmark, road_lanes, 3)->DenseRange(
            1, kMaxNumberOfVehicles)->Unit(benchmark::kMicrosecond);
  }
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();

  return 0;
}

}  // namespace benchmarks
}  // namespace multilane
}  // namespace maliput
}  // namespace drake

int main(int argc, char **argv) {
  return drake::maliput::multilane::benchmarks::main(argc, argv);
}
