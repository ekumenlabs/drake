#include <cmath>
#include <memory>
#include <utility>

#include "drake/automotive/maliput/multilane/cubic_polynomial.h"
#include "drake/automotive/maliput/multilane/line_road_curve.h"
#include "drake/automotive/maliput/multilane/road_curve.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace maliput {
namespace multilane {

template<typename T>
class RoadCurveSystem : public systems::LeafSystem<T> {
};

template<>
class RoadCurveSystem<double> : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadCurveSystem)

  /// A constructor that initializes the system.
  RoadCurveSystem(const RoadCurve* rc, double r, double h) :
      rc_(rc), r_(r), h_(h) {
    // Adding one generalized position and one generalized velocity.
    this->DeclareContinuousState(1, 1, 0);
    // A 1D output vector for position and velocity.
    this->DeclareVectorOutputPort(drake::systems::BasicVector<double>(1),
                                  &RoadCurveSystem::CopyStateOut);
  }

  std::unique_ptr<drake::systems::Context<double>>
  CreateContext(const double& s0) const {
    auto context = this->AllocateContext();
    // Set continuous state.
    drake::systems::VectorBase<double>* cstate =
        context->get_mutable_continuous_state_vector();
    cstate->SetAtIndex(0, s0);
    return context;
  }

 protected:
  void CopyStateOut(const drake::systems::Context<double>& context,
                    drake::systems::BasicVector<double>* output) const {
    // Get current state from context.
    const drake::systems::VectorBase<double>& continuous_state_vector =
        context.get_continuous_state_vector();
    // Write system output.
    output->set_value(continuous_state_vector.CopyToVector());
  }

  void DoCalcTimeDerivatives(const systems::Context<double>& context,
      systems::ContinuousState<double>* derivatives) const override {
    // Obtain the structure we need to write into.
    drake::systems::VectorBase<double>* const derivatives_vector =
        derivatives->get_mutable_vector();

    const double p = context.get_time();
    const double s_dot = rc_->W_prime_of_prh(p, r_, h_, rc_->Rabg_of_p(p),
    	                                     rc_->elevation().f_dot_p(p))
                              .norm();
    derivatives_vector->SetAtIndex(0, s_dot);
  }

 private:
  const RoadCurve* rc_;
  const double r_{};
  const double h_{};
};

enum GeometryType{
  FLAT_LINE,
  LINEAR_ELEVATION_LINE,
  QUADRATIC_ELEVATION_LINE,
  CUBIC_ELEVATION_LINE,
  CUBIC_ELEV_LINEAR_SUPER_LINE,
};

double GetOutput(const drake::systems::Simulator<double>* simulator) {
  return simulator->get_context().get_continuous_state()->
                    get_generalized_position()[0];
}

std::unique_ptr<RoadCurve> CreateFlatLine() {
  const Vector2<double> kOrigin{0., 0.};
  const Vector2<double> kDirection{10., 0.};
  const CubicPolynomial kElevation{0., 0., 0., 0.};
  const CubicPolynomial kSuperelevation{0., 0., 0., 0.};
  return std::make_unique<LineRoadCurve>(kOrigin, kDirection, kElevation,
                                         kSuperelevation);
}

std::unique_ptr<RoadCurve> CreateLinearElevationLine() {
  const Vector2<double> kOrigin{0., 0.};
  const Vector2<double> kDirection{10., 0.};
  const CubicPolynomial kElevation{0., 1., 0., 0.};
  const CubicPolynomial kSuperelevation{0., 0., 0., 0.};
  return std::make_unique<LineRoadCurve>(kOrigin, kDirection, kElevation,
                                         kSuperelevation);
}

std::unique_ptr<RoadCurve> CreateQuadraticElevationLine() {
  const Vector2<double> kOrigin{0., 0.};
  const Vector2<double> kDirection{10., 0.};
  const CubicPolynomial kElevation{1., 1., 1., 0.};
  const CubicPolynomial kSuperelevation{0., 0., 0., 0.};
  return std::make_unique<LineRoadCurve>(kOrigin, kDirection, kElevation,
                                         kSuperelevation);
}

std::unique_ptr<RoadCurve> CreateCubicElevationLine() {
  const Vector2<double> kOrigin{0., 0.};
  const Vector2<double> kDirection{10., 0.};
  const CubicPolynomial kElevation{0., -1., 1., 1.};
  const CubicPolynomial kSuperelevation{0., 0., 0., 0.};
  return std::make_unique<LineRoadCurve>(kOrigin, kDirection, kElevation,
                                         kSuperelevation);
}

std::unique_ptr<RoadCurve> CreateCubicElevLinearSuperLine() {
  const Vector2<double> kOrigin{0., 0.};
  const Vector2<double> kDirection{10., 0.};
  const CubicPolynomial kElevation{0., -1., 1., 1.};
  const CubicPolynomial kSuperelevation{
      0., M_PI / 2. / kDirection.norm(), 0., 0.};
  return std::make_unique<LineRoadCurve>(kOrigin, kDirection, kElevation,
                                         kSuperelevation);
}

double Integrate(double p0, double step, int steps, const RoadCurve* const rc,
                 double r, double h) {
  double l{0.};
  double p{p0};
  for (int i = 0; i < steps; i++, p += step) {
    const Rot3 R = rc->Rabg_of_p(p);
    const double real_g_prime = rc->elevation().f_dot_p(p);
    const double ds_dsigma =
        rc->W_prime_of_prh(p, r, h, R, real_g_prime).norm();
    l += (ds_dsigma * step);
  }
  return l;
}


void TestIntegration(const GeometryType& geo_type) {
  const double kR0{5.};
  const double kH0{0.};
  const double kS0{0.};
  const double kPEnd{1.};
  const double kTargetFrameRate{1.};

  std::unique_ptr<RoadCurve> rc;
  switch(geo_type) {
    case GeometryType::FLAT_LINE:
      rc = CreateFlatLine();
      break;
    case GeometryType::LINEAR_ELEVATION_LINE:
      rc = CreateLinearElevationLine();
      break;
    case GeometryType::QUADRATIC_ELEVATION_LINE:
      rc = CreateQuadraticElevationLine();
      break;
    case GeometryType::CUBIC_ELEVATION_LINE:
      rc = CreateCubicElevationLine();
      break;
    case GeometryType::CUBIC_ELEV_LINEAR_SUPER_LINE:
      rc = CreateCubicElevLinearSuperLine();
      break;
    default:
      std::cerr << "geo_type is not recognized." << std::endl;
      return;
  }

  // Computes the simulator's integration.
  auto rc_system =
      std::make_unique<RoadCurveSystem<double>>(rc.get(), kR0, kH0);
  auto context = rc_system->CreateContext(kS0);
  auto simulator =
      std::make_unique<drake::systems::Simulator<double>>(*rc_system,
                                                          std::move(context));
  simulator->set_target_realtime_rate(kTargetFrameRate);
  simulator->Initialize();
  // Runs the simulation --> Integrates the system.
  simulator->StepTo(kPEnd);

  // Computes a manual integration.
  const int kSteps = 10000;
  const double kP0{0.};
  const double kStep{(kPEnd - kP0) / static_cast<double>(kSteps)};
  const double manual_integration =
      Integrate(kP0, kStep, kSteps, rc.get(), kR0, kH0);
  // Present integration values.
  std::cout << "Simulator's integration output: " <<
                GetOutput(simulator.get()) << std::endl;
  std::cout << "Manual integration output: " <<
                manual_integration << std::endl;
}

}
}
}

int main(int argc, char *argv[]) {
  using drake::maliput::multilane::GeometryType;
  std::cout << "FLAT_LINE" << std::endl;
  drake::maliput::multilane::TestIntegration(GeometryType::FLAT_LINE);
  std::cout << "---------------------" << std::endl;
  std::cout << "LINEAR_ELEVATION_LINE" << std::endl;
  drake::maliput::multilane::TestIntegration(
      GeometryType::LINEAR_ELEVATION_LINE);
  std::cout << "---------------------" << std::endl;
  std::cout << "QUADRATIC_ELEVATION_LINE" << std::endl;
  drake::maliput::multilane::TestIntegration(
      GeometryType::QUADRATIC_ELEVATION_LINE);
  std::cout << "---------------------" << std::endl;
  std::cout << "CUBIC_ELEVATION_LINE" << std::endl;
  drake::maliput::multilane::TestIntegration(
      GeometryType::CUBIC_ELEVATION_LINE);
  std::cout << "---------------------" << std::endl;
  std::cout << "CUBIC_ELEV_LINEAR_SUPER_LINE" << std::endl;
  drake::maliput::multilane::TestIntegration(
      GeometryType::CUBIC_ELEV_LINEAR_SUPER_LINE);
  return 0;
}