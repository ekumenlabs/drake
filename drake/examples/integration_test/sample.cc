#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

#include <cmath>
#include <memory>
#include <utility>


#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/multilane/cubic_polynomial.h"
#include "drake/automotive/maliput/multilane/lane.h"
#include "drake/automotive/maliput/multilane/line_road_curve.h"
#include "drake/automotive/maliput/multilane/junction.h"
#include "drake/automotive/maliput/multilane/road_geometry.h"
#include "drake/automotive/maliput/multilane/segment.h"
#include "drake/automotive/maliput/multilane/road_curve.h"
#include "drake/automotive/maliput/utility/generate_obj.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
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
    // A 1D output vector for position.
    this->DeclareVectorOutputPort(drake::systems::BasicVector<double>(1),
                                  &RoadCurveSystem::CopyStateOut);
  }

  std::unique_ptr<drake::systems::Context<double>>
  CreateContext(double s0) const {
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

    // Retrieve the parameter, given as time.
    const double p = context.get_time();
    // Compute the arc length derivative with respect to p as the norm
    // of the world function derivative with respect to p.
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

template<typename T>
class InverseRoadCurveSystem : public systems::LeafSystem<T> {
};


template<>
class InverseRoadCurveSystem<double> : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseRoadCurveSystem)

  /// A constructor that initializes the system.
  InverseRoadCurveSystem(const RoadCurve* rc, double r, double h) :
    rc_(rc), r_(r), h_(h) {
    // Adding one generalized position and one generalized velocity.
    this->DeclareContinuousState(1, 1, 0);
    // A 1D output vector for position.
    this->DeclareVectorOutputPort(drake::systems::BasicVector<double>(1),
                                  &InverseRoadCurveSystem::CopyStateOut);
  }

  std::unique_ptr<drake::systems::Context<double>>
  CreateContext(double p0) const {
    auto context = this->AllocateContext();
    // Set continuous state.
    drake::systems::VectorBase<double>* cstate =
        context->get_mutable_continuous_state_vector();
    cstate->SetAtIndex(0, p0);
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
    // Get current state from context.
    const systems::VectorBase<double>& continuous_state_vector =
        context.get_continuous_state_vector();
    // Retrieve current p parameter.
    const double p = continuous_state_vector.GetAtIndex(0);
    // Compute the derivatve of the parameter p with respect to the
    // arc length as the reciprocal of the derivative of the world
    // function derivative with respect to the current p parameter.
    const double p_dot = 1./rc_->W_prime_of_prh(
        p, r_, h_, rc_->Rabg_of_p(p),
        rc_->elevation().f_dot_p(p)).norm();
    // Obtain the structure we need to write into.
    drake::systems::VectorBase<double>* const derivatives_vector =
        derivatives->get_mutable_vector();
    derivatives_vector->SetAtIndex(0, p_dot);
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
  const Vector2<double> kDirection{100., 0.};
  const CubicPolynomial kElevation{0., 0., 0., 0.};
  const CubicPolynomial kSuperelevation{0., 0., 0., 0.};
  return std::make_unique<LineRoadCurve>(kOrigin, kDirection, kElevation,
                                         kSuperelevation);
}

std::unique_ptr<RoadCurve> CreateLinearElevationLine() {
  const Vector2<double> kOrigin{0., 0.};
  const Vector2<double> kDirection{100., 0.};
  const CubicPolynomial kElevation{0., 1., 0., 0.};
  const CubicPolynomial kSuperelevation{0., 0., 0., 0.};
  return std::make_unique<LineRoadCurve>(kOrigin, kDirection, kElevation,
                                         kSuperelevation);
}

std::unique_ptr<RoadCurve> CreateQuadraticElevationLine() {
  const Vector2<double> kOrigin{0., 0.};
  const Vector2<double> kDirection{100., 0.};
  const CubicPolynomial kElevation{1., 1., 1., 0.};
  const CubicPolynomial kSuperelevation{0., 0., 0., 0.};
  return std::make_unique<LineRoadCurve>(kOrigin, kDirection, kElevation,
                                         kSuperelevation);
}

std::unique_ptr<RoadCurve> CreateCubicElevationLine() {
  const Vector2<double> kOrigin{0., 0.};
  const Vector2<double> kDirection{100., 0.};
  const CubicPolynomial kElevation{0., -1., 1., 1.};
  const CubicPolynomial kSuperelevation{0., 0., 0., 0.};
  return std::make_unique<LineRoadCurve>(kOrigin, kDirection, kElevation,
                                         kSuperelevation);
}

std::unique_ptr<RoadCurve> CreateCubicElevLinearSuperLine() {
  const Vector2<double> kOrigin{0., 0.};
  const Vector2<double> kDirection{100., 0.};
  const CubicPolynomial kElevation{0., -1., 1., 1.};
  const CubicPolynomial kSuperelevation{
      0., M_PI / 2. / kDirection.norm(), 0., 0.};
  return std::make_unique<LineRoadCurve>(kOrigin, kDirection, kElevation,
                                         kSuperelevation);
}

// GSL does not play along with any form of callable object besides raw
// function pointers. The template functions below enable its use with
// functors and lambdas.
template <typename T>
int unwrap_gsl_odeiv2_function(double t, const double y[],
                               double dydt[], void *params) {
  auto fp = static_cast<T*>(params);
  return (*fp)(t, y, dydt);
}

template <typename T>
gsl_odeiv2_system make_gsl_odeiv2_system(T *function, size_t dim)
{
  return {unwrap_gsl_odeiv2_function<T>, nullptr, dim, function};
}

double IntegrateArcLengthRK8(const RoadCurve* rc, double p, double r, double h,
                             double initial_step, double reltol) {
  const size_t kDim = 1;
  gsl_odeiv2_step *step = gsl_odeiv2_step_alloc(gsl_odeiv2_step_rk8pd, kDim);
  gsl_odeiv2_control *control = gsl_odeiv2_control_y_new(0.0, reltol);
  gsl_odeiv2_evolve *evolve = gsl_odeiv2_evolve_alloc(kDim);
  auto arc_length_function = [rc, r, h](double p_,
                                        const double s_[],
                                        double dsdp_[]) {
    unused(s_);
    *dsdp_ = rc->W_prime_of_prh(
        p_, r, h, rc->Rabg_of_p(p_),
        rc->elevation().f_dot_p(p_)).norm();
    return GSL_SUCCESS;
  };
  gsl_odeiv2_system system = make_gsl_odeiv2_system(&arc_length_function, kDim);

  double s = 0.0;
  double tau = 0.0;
  while (tau < p) {
    int status = gsl_odeiv2_evolve_apply(evolve, control, step, &system,
                                         &tau, p, &initial_step, &s);
    if (status != GSL_SUCCESS) {
      s = std::numeric_limits<double>::quiet_NaN();
      break;
    }
  }
  gsl_odeiv2_evolve_free(evolve);
  gsl_odeiv2_control_free(control);
  gsl_odeiv2_step_free(step);
  return s;
}

double IntegrateParameterRK8(const RoadCurve* rc, double s, double r, double h,
                             double initial_step, double reltol) {
  const size_t kDim = 1;
  gsl_odeiv2_step *step = gsl_odeiv2_step_alloc(gsl_odeiv2_step_rk8pd, kDim);
  gsl_odeiv2_control *control = gsl_odeiv2_control_y_new(0.0, reltol);
  gsl_odeiv2_evolve *evolve = gsl_odeiv2_evolve_alloc(kDim);
  auto arc_length_function = [rc, r, h](double s_,
                                        const double p_[],
                                        double dpds_[]) {
    unused(s_);
    *dpds_ = 1./rc->W_prime_of_prh(*p_, r, h, rc->Rabg_of_p(*p_),
                                   rc->elevation().f_dot_p(*p_)).norm();
    return GSL_SUCCESS;
  };
  gsl_odeiv2_system system = make_gsl_odeiv2_system(&arc_length_function, kDim);

  double p = 0.0;
  double tau = 0.0;
  while (tau < s) {
    int status = gsl_odeiv2_evolve_apply(evolve, control, step, &system,
                                         &tau, s, &initial_step, &p);
    if (status != GSL_SUCCESS) {
      p = std::numeric_limits<double>::quiet_NaN();
      break;
    }
  }
  gsl_odeiv2_evolve_free(evolve);
  gsl_odeiv2_control_free(control);
  gsl_odeiv2_step_free(step);
  return p;
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

void CreateMesh(const std::string& mesh_path, const std::string& mesh_name,
                const GeometryType& geo_type) {
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

  const double kLinearTolerance = 1e-6;
  const double kAngularTolerance = 1e-6;
  const double kHalfWidth{10.};
  const double kMaxHeight{20.};
  const double kHalfLaneWidth{5.};
  const double kR0{5.};
  RoadGeometry rg(api::RoadGeometryId{"apple"},
                  kLinearTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction(api::JunctionId{"j1"})
                    ->NewSegment(api::SegmentId{"s1"}, std::move(rc),
                                 -kHalfWidth + kR0, kHalfWidth + kR0,
                                 {0., kMaxHeight});
  s1->NewLane(api::LaneId{"l1"}, kR0, {-kHalfLaneWidth, kHalfLaneWidth});

  utility::ObjFeatures features;
  features.max_grid_unit = utility::ObjFeatures().max_grid_unit;
  features.min_grid_resolution =  utility::ObjFeatures().min_grid_resolution;
  utility::GenerateObjFile(&rg, mesh_path, mesh_name, features);
}


void TestIntegration(const GeometryType& geo_type) {
  const double kR0{5.};
  const double kH0{0.};
  const double kS0{0.};
  const double kP0{0.};
  const double kPHalf{0.5};
  const double kPEnd{1.};
  const double kTargetFrameRate{1.};
  const double kInitialStep{1e-5};
  const double kAccuracy{1e-8};

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

  // Initializes the simulator for p-to-s integration of the road curve.
  auto rc_system =
      std::make_unique<RoadCurveSystem<double>>(rc.get(), kR0, kH0);
  auto context = rc_system->CreateContext(kS0);
  auto simulator =
      std::make_unique<drake::systems::Simulator<double>>(*rc_system,
                                                          std::move(context));
  simulator->set_target_realtime_rate(kTargetFrameRate);
  simulator->Initialize();

  // Initializes the simulator for s-to-p integration of the road curve.
  auto irc_system =
      std::make_unique<InverseRoadCurveSystem<double>>(rc.get(), kR0, kH0);
  auto icontext = irc_system->CreateContext(kP0);
  auto isimulator =
      std::make_unique<drake::systems::Simulator<double>>(*irc_system,
                                                          std::move(icontext));
  // Increases realtime rate to speed up the integration process.
  isimulator->set_target_realtime_rate(kTargetFrameRate * 100.);
  isimulator->Initialize();

  // Approximates the p-to-s function at half the parameter interval using
  // Drake's RK3 integrator.
  simulator->StepTo(kPHalf);
  const double s_at_half_p_using_rk3 = GetOutput(simulator.get());

  // Uses the previous result to approximate the s-to-p function at the same
  // spot using Drake's RK3 integrator.
  isimulator->StepTo(s_at_half_p_using_rk3);
  const double p_at_s_at_half_p_using_rk3 = GetOutput(isimulator.get());

  // Approximates the p-to-s function at half the parameter interval using GSL's
  // RK8 integrator.
  const double s_at_half_p_using_rk8 = IntegrateArcLengthRK8(
      rc.get(), kPHalf, kR0, kH0, kInitialStep, kAccuracy);
  // Uses the previous result to approximate the s-to-p function at the same
  // spot using GSL's RK8 integrator.
  const double p_at_s_at_half_p_using_rk8 = IntegrateParameterRK8(
      rc.get(), s_at_half_p_using_rk8, kR0, kH0, kInitialStep, kAccuracy);

  std::cout.precision(12);

  std::cout << "Drake's RK3 approximate for s(p = " << kPHalf << ") = "
            << s_at_half_p_using_rk3 << std::endl;
  std::cout << "GSL's RK8 approximate for s(p = " << kPHalf << ") = "
            << s_at_half_p_using_rk8 << std::endl;
  std::cout << "Drake's RK3 approximate for p(s = "
            << s_at_half_p_using_rk3 << ") = "
            << p_at_s_at_half_p_using_rk3 << std::endl;
  std::cout << "GSL's RK8 approximate for p(s = "
            << s_at_half_p_using_rk8 << ") = "
            << p_at_s_at_half_p_using_rk8 << std::endl;

  // Approximates the p-to-s function at the parameter interval end using
  // Drake's RK3 integrator.
  simulator->StepTo(kPEnd);
  const double s_at_p_end_using_rk3 = GetOutput(simulator.get());

  // Uses the previous result to approximate the s-to-p function at the same
  // spot using Drake's RK3 integrator.
  isimulator->StepTo(GetOutput(simulator.get()));
  const double p_at_s_at_p_end_using_rk3 = GetOutput(isimulator.get());

  // Approximates the p-to-s function at the parameter interval end using GSL's
  // RK8 integrator.
  const double s_at_p_end_using_rk8 = IntegrateArcLengthRK8(
      rc.get(), kPEnd, kR0, kH0, kInitialStep, kAccuracy);

  // Uses the previous result to approximate the s-to-p function at the same
  // spot using GSL's RK8 integrator.
  const double p_at_s_at_p_end_using_rk8 = IntegrateParameterRK8(
      rc.get(), s_at_p_end_using_rk8, kR0, kH0, kInitialStep, kAccuracy);

  std::cout << "Drake's RK3 approximate for s(p = " << kPEnd << ") = "
            << s_at_p_end_using_rk3 << std::endl;
  std::cout << "GSL's RK8 approximate for s(p = " << kPEnd << ") = "
            << s_at_p_end_using_rk8 << std::endl;
  std::cout << "Drake's RK3 approximate for p(s = "
            << s_at_p_end_using_rk3 << ") = "
            << p_at_s_at_p_end_using_rk3 << std::endl;
  std::cout << "GSL's RK8 approximate for p(s = "
            << s_at_p_end_using_rk8 << ") = "
            << p_at_s_at_p_end_using_rk8 << std::endl;

  // Computes a manual integration.
  const int kSteps = 10000;
  const double kStep{(kPEnd - kP0) / static_cast<double>(kSteps)};
  const double s_at_p_end_manual =
      Integrate(kP0, kStep, kSteps, rc.get(), kR0, kH0);

  // Present integration values.
  std::cout << "Manual arc length integration s(p = " << kPEnd << ") = "
            << s_at_p_end_manual << std::endl;
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

  const std::string kMeshPath = "/tmp";
  drake::maliput::multilane::CreateMesh(kMeshPath, "FLAT_LINE",
                                        GeometryType::FLAT_LINE);
  drake::maliput::multilane::CreateMesh(kMeshPath, "LINEAR_ELEVATION_LINE",
                                        GeometryType::LINEAR_ELEVATION_LINE);
  drake::maliput::multilane::CreateMesh(kMeshPath, "QUADRATIC_ELEVATION_LINE",
                                        GeometryType::QUADRATIC_ELEVATION_LINE);
  drake::maliput::multilane::CreateMesh(kMeshPath, "CUBIC_ELEVATION_LINE",
                                        GeometryType::CUBIC_ELEVATION_LINE);
  drake::maliput::multilane::CreateMesh(
      kMeshPath, "CUBIC_ELEV_LINEAR_SUPER_LINE",
      GeometryType::CUBIC_ELEV_LINEAR_SUPER_LINE);
  return 0;
}
