#ifndef PARTICLE_H
#define PARTICLE_H

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace particle {

template <typename T>
class Particle : public systems::LeafSystem<T> {
	public:
		// Constructor for the Particle system.
		Particle();

		double GetAcceleration() const {
			return a;
		}
		void SetAcceleration(const double _a) {
			a = _a;
		}

		void DoCalcOutput(const systems::Context<T>& context,
            systems::SystemOutput<T>* output) const override;

		void DoCalcTimeDerivatives(
			const systems::Context<T>& context,
			systems::ContinuousState<T>* derivatives) const override;

		void SetDefaultState(const systems::Context<T>& context,
			systems::State<T>* state) const override;

	private:
		double lastTime{0.0};
		double a{1.0};
};

}
}

#endif