#ifndef PARTICLE_INL_HH
#define PARTICLE_INL_HH

#include "drake/examples/particle_system/src/particle.hh"

namespace drake {
  namespace particles {

    template <typename T>
    Particle<T>::Particle() {
      this->DeclareInputPort(systems::kVectorValued, 1);
      this->DeclareContinuousState(1, 1, 0);
      this->DeclareOutputPort(systems::kVectorValued, 2);
    }
    
    template <typename T>
    void Particle<T>::SetInitialConditions(const T& position, const T& velocity) {
      ic_ << position, velocity;
    }
    
    template <typename T>
    void Particle<T>::DoCalcOutput(const systems::Context<T>& context,
				   systems::SystemOutput<T>* output) const {
      // Get current state from context.
      const systems::VectorBase<T>& state_vec =
	context.get_continuous_state_vector();
      // Obtain the structure we need to write into.
      systems::BasicVector<T>* const output_vec =
	output->GetMutableVectorData(0);
      
      output_vec->set_value(state_vec.CopyToVector());
    }

    template <typename T>
    void Particle<T>::DoCalcTimeDerivatives(const systems::Context<T>& context,
					    systems::ContinuousState<T>* derivatives) const {
      // Get current state from context.
      const systems::VectorBase<T>& cstate_vec =
	context.get_continuous_state_vector();
      // Obtain the structure we need to write into.
      systems::VectorBase<T>* const deriv_vec =
	derivatives->get_mutable_vector();
      // Get current acceleration input value
      const systems::BasicVector<T>* accel = this->EvalVectorInput(context, 0);
      // Set the derivatives. The first one is the speed
      // and the other one is the acceleration
      deriv_vec->SetAtIndex(0, cstate_vec.GetAtIndex(1));
      deriv_vec->SetAtIndex(1, accel->GetAtIndex(0));
    }

    template <typename T>
    void Particle<T>::SetDefaultState(const systems::Context<T>& context,
				      systems::State<T>* state) const {
      systems::ContinuousState<T>* cstate = state->get_mutable_continuous_state();
      cstate->SetFromVector(ic_);
    }
    
  }  // namespace particles
}  // namespace drake

#endif  // PARTICLE_INL_HH
