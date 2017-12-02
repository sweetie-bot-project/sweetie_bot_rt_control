#include <sweetie_bot_controller_joint_space/filter_joint_state.hpp>

#include <stdexcept>

#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

namespace sweetie_bot {
namespace motion {
namespace filter {

class TransientJointStateExp : 
	public FilterJointStateInterface, public RTT::Service 
{
	protected:
		// SERVICE INTERFACE
		// properties
		double transient_time;

		// STATE
		double a11, a12, a21, a22;
		bool is_configured;

		JointState residual;
		bool first_update;
	public:
		TransientJointStateExp(RTT::TaskContext * owner);

		bool reset(const JointState& state0, double period);
		bool update(const JointState& state, const JointState & ref, JointState& new_state);
};

TransientJointStateExp::TransientJointStateExp(RTT::TaskContext * owner) :
	Service("transient_joint_state_exp", owner)
{
	this->doc("Exponential transient filter with critical damping for JointState trajectory. It reacts only on inial state.");

	this->addProperty("transient_time", transient_time)
		.doc("Transient time in seconds of exponential filter.");

	is_configured = false;
}

bool TransientJointStateExp::reset(const JointState& state, double T) 
{
	// transfer function synthesis for given transient time
	// poles are equal and real
	if (transient_time <= 0.0 || T <= 0.0) {
		throw std::invalid_argument("TransientJointStateExp: transient_time and period must be positive.");
	}
	double tau = transient_time/4.6; 
	double l1 = -1/tau;
	double l2 = -1.2/tau;

	// calculate evolution matrix
	a11 = (l2*exp(l1*T)-l1*exp(l2*T))/(l2-l1); 
	a12 = (-exp(l1*T)+exp(l2*T))/(l2-l1); 
	a21 = (l1*l2*(exp(l1*T)-exp(l2*T)))/(l2-l1); 
	a22 = (-l1*exp(l1*T)+l2*exp(l2*T))/(l2-l1);

	// store initial state in residual
	first_update = true;
	residual = state;
	if (residual.velocity.size() == 0) residual.velocity.assign(residual.position.size(), 0.0);
	if (residual.velocity.size() != residual.position.size()) {
		throw std::invalid_argument("TransientJointStateExp: incorrect initial state.");
	}

	is_configured = true;
	return is_configured;
}

bool TransientJointStateExp::update(const JointState& state, const JointState& ref, JointState& new_state) 
{
	if (!is_configured) return false;

	int sz = residual.position.size();
	if (residual.velocity.size() == sz && ref.position.size() == sz && ref.velocity.size() == sz) {
		if (first_update) {
			// calculate residual: after reset residual contains initial state
			for(int i = 0; i < sz; i++) {
				residual.position[i] -= ref.position[i];
				residual.velocity[i] -= ref.velocity[i];
			}
			first_update = false;
		}
		// normal operational: 
		for(int i = 0; i < sz; i++) {
			// exponentially fade residual
			double pos = a11*residual.position[i] + a12*residual.velocity[i];
			double vel = a21*residual.position[i] + a22*residual.velocity[i];
			residual.position[i] = pos;
			residual.velocity[i] = vel;
			// add it to reference position
			new_state.position[i] = ref.position[i] + pos;
			new_state.velocity[i] = ref.velocity[i] + vel;
		}
		return true;
	}
	return false;
}

} // namespace filter
} // namespace sweetie_bot 
} // namespace motion 

/* For consistency reasons, it's better to name the
 * service the same as in the class above.
 */
ORO_SERVICE_NAMED_PLUGIN(sweetie_bot::motion::filter::TransientJointStateExp, "transient_joint_state_exp")
