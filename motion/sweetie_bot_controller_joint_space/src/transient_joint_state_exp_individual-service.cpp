#include <sweetie_bot_controller_joint_space/transient_joint_state.hpp>

#include <string>
#include <vector>
#include <algorithm>

#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

namespace sweetie_bot {
namespace motion {
namespace filter {

class TransientJointStateExpIndividual : 
	public TransientJointStateInterface, public RTT::Service 
{
	protected:
		/**
		 * @brief Discrete 2nd order filter. State-space representation.
		 * x(k+1) = Ax(k) + Bu(k)
		 **/
		class DiscreteFilter2 {
			public:
				double a11, a12, a21, a22;
			   	double b11, b12, b21, b22;

				DiscreteFilter2(double transient_time, double period);
				DiscreteFilter2(const DiscreteFilter2& filter) = default;
		};

	protected:
		// SERVICE INTERFACE
		// properties
		double transient_time;
		std::vector<std::string> joints_override;
		std::vector<double> transient_time_override;

		// STATE
		std::vector<DiscreteFilter2> filters;	
		bool is_configured;
	public:
		TransientJointStateExpIndividual(RTT::TaskContext * owner);

		bool reset(const JointState& state0, double period);
		bool update(const JointState& state, const JointState & ref, JointState& new_state);

};

TransientJointStateExpIndividual::DiscreteFilter2::DiscreteFilter2(double transient_time, double T) 
{
	if (transient_time <= 0.0 || T <= 0.0) {
		throw std::invalid_argument("TransientJointStateExp: transient_time and period must be positive.");
	}
	// pole placment
	double tau = transient_time/4.6; 
	double l1 = -1/tau; 
	double l2 = -1.2/tau;

	// matrix exponent calculations
	a11 = (l2*exp(l1*T)-l1*exp(l2*T))/(l2-l1); 
	a12 = (-exp(l1*T)+exp(l2*T))/(l2-l1); 
	a21 = (l1*l2*(exp(l1*T)-exp(l2*T)))/(l2-l1); 
	a22 = (-l1*exp(l1*T)+l2*exp(l2*T))/(l2-l1);

	// input matrix for position error
	b11 = 1/(l1*l2)*((l1+l2)*a12 - a22 + 1)*(l1*l2);
	b21 = a12*l1*l2;

	// input matrix for speed error
	b12 = 1/(l1*l2)*((l1+l2)*a12 - a22 + 1)*(-l1-l2);
	b22 = a12*(-l1-l2);
}

TransientJointStateExpIndividual::TransientJointStateExpIndividual(RTT::TaskContext * owner) :
	Service("transient_joint_state_exp_individual", owner)
{
	this->doc("Exponential flter with given transient time for JointState trajectory.");

	this->addProperty("transient_time", transient_time)
		.doc("Default transient time in seconds of exponential filter.");

	this->addProperty("joints_override", joints_override)
		.doc("List of joints with individual transient time settings.");

	this->addProperty("transient_time_override", transient_time_override)
		.doc("Individual transient time for joints in joints_override list. Size of lists must be same.");

	is_configured = false;
}

bool TransientJointStateExpIndividual::reset(const JointState& state, double T) 
{
	// load individual filters
	if (joints_override.size() != transient_time_override.size()) {
		is_configured = false;
		return is_configured;
		//throw std::invalid_argument("TransientJointStateExpIndividual: sizes of joints_override and transient_time_override sre not same");
	}
	// init filters
	DiscreteFilter2 default_filter = DiscreteFilter2(transient_time, T);
	filters.clear();
	for(auto joint = state.name.begin(); joint != state.name.end(); joint++) {
		auto found = std::find(joints_override.begin(), joints_override.end(), *joint);
		if (found != joints_override.end()) {
			filters.push_back( DiscreteFilter2(transient_time_override[found - joints_override.begin()], T) );
		}
		else {
			filters.push_back( default_filter );
		}
	}
	is_configured = true;
	return is_configured;
}

bool TransientJointStateExpIndividual::update(const JointState& state, const JointState& ref, JointState& new_state) 
{
	if (!is_configured) return false;

	int sz = filters.size();
	if (state.position.size() == sz && state.velocity.size() == sz && ref.position.size() == sz && ref.velocity.size() == sz) {
		for(int i = 0; i < sz; i++) {
			double pos = filters[i].a11 * state.position[i] + filters[i].a12 * state.velocity[i] + filters[i].b11 * ref.position[i] + filters[i].b12 * ref.velocity[i];
			double vel = filters[i].a21 * state.position[i] + filters[i].a22 * state.velocity[i] + filters[i].b21 * ref.position[i] + filters[i].b22 * ref.velocity[i];
			new_state.position[i] = pos;
			new_state.velocity[i] = vel;
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
ORO_SERVICE_NAMED_PLUGIN(sweetie_bot::motion::filter::TransientJointStateExpIndividual, "transient_joint_state_exp_individual")
