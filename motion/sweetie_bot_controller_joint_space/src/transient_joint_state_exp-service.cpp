#include <sweetie_bot_controller_joint_space/transient_joint_state.hpp>

#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

namespace sweetie_bot {
namespace motion {
namespace filter {

class TransientJointStateExp : 
	public TransientJointStateInterface, public RTT::Service 
{
	protected:
		// SERVICE INTERFACE
		// properties
		double transient_time;

		// STATE
		double a11, a12, a21, a22, b11, b12, b21, b22;
		bool is_configured;
	public:
		TransientJointStateExp(RTT::TaskContext * owner);

		bool reset(const JointState& state0, double period);
		bool update(JointState& state, const JointState & ref);
};

TransientJointStateExp::TransientJointStateExp(RTT::TaskContext * owner) :
	Service("transient_joint_state_exp", owner)
{
	this->doc("Exponential flter with critical damping for JointState trajectory.");

	this->addProperty("transient_time", transient_time)
		.doc("Transient time in seconds of exponential filter.");

	is_configured = false;
}

bool TransientJointStateExp::reset(const JointState& state, double T) 
{
	// transfer function synthesis for given transient time
	// poles are equal and real
	// TODO MATRIX EXPONENT
	double tau = transient_time/4.6; 
	double l1 = -1/tau;
	double l2 = -1.2/tau;

	a11 = (l2*exp(l1*T)-l1*exp(l2*T))/(l2-l1); 
	a12 = (-exp(l1*T)+exp(l2*T))/(l2-l1); 
	a21 = (l1*l2*(exp(l1*T)-exp(l2*T)))/(l2-l1); 
	a22 = (-l1*exp(l1*T)+l2*exp(l2*T))/(l2-l1);

	b11 = 1/(l1*l2)*((l1+l2)*a12 - a22 + 1)*(l1*l2);
	b21 = a12*l1*l2;

	b12 = 1/(l1*l2)*((l1+l2)*a12 - a22 + 1)*(-l1-l2);
	b22 = a12*(-l1-l2);

	//std::cout << a11 << " " << a12 << " " << b11 << " " << b12 << std::endl;
	//std::cout << a21 << " " << a22 << " " << b21 << " " << b22 << std::endl;

	is_configured = true;
	return is_configured;
}

bool TransientJointStateExp::update(JointState& state, const JointState& ref) 
{
	if (!is_configured) return false;

	int sz = state.position.size();
	if (state.velocity.size() == sz && ref.position.size() == sz && ref.velocity.size() == sz) {
		for(int i = 0; i < sz; i++) {
			double pos = a11*state.position[i] + a12*state.velocity[i] + b11*ref.position[i] + b12*ref.velocity[i];
			double vel = a21*state.position[i] + a22*state.velocity[i] + b21*ref.position[i] + b22*ref.velocity[i];
			//std::cout << state.position[i] << " " << ref.position[i] << " " << pos << std::endl;
			//std::cout << state.velocity[i] << " " << ref.velocity[i] << " " << vel << std::endl;
			state.position[i] = pos;
			state.velocity[i] = vel;
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
