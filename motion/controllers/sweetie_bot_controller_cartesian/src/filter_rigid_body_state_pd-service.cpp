#include <sweetie_bot_controller_cartesian/filter_rigid_body_state.hpp>

#include <stdexcept>

#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

#include <sweetie_bot_orocos_misc/message_checks.hpp>
#include <sweetie_bot_orocos_misc/math.hpp>

namespace sweetie_bot {
namespace motion {
namespace filter {

class FilterRigidBodyStatePD : 
	public FilterRigidBodyStateInterface, public RTT::Service 
{
	protected:
		// SERVICE INTERFACE
		// properties
		double Kp;
		double Kv;

		// STATE
		bool is_configured;
		double period;
	public:
		FilterRigidBodyStatePD(RTT::TaskContext * owner);

		bool reset(const RigidBodyState& state0, double period);
		bool update(const RigidBodyState& state, const RigidBodyState & ref, RigidBodyState& new_state);
};

FilterRigidBodyStatePD::FilterRigidBodyStatePD(RTT::TaskContext * owner) :
	Service("filter_rigid_body_state_pd", owner)
{
	this->doc("PD requlator for rigid body pose. It uses equal coefficients for rotation and translation. Reference speed is ignored.");

	this->addProperty("Kp", Kp)
		.doc("Feedback  coefficient for position error. (PD regulator parameter).")
		.set(9);
	this->addProperty("Kd", Kv)
		.doc("Feedback  coefficient for velocity. (PD regulator parameter).")
		.set(6);

	is_configured = false;
}

bool FilterRigidBodyStatePD::reset(const RigidBodyState& state, double T) 
{
	// transfer function synthesis for given transient time
	// poles are equal and real
	if (Kp <= 0.0 || Kv <= 0.0 || T <= 0.0) {
		throw std::invalid_argument("FilterRigidBodyStatePD: regulator parameters and period must be positive.");
	}

	// set period
	period = T;

	is_configured = true;
	return is_configured;
}

bool FilterRigidBodyStatePD::update(const RigidBodyState& state, const RigidBodyState& ref, RigidBodyState& new_state) 
{
	if (!is_configured) return false;

	int sz = state.frame.size();
	if (ref.frame.size() != sz || new_state.frame.size() != sz || state.twist.size() != sz || new_state.twist.size() != sz) return false;

	for(int i = 0; i < state.frame.size(); i++) {
		// calculate speed of the origin of state in world frame
		KDL::Vector dp = state.twist[i].vel + state.twist[i].rot*state.frame[i].p; 
		// Re = inv(Rr)*R, logm(Re) --- orientation error
		// calculate matrix log: inv(Rr)*R = exp(S(Re_axis*Re_angle))
		KDL::Vector Q = rotationMatrixLogarithm(state.frame[i].M.Inverse()*ref.frame[i].M);

		// now calculate acceleration: PD regulators for pe = pr - p and Re 
		KDL::Twist accel;
		accel.rot = (-Kv)*state.twist[i].rot + (Kp)*(ref.frame[i].M*Q);
		accel.vel = (-Kv)*dp + Kp*(ref.frame[i].p - state.frame[i].p) - accel.rot*state.frame[i].p - state.twist[i].rot*dp;
		// integrate base_link pose
		// TODO Is there better way to integrate pose?
		new_state.twist[i] = state.twist[i] + (0.5*period)*accel;
		KDL::Rotation Rt = KDL::Rot(new_state.twist[i].rot*period);
		new_state.frame[i].M = Rt*state.frame[i].M;
		new_state.frame[i].p = Rt*state.frame[i].p + new_state.twist[i].vel*period;
		new_state.twist[i] += (0.5*period)*accel;
	}
	return true;
}

} // namespace filter
} // namespace sweetie_bot 
} // namespace motion 

/* For consistency reasons, it's better to name the
 * service the same as in the class above.
 */
ORO_SERVICE_NAMED_PLUGIN(sweetie_bot::motion::filter::FilterRigidBodyStatePD, "filter_rigid_body_state_pd")
