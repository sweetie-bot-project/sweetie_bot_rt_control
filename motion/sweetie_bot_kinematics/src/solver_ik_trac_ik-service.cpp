#include <rtt/Logger.hpp>
#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

#include <trac_ik/trac_ik.hpp>

#include <sweetie_bot_kinematics/solver_ik.hpp>

namespace sweetie_bot {
namespace motion {

/**
 * @brief Wrapper for TRAC_IK solver
 **/
class SolverIKTracIK :
	public SolverIKInterface
{
	protected:
		KDL::Chain chain_;
		KDL::Twist tolerance_;
		TRAC_IK::TRAC_IK solver_;

	public:
		SolverIKTracIK(const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance, double timeout, double eps_pos) :
			chain_(chain),
			solver_(chain_, lower, upper, timeout, eps_pos, TRAC_IK::Speed),
			tolerance_(tolerance)
		{}

		ReturnStatus solveIK(const KDL::Frame& b_T_e, const KDL::JntArray& jnt, KDL::JntArray& result, sweetie_bot::logger::Logger& log) override;
};

SolverIKInterface::ReturnStatus SolverIKTracIK::solveIK(const KDL::Frame& b_T_e, const KDL::JntArray& jnt, KDL::JntArray& result, sweetie_bot::logger::Logger& log)
{
	int ret = solver_.CartToJnt(jnt, b_T_e, result, tolerance_);
	if (ret < 0) {
		//log(DEBUG) << "TRAC_IK failed: chain " << name << ", error code: " << ret << endlog();
		log(DEBUG) << "TRAC_IK failed: error code: " << ret << RTT::endlog();
		return NO_SOLUTION;
	}
	else {
		// fix trak_ik bug: map angles to [-pi, pi] interval. 
		// TODO: bug report.
		std::transform( result.data.data(), result.data.data() + result.data.size(),  result.data.data(), 
				[](double angle) {
					angle = std::fmod(angle + M_PI, 2*M_PI);
					angle = (angle >= 0.0) ? angle : angle + 2*M_PI;
					return angle - M_PI;
				}
			);
		return PRECISE_SOLUTION;
	}
}

/**
 * @brief TRAC_IK IK solver factory.
 *
 **/
class SolverIKFactoryTracIK : 
	public SolverIKFactoryInterface, public RTT::Service 
{
	protected:
		// SERVICE INTERFACE
		// properties
		std::vector<std::string> chain_names_;
		double eps_pos_;
		double timeout_;

	public:
		SolverIKFactoryTracIK(RTT::TaskContext * owner);

		std::unique_ptr<SolverIKInterface> getSolver(const std::string& name, const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance, sweetie_bot::logger::Logger& log) const override;
};

SolverIKFactoryTracIK::SolverIKFactoryTracIK(RTT::TaskContext * owner) :
	Service("solver_ik_factory_trac_ik", owner)
{
	// Service description.
	this->doc("Provides TRAC_IK inverse kinematic solvers.");
	// Properties
	this->addProperty( "kinematic_chains", chain_names_ )
		.doc( "List of kinematic chains for which poses are calculated. Other chains are ignored.");
	this->addProperty( "eps_pos", eps_pos_ )
		.doc( "Singular values less then eps is assumed to be zero. TracIk solver parameter.")
		.set(1e-4);
	this->addProperty( "timeout_pos", timeout_ )
		.doc( "TracIK solver timeout (s).")
		.set(0.003);
}

std::unique_ptr<SolverIKInterface> 
SolverIKFactoryTracIK::getSolver(const std::string& name, const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance, sweetie_bot::logger::Logger& log) const
{
	// ignore kinematic chain if it is not inside list
	if (std::find(chain_names_.begin(), chain_names_.end(), name) == chain_names_.end()) {
		return nullptr;
	}
	// create solver
	return std::unique_ptr<SolverIKInterface>(new SolverIKTracIK(chain, lower, upper, tolerance, timeout_, eps_pos_));
}


} // namespace sweetie_bot 
} // namespace motion 

/* For consistency reasons, it's better to name the
 * service the same as in the class above.
 */
ORO_SERVICE_NAMED_PLUGIN(sweetie_bot::motion::SolverIKFactoryTracIK, "solver_ik_factory_trac_ik")
