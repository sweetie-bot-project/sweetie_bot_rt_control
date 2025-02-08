#include <rtt/Logger.hpp>
#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

#include <sweetie_bot_kinematics/solver_ik.hpp>

#include "chainiksolverpos_sqp.hpp"

namespace sweetie_bot {
namespace motion {

/**
 * @brief Wrapper for TRAC_IK solver
 **/
class SolverIKKDL_sqp :
	public SolverIKInterface
{
	protected:
		KDL::Chain chain_;
		KDL::ChainIkSolverPos_sqp solver_;

	public:
		SolverIKKDL_sqp(const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::JntArray& q_opt, const KDL::Twist& tol, double timeout, double eps) :
			chain_(chain),
			solver_(chain_, lower, upper, q_opt, tol, timeout, eps)
		{}

		ReturnStatus solveIK(const KDL::Frame& b_T_e, const KDL::JntArray& jnt, KDL::JntArray& result, sweetie_bot::logger::Logger& log) override;
		KDL::ChainIkSolverPos_sqp& solver() { return solver_; }
};

SolverIKInterface::ReturnStatus SolverIKKDL_sqp::solveIK(const KDL::Frame& b_T_e, const KDL::JntArray& jnt, KDL::JntArray& result, sweetie_bot::logger::Logger& log)
{
	int ret = solver_.CartToJnt(jnt, b_T_e, result);
	if (ret < 0) {
		if (log(DEBUG)) {
			int nlopt_result = solver_.getNLOptResult();
			log() << "KDL IK SQP solver failed: error code " << ret << " (" << solver_.strError(ret) << "), nlpopt_result " << nlopt_result << " (" << solver_.strNLOptResult(nlopt_result) << ") "  << RTT::endlog();
		}
		return NO_SOLUTION;
	}
	else {
		// fmap angles to [-pi, pi] interval
		std::transform( result.data.data(), result.data.data() + result.data.size(), result.data.data(), 
				[](double angle) {
					angle = std::fmod(angle + M_PI, 2*M_PI);
					angle = (angle >= 0.0) ? angle : angle + 2*M_PI;
					return angle - M_PI;
				}
			);
		// result
		if (ret == KDL::ChainIkSolverPos::E_NOERROR) return PRECISE_SOLUTION;
		else return APPROXIMATE_SOLUTION;
	}
}

/**
 * @brief KDL IK solver factory.
 *
 **/
class SolverIKFactoryKDL_sqp : 
	public SolverIKFactoryInterface, public RTT::Service 
{
	protected:
		// SERVICE INTERFACE
		// properties
		std::vector<std::string> chain_names_;
		double timeout_;
		double eps_pos_;
		double weight_rot_;
		double weight_q_opt_;

	public:
		SolverIKFactoryKDL_sqp(RTT::TaskContext * owner);

		std::unique_ptr<SolverIKInterface> getSolver(const std::string& name, const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance, sweetie_bot::logger::Logger& log) const override;
};

SolverIKFactoryKDL_sqp::SolverIKFactoryKDL_sqp(RTT::TaskContext * owner) :
	Service("solver_ik_factory_kdl_sqp", owner)
{
	// Service description.
	this->doc("Provides KDL SQP inverse kinematic solvers.");
	// Properties
	this->addProperty( "kinematic_chains", chain_names_ )
		.doc( "List of kinematic chains for which poses are calculated. Other chains are ignored.");
	this->addProperty( "eps_pos", eps_pos_ )
		.doc( "Tolerance (distance, m).")
		.set(1e-4);
	this->addProperty( "timeout", timeout_ )
		.doc( "Solver timeout (s).")
		.set(0.003);
	this->addProperty( "weight_rot", weight_rot_ )
		.doc( "Weight of rotation in squared sum error.")
		.set(0.1);
	this->addProperty( "weight_q_opt", weight_q_opt_ )
		.doc( "Weight of divergence from optimal pose in squared sum error.")
		.set(0.001);
}

std::unique_ptr<SolverIKInterface> 
SolverIKFactoryKDL_sqp::getSolver(const std::string& name, const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tol, sweetie_bot::logger::Logger& log) const
{
	// ignore kinematic chain if it is not inside list
	if (std::find(chain_names_.begin(), chain_names_.end(), name) == chain_names_.end()) {
		return nullptr;
	}
	// get optimal pose
	KDL::JntArray q_opt;
	RTT::Property< std::vector<double> > q_opt_prop = this->getProperty(name + "_q_opt");
	if (q_opt_prop.ready()) {
		q_opt.data = Eigen::Map<Eigen::VectorXd>(&q_opt_prop.value().front(), q_opt_prop.value().size());
	}
	else {
		q_opt.data = 0.5*(lower.data + upper.data);
	}
	// create solver
	std::unique_ptr<SolverIKKDL_sqp> solver;
	try {
		solver.reset(new SolverIKKDL_sqp(chain, lower, upper, q_opt, tol, timeout_, eps_pos_));
	} 
	catch (std::exception& e) {
		log(ERROR) << "Unable to create KDL SQP solver: " << e.what() << RTT::endlog();
		return nullptr;
	}
	solver->solver().setWeightQOpt(weight_q_opt_);
	solver->solver().setWeightRot(weight_rot_);
	// return solver
	return solver;
}


} // namespace sweetie_bot 
} // namespace motion 

/* For consistency reasons, it's better to name the
 * service the same as in the class above.
 */
ORO_SERVICE_NAMED_PLUGIN(sweetie_bot::motion::SolverIKFactoryKDL_sqp, "solver_ik_factory_kdl_sqp")
