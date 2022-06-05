#include "solver_ik_analytical.hpp"

namespace sweetie_bot {
namespace motion {

SolverIKInterface::ReturnStatus SolverIKAnalytical::solveIK(const KDL::Frame& b_T_e, const KDL::JntArray& jnt, KDL::JntArray& result, sweetie_bot::logger::Logger& log)
{
	// invoke solver
	bool ret = solveIK_impl(b_T_e, result, log);
	// check if solution exists
	if (!ret) {
		return NO_SOLUTION;
	}
	// Limits check
	if ( (result.data.array() > jnt_upper_bounds_.data.array()).any()  || 
		 (result.data.array() < jnt_lower_bounds_.data.array()).any() ) 
	{
		log(DEBUG) << "IK Analytical failed: joint limits: solution  " << result.data.transpose() << ", lower " << jnt_lower_bounds_.data.transpose() << ", upper = " << jnt_upper_bounds_.data.transpose() << RTT::endlog();
		return NO_SOLUTION;
	}
	// TODO: tolerance check
	return PRECISE_SOLUTION;
}

std::vector<SolverIKFactoryAnalytical::SolverIKCreatePtr> SolverIKFactoryAnalytical::solver_list;

bool SolverIKFactoryAnalytical::Register(SolverIKFactoryAnalytical::SolverIKCreatePtr create_method) {
	solver_list.push_back(create_method);
	return true;
}

std::unique_ptr<SolverIKInterface> SolverIKFactoryAnalytical::getSolver(const std::string& name, const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance, sweetie_bot::logger::Logger& log) const
{
	std::unique_ptr<SolverIKAnalytical> solver;
	for (auto createMethod : solver_list) {
		solver = createMethod(chain, lower, upper, tolerance, log);
		if (solver != nullptr) break;
	}
	return solver;
}

}
}

