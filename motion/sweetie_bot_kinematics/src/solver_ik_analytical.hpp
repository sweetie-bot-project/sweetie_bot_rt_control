#ifndef  SOLVER_IK_ANALYTICAL_HPP
#define  SOLVER_IK_ANALYTICAL_HPP

#include <sweetie_bot_kinematics/solver_ik.hpp>

#include <sweetie_bot_logger/logger.hpp>

namespace sweetie_bot {
namespace motion {

class SolverIKAnalytical :
	public SolverIKInterface
{
	protected:
		KDL::Chain chain_;
		KDL::JntArray jnt_lower_bounds_;
		KDL::JntArray jnt_upper_bounds_;
		KDL::Twist tolerance_;

	protected:
		virtual bool solveIK_impl(const KDL::Frame& b_T_e, KDL::JntArray& jnt, sweetie_bot::logger::Logger& log) const = 0;
		
	public:
		SolverIKAnalytical(const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance) : 
			chain_(chain), jnt_lower_bounds_(lower), jnt_upper_bounds_(upper), tolerance_(tolerance)
		{}

		ReturnStatus solveIK(const KDL::Frame& b_T_e, const KDL::JntArray& jnt, KDL::JntArray& result, sweetie_bot::logger::Logger& log) override;
};

class SolverIKFactoryAnalytical :
	public SolverIKFactoryInterface
{
	public:
		using SolverIKCreatePtr = std::unique_ptr<SolverIKAnalytical>(*)(const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance, sweetie_bot::logger::Logger& log);

	private:
		static std::vector<SolverIKCreatePtr> solver_list;

	public:
		static bool Register(SolverIKCreatePtr create_method);
		std::unique_ptr<SolverIKInterface> getSolver(const std::string& name, const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance, sweetie_bot::logger::Logger& log) const override;

};

}
}

#endif  /*SOLVER_IK_ANALYTICAL_HPP*/
