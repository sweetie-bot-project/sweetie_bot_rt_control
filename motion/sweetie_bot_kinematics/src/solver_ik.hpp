#ifndef  SOLVER_IK_HPP
#define  SOLVER_IK_HPP

#include <sweetie_bot_logger/logger.hpp>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

namespace sweetie_bot {
namespace motion {

class SolverIK {
	protected:
		double tolerance_pos_;
	public:
		SolverIK() : tolerance_pos_(0.001) {}
		void setTolerance(double tol) { tolerance_pos_ = tol; }

		virtual bool solveIK(const KDL::Chain& chain, const KDL::Frame& b_T_e, const KDL::JntArray& lower, const KDL::JntArray& upper, KDL::JntArray& jnt, sweetie_bot::logger::Logger& log) const = 0;
};

class SolverIKFactory 
{
	public:
		using SolverIKCreatePtr = std::unique_ptr<SolverIK>(*)(const KDL::Chain& chain, sweetie_bot::logger::Logger& log);

	private:
		static std::vector<SolverIKCreatePtr> solver_list;

	public:
		static bool Register(SolverIKCreatePtr create_method);
		static std::unique_ptr<SolverIK> GetSolver(const KDL::Chain& chain, sweetie_bot::logger::Logger& log);

};

}
}

#endif  /*SOLVER_IK_HPP*/
