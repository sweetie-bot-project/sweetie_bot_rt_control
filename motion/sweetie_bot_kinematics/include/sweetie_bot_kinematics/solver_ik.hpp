#ifndef  SOLVER_IK_HPP
#define  SOLVER_IK_HPP

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include <sweetie_bot_logger/logger.hpp>

namespace sweetie_bot {
namespace motion {

/**
 * @brief Inverse kinematic problem solver interface.
 *
 * This class generalizes IK solver.
 **/
class SolverIKInterface {
	public:
		enum ReturnStatus {
			PRECISE_SOLUTION = 0, /**< IK solution complies with tolerance and joint limits. */
			APPROXIMATE_SOLUTION = 1, /**< IK solution does not comply with tolerance. */
			NO_SOLUTION = -1, /**< No solution is found or joint limits are violated. */
		};

	public:
		/**
		 * @brief Solve IK problem.
		 *
		 * @param b_T_e Desired end effector position in base link frame.
		 * @param jnt Curent joint space pose. Some solvers may ignore it.
		 * @param result IK solution is returned via this param.
		 * @param log Logger.
		 * @return IK solution result code.
		 **/
		virtual ReturnStatus solveIK(const KDL::Frame& b_T_e, const KDL::JntArray& jnt, KDL::JntArray& result, sweetie_bot::logger::Logger& log) = 0;
};

/**
 * @brief Solver Factory interface.
 *
 * IK solver factories provides solvers for kinemaitic chains. 
 * Solver factory can be loaded to in OROCOS component as Servoces 
 * and then be used to get solver for specific kinematic chain.
 *
 **/
class SolverIKFactoryInterface
{
	public:
		/**
		 * @brief Get solver for kinematic chain.
		 *
		 * @param name Kinematic chain name.
		 * @param chain Kinematic chain description.
		 * @param lower Lower joints limits
		 * @param upper Upper joints limits
		 * @param tolerance New tolerance value.
		 * @param log Logger.
		 * @return Poiner to IK solver.
		 **/
		virtual std::unique_ptr<SolverIKInterface> getSolver(const std::string& name, const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance, sweetie_bot::logger::Logger& log) const = 0;

};

}
}

#endif  /*SOLVER_IK_HPP*/
