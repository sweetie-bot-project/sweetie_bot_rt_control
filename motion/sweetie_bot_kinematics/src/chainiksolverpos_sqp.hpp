#ifndef  CHAINIKSOLVERPOS_SQP_HPP
#define  CHAINIKSOLVERPOS_SQP_HPP

#include <vector>
#include <nlopt.h>

#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace KDL {

class ChainIkSolverPos_sqp : public KDL::ChainIkSolverPos 
{
	public:
		enum { E_INVALID_LIMITS = -50,  E_NLOPT_FAILED = -100, };

	public:
		ChainIkSolverPos_sqp(const KDL::Chain& chain, const KDL::JntArray& q_min, const KDL::JntArray& q_max, const KDL::JntArray& q_opt, const KDL::Twist& tol, double maxtime = 0.005, double eps_pos = 1e-3);
		~ChainIkSolverPos_sqp() override;

		void updateInternalDataStructures() override;
		int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out) override;

		const char* strError(const int error) const override;

		int getNLOptResult() const {
			return nlopt_result_;
		}
		const char * strNLOptResult(const int result) const;

		int setJointLimits(const KDL::JntArray& q_min, const KDL::JntArray& q_max);
		int setJointOpt(const KDL::JntArray& q_opt);
		void setWeightRot(double value) {
			weight_rot_ = value;
		}
		void setWeightQOpt(double value) {
			weight_q_ = value;
		}
		void setMaxtime(double value) {
			maxtime_ = value;
		}
		void setTolerance(const KDL::Twist& value) {
			tolerance_ = value;
		}
		void setEpsPos(double value) {
			eps_pos_ = value;
		}

		double cartSumSquaredError(const double * q_data, double * grad_data);

	private:
		// KDL chain
		const KDL::Chain& chain_;
		// KDL solvers
		KDL::ChainFkSolverPos_recursive fk_solver_;
		KDL::ChainJntToJacSolver jac_solver_;
		// problem properties
		KDL::Frame b_Tref_ee_;
		KDL::JntArray q_min_;
		KDL::JntArray q_max_;
		KDL::JntArray q_opt_;
		KDL::Twist tolerance_;
		double weight_rot_;
		double weight_q_;
		// limits
		double maxtime_;
		double eps_pos_;
		// nlp solver
		nlopt_opt nlopt_solver_;
		int nlopt_result_;
		// buffers
		KDL::Jacobian jac_;
		std::vector<double> q_;
		std::vector<double> lower_bounds_;
		std::vector<double> upper_bounds_;
		KDL::JntArray q_jnt_;
		KDL::JntArray best_q_jnt_;
		double best_q_fval_;
		bool best_q_in_tolerance_; 
};

} /* namespace KDL */

#endif  /*CHAINIKSOLVERPOS_SQP_HPP*/
