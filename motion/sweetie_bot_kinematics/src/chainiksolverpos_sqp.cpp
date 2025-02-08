#include "chainiksolverpos_sqp.hpp"

#include <cmath>
#include <limits>
#include <Eigen/Dense>

namespace KDL {

static double cartSumSquaredError_adapter(unsigned n, const double* x, double* grad, void* f_data) 
{
	ChainIkSolverPos_sqp * sqp_ik_solver = static_cast<ChainIkSolverPos_sqp *>(f_data);
	return sqp_ik_solver->cartSumSquaredError(x, grad);
}


ChainIkSolverPos_sqp::ChainIkSolverPos_sqp(const KDL::Chain& chain, const KDL::JntArray& q_min, const KDL::JntArray& q_max, const KDL::JntArray& q_opt, const KDL::Twist& tol, double maxtime, double eps_pos) :
	chain_(chain),
	fk_solver_(chain), jac_solver_(chain),
	tolerance_(tol), 
	maxtime_(maxtime),
	eps_pos_(eps_pos),
	weight_rot_(0.1), weight_q_(0.0001)
{
	// nlopt_solver_ = std::make_unique<nlopt::opt>(nlopt::LD_SLSQP, _chain.getNrOfJoints());
	nlopt_solver_ = nlopt_create(NLOPT_LD_SLSQP, chain_.getNrOfJoints());
	if (nlopt_solver_ == nullptr) {
		error = E_NLOPT_FAILED;
		nlopt_result_ = NLOPT_FAILURE;
		throw std::runtime_error("Unable to create NLOPT solver: dimention = " + std::to_string(chain_.getNrOfJoints()));
	}
	// limits
	if (setJointLimits(q_min, q_max) != E_NOERROR) throw std::runtime_error(std::string("Unable to set limits: ") + strError(error));
	if (setJointOpt(q_opt) != E_NOERROR) throw std::runtime_error(std::string("Unable to set optimal pose: ") + strError(error));
	// data buffers
	int n_joints = chain_.getNrOfJoints();
	q_.resize(n_joints);
	upper_bounds_.resize(n_joints);
	lower_bounds_.resize(n_joints);
	jac_.resize(n_joints);
	q_jnt_.resize(n_joints);
	best_q_jnt_.resize(n_joints);
}

ChainIkSolverPos_sqp::~ChainIkSolverPos_sqp()
{
	nlopt_destroy(nlopt_solver_);
}

void ChainIkSolverPos_sqp::updateInternalDataStructures()
{
	const int n_joints = chain_.getNrOfJoints();
	// update KDL solvers
	fk_solver_.updateInternalDataStructures();
	jac_solver_.updateInternalDataStructures();
	// update NLP solver
	nlopt_destroy(nlopt_solver_);
	nlopt_solver_ = nlopt_create(NLOPT_LD_SLSQP, n_joints);
	if (nlopt_solver_ == nullptr) {
		error = E_NLOPT_FAILED;
		throw std::runtime_error("Unable to create NLOPT solver: dimention = " + std::to_string(n_joints));
	}
	// update limits and optimal pose
	q_min_.data.conservativeResizeLike(Eigen::VectorXd::Constant(n_joints, std::numeric_limits<double>::lowest()));
    q_max_.data.conservativeResizeLike(Eigen::VectorXd::Constant(n_joints, std::numeric_limits<double>::max()));
    q_opt_.data.conservativeResizeLike(Eigen::VectorXd::Constant(n_joints, 0.0));
	// data buffers
	q_.resize(n_joints);
	upper_bounds_.resize(n_joints);
	lower_bounds_.resize(n_joints);
	jac_.resize(n_joints);
	q_jnt_.resize(n_joints);
	best_q_jnt_.resize(n_joints);
}

int ChainIkSolverPos_sqp::CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out)
{
	// const std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();

	// check arguments
	if (q_init.data.size() != chain_.getNrOfJoints()) {
		return E_SIZE_MISMATCH;
	}
	// check solver
	if (nlopt_get_dimension(nlopt_solver_) != chain_.getNrOfJoints()) {
		return E_NOT_UP_TO_DATE;
	}

	// desired pose
	b_Tref_ee_ = p_in;

	// calculate bounds and initial value
	int k = 0;
	for (const KDL::Segment& segment : chain_.segments) {
		switch (segment.getJoint().getType()) {
			case KDL::Joint::RotAxis:
			case KDL::Joint::RotX:
			case KDL::Joint::RotY:
			case KDL::Joint::RotZ:
				// get initial value
				q_[k] = q_init(k);
				// check limits for rotational joint
				if (q_[k] > q_max_(k)) {
					// adjust seed to comform upper limit
					double diffangle = fmod(q_[k] - q_max_(k), 2 * M_PI);
					q_[k] = q_max_(k) + diffangle - 2 * M_PI;
				}
				if (q_[k] < q_min_(k)) {
					// adjust seed to comform lower limit
					double diffangle = fmod(q_min_(k) - q_[k], 2 * M_PI);
					q_[k] = q_min_(k) - diffangle + 2 * M_PI;
					// check upper limit again 
					if (q_[k] > q_max_(k)) {
						// angle can not be preserved and both limits are finite
						q_[k] = (q_max_(k) + q_min_(k)) / 2.0;
					}
				}
				// adjust bounds to limit search space 
				lower_bounds_[k] = std::max(q_min_(k), q_[k] - 2 * M_PI);
				upper_bounds_[k] = std::min(q_max_(k), q_[k] + 2 * M_PI);
				// increase joint index
				k++;
				break;

			case KDL::Joint::TransAxis:
			case KDL::Joint::TransX:
			case KDL::Joint::TransY:
			case KDL::Joint::TransZ:
				// get initial value
				q_[k] = q_init(k);
				// check limits
				q_[k] = std::min(q_[k], q_max_(k));
				q_[k] = std::max(q_[k], q_min_(k));
				// adjust limits
				lower_bounds_[k] = q_min_(k);
				upper_bounds_[k] = q_max_(k);
				// increase joint index
				k++;
				break;

			case KDL::Joint::Fixed:
				break;
		}
	}

	// configure solver
	nlopt_result result;
	if ((result = nlopt_set_lower_bounds(nlopt_solver_, lower_bounds_.data())) < 0 ||
		(result = nlopt_set_upper_bounds(nlopt_solver_, upper_bounds_.data())) < 0 ||
		(result = nlopt_set_maxtime(nlopt_solver_, maxtime_)) < 0 ||
		(result = nlopt_set_xtol_abs1(nlopt_solver_, std::numeric_limits<double>::epsilon())) < 0 ||
		(result = nlopt_set_ftol_abs(nlopt_solver_, std::numeric_limits<double>::epsilon())) < 0 ||
		(result = nlopt_set_min_objective(nlopt_solver_, cartSumSquaredError_adapter, this)) < 0)

	{
		nlopt_result_ = result;
		return (error = (E_NLOPT_FAILED + result));
	}

	// save initial as first 
	best_q_fval_ = std::numeric_limits<double>::max(); 
	best_q_in_tolerance_ = false;

	// optimization
	double minf;
	nlopt_result_ = nlopt_optimize(nlopt_solver_, q_.data(), &minf);
	if (nlopt_result_ < 0) {
		error = E_NLOPT_FAILED + nlopt_result_;
		return error;
	}

	// check result
	if (best_q_in_tolerance_) {
		// exact (within tolerance) solution is found
		error = E_NOERROR;
		q_out = best_q_jnt_;
	}
	else {
		// degraded solution
		error = E_DEGRADED;
		q_out.data = Eigen::Map<Eigen::VectorXd>(q_.data(), q_.size());
	}
	return error;
}

const char* ChainIkSolverPos_sqp::strError(const int error) const 
{
    if ( (error > (E_NLOPT_FAILED + NLOPT_NUM_FAILURES)) || (error < (E_NLOPT_FAILED + NLOPT_NUM_RESULTS)) ) return "NLOpt solver error.";
	if (error == E_INVALID_LIMITS) return "Bad joint limits (q_min > q_max).";
    else return SolverI::strError(error);
}

const char* ChainIkSolverPos_sqp::strNLOptResult(const int result) const 
{
	return nlopt_result_to_string(static_cast<nlopt_result>(result));
}

int ChainIkSolverPos_sqp::setJointLimits(const KDL::JntArray& q_min, const KDL::JntArray& q_max)
{
	int n_joints = chain_.getNrOfJoints();
	if (q_min.rows() != n_joints || q_max.rows() != n_joints) {
		return (error = E_SIZE_MISMATCH);
	}
	if ( (q_min.data.array() > q_max.data.array()).any() ) {
		return (error = E_INVALID_LIMITS);
	}
	q_min_ = q_min;
	q_max_ = q_max;
	return E_NOERROR;
}

int ChainIkSolverPos_sqp::setJointOpt(const KDL::JntArray& q_opt)
{
	if (q_opt.rows() != chain_.getNrOfJoints()) {
		return (error = E_SIZE_MISMATCH);
	}
	q_opt_ = q_opt;
	return (error = E_NOERROR);
}

double ChainIkSolverPos_sqp::cartSumSquaredError(const double * q_data, double * grad_data)
{
	double weight_rot_square = std::pow(weight_rot_, 2);
	double weight_q_square = std::pow(weight_q_, 2);
	KDL::Frame b_T_ee;
	// solve FK problem
  	q_jnt_.data = Eigen::Map<const Eigen::VectorXd>(q_data, chain_.getNrOfJoints());
  	int res = fk_solver_.JntToCart(q_jnt_, b_T_ee);
	// calculate error
  	KDL::Twist tdiff = KDL::diff(b_T_ee, b_Tref_ee_); // w.r.t. b with ref point at ee
	double fval = KDL::dot(tdiff.vel, tdiff.vel) + 
	              weight_rot_square * KDL::dot(tdiff.rot, tdiff.rot) +
	              weight_q_square * (q_jnt_.data - q_opt_.data).squaredNorm();
	// calulate gradient
	if (grad_data != nullptr) {
		Eigen::Map<Eigen::VectorXd> grad(grad_data, chain_.getNrOfJoints());
		grad.setZero();
		// calulate jacobian 
		jac_solver_.JntToJac(q_jnt_, jac_);
		// gradient: translation
		for(int i = 0; i < 3; i++) {
			grad -= (2.0 * tdiff.vel[i]) * jac_.data.row(i);
		}
		//// gradient: rotation
		for(int i = 0; i < 3; i++) {
			grad -= (weight_rot_square * 2.0 * tdiff.rot[i]) * jac_.data.row(i+3);
		}
		// gradient: optimal pose
		grad += (2.0 * weight_q_square) * (q_jnt_.data - q_opt_.data);
	}
	// check tolerance ans save appropriate solution
	bool in_tolerance_limits = true;
	for(int k = 0; k < 3; k++) {
		if (std::abs(tdiff.vel[k]) > std::max(tolerance_.vel[k], eps_pos_)) {
			in_tolerance_limits = false;
			break;
		}
		if ((std::abs(tdiff.rot[k]) > tolerance_.rot[k]) && (weight_rot_*std::abs(tdiff.rot[k]) > eps_pos_)) {
			in_tolerance_limits = false;
			break;
		}
	}
	if (in_tolerance_limits && best_q_fval_ > fval) {
		best_q_jnt_ = q_jnt_;
		best_q_fval_ = fval;
		best_q_in_tolerance_ = true; 
	}
	return fval;
}


} /* namespace KDL */
