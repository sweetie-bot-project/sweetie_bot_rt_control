#include <sweetie_bot_orocos_misc/stream_operators.hpp>

#include "kinematics_inv_control-component.hpp"

#include <cmath>
#include <algorithm>
#include <sstream>

#include <rtt/Component.hpp>

#include <kdl/chainiksolvervel_pinv_nso.hpp>

#include <sweetie_bot_orocos_misc/message_checks.hpp>
#include <sweetie_bot_orocos_misc/joint_state_check.hpp>
#include <sweetie_bot_orocos_misc/get_subservice_by_type.hpp>


using sweetie_bot::logger::Logger;
using namespace RTT;
using namespace KDL;

namespace sweetie_bot {
namespace motion {


KinematicsInvControl::IKControlSolver::IKControlSolver(const KDL::Chain& chain, const Eigen::VectorXd& q_min, const Eigen::VectorXd& q_max, const Eigen::VectorXd& q_opt, const Eigen::VectorXd& weights, double jac_svd_threshold) :
	// solver initialization
	chain_(chain),
	fk_solver_(chain_),
	jac_solver_(chain_),
	// preallocated SVD solver
	J_svd_(6, chain.getNrOfJoints()),
	// buffers
	dq_dz_(chain.getNrOfJoints()),
	dC_dz_(chain.getNrOfJoints()),
	J_(chain.getNrOfJoints())
{
	// check sizes
	unsigned int n_joints = chain.getNrOfJoints();
	if (q_min.size() != n_joints || q_max.size() != n_joints || q_opt.size() != n_joints || weights.size() != n_joints) {
		throw std::runtime_error("invalid sovler configuration: q_min, q_max, q_opt and weight vectors must have the same size as kinematic chain.");
	}
	if ( (q_opt.array() <= q_min.array()).any() || (q_opt.array() >= q_max.array()).any() ) {
		throw std::runtime_error("invalid sovler configuration: assertion q_min < q_opt < q_max is failed.");
	}
	// translate limits to center/scale pair
	q_center_ = 0.5*(q_min + q_max);	
	q_scale_ = (q_max - q_min) / M_PI;
	// store optimal pose and weights
	z_opt_ = Eigen::tan( (q_opt.array() - q_center_.array()) / q_scale_.array() );
	weights_ = weights;
	// SVD solver singulr value detection
	J_svd_.setThreshold(jac_svd_threshold);
}

bool KinematicsInvControl::IKControlSolver::step(JntArrayVel& state, const KDL::Frame& target_frame, const KDL::Twist& target_twist, double T, double Kp_rot, double Kp_pos, double alpha,  double q_reduction_factor, double max_rot_vel, double max_pos_vel, sweetie_bot::logger::Logger& log) 
{
	// calculate jacobian
	if (jac_solver_.JntToJac(state.q, J_) < 0) return false;
	// and end effector position
	KDL::Frame current_frame;
	if (fk_solver_.JntToCart(state.q, current_frame) < 0) return false;

	/*if (log(DEBUG)) {
		log() << "state: q: " << state.q.data.transpose() << ", qdot : " << state.qdot.data.transpose() << std::endl;
		log() << "frame current: " << current_frame << std::endl << "Frame target: " << target_frame << std::endl;
		log() << "jacobian: " << J_.data << endlog();
	}*/

	// temporaries
	Eigen::VectorXd& q = state.q.data;
	Eigen::VectorXd& dq = state.qdot.data;

	// joint transform: q -> z
	// NOTE! use q to store z
	{
		const double limit = q_reduction_factor*M_PI/2.0;
		q = (q.array() - q_center_.array()) / q_scale_.array();
		q = q.cwiseMax(-limit).cwiseMin(limit);
		// calculate transformed pose
		q = Eigen::tan( q.array() );
		// transform jacobian
		dq_dz_ = q_scale_.array() * (q.array().square() + 1).inverse();
		// modify chain jacobian
		// TODO noalias() to avoid allocation 
		J_.data = J_.data * dq_dz_.asDiagonal();
	}

	// SVD decomposition for jacobian
	J_svd_.compute(J_.data, Eigen::ComputeThinU | Eigen::ComputeThinV);

	/*if (log(DEBUG)) {
		log() << "state: z: " << state.q.data.transpose() << std::endl;
		log() << "jacobian z: " << J_.data << endlog();
	}*/

	// primary target: miminimize pose difference
	// dq / dt = J^{+} ( Kp * diff(current_frame, target_frame) + target_twist )
	{
		// calculate positional twist assotiated with postion error 
		KDL::Twist diff_twist = KDL::diff(current_frame, target_frame, 1.0);
		// apply feedback coafficient and reference velocity
		diff_twist.vel = Kp_pos * diff_twist.vel;
		diff_twist.rot = Kp_rot * diff_twist.rot;
		diff_twist += target_twist.RefPoint(current_frame.p);
		// limit speed 
		if (double norm = diff_twist.rot.Norm(); norm > max_rot_vel) { diff_twist.rot = diff_twist.rot * max_rot_vel/norm; }
		if (double norm = diff_twist.vel.Norm(); norm > max_pos_vel) { diff_twist.vel = diff_twist.vel * max_pos_vel/norm; }
		// convert to Eigen
		Eigen::Matrix<double, 6, 1> diff_twist_vec;
		diff_twist_vec.head<3>() = Eigen::Map<Eigen::Vector3d>(diff_twist.vel.data);
		diff_twist_vec.tail<3>() = Eigen::Map<Eigen::Vector3d>(diff_twist.rot.data);
		// joint-space velocity
		dq = J_svd_.solve(diff_twist_vec);
			
		//if (log(DEBUG)) {
			//log() << "singular: n: "  << J_svd_.rank() << ", values: " << J_svd_.singularValues().transpose() << std::endl;
			//log() << "target 1: diff twist: " << diff_twist  << ", dz: " << dq.transpose() << std::endl;
		//}
	}


	// secondary target C(z): 
	// d/dt C(z) = - alpha C(z)
	{
		// C(z) value
		double C = 0.5 * ((q.array() - z_opt_.array()).square() * weights_.array()).sum();
		// C(z) gradient as [size 1] vector
		dC_dz_ = (q.array() - z_opt_.array()) * weights_.array();
		double A = dC_dz_.dot(dC_dz_);
		// project gradient onto jacobian null-space
		dq += (- alpha * C / A) * (dC_dz_ - J_svd_.solve(J_.data * dC_dz_));

		//if (log(DEBUG)) {
			//Eigen::VectorXd P = dC_dz_ - J_svd_.solve(J_.data * dC_dz_);
			//log() << "null projection: " << P.transpose() << std::endl;
		//}
	
		//if (log(DEBUG)) {
			//// log() << "target 2: C: " << C << " dC_dz: " << dC_dz_.transpose() << ", A: " << A << std::endl;
			//log() << "target 1+2: dz: " << dq.transpose() << endlog();
		//}
	}

	// integrate
	q += dq * T;

	/*if (log(DEBUG)) {
		log() << "result pose: z: " << q.transpose() << ", T: " << T << endlog();
	}*/

	// backward transform: z -> q
	{ 
		dq = dq_dz_.array() * dq.array();
		q = q_scale_.array() * Eigen::atan(q.array()) + q_center_.array();
	}

	/*if (log(DEBUG)) {
		log() << "result pose: q: " << q.transpose() << ", dq: " << dq.transpose() << endlog();
	}*/

	return true;
}

KinematicsInvControl::KinematicsInvControl(const std::string& name) : 
	TaskContext(name, PreOperational),
	log(logger::categoryFromComponentName(name))
{
	// ports
	this->addPort("in_joints_sorted", in_joints_fullpose_port_)
		.doc( "Current robot pose." );
	this->addEventPort("in_limbs", in_limbs_port_ )
		.doc( "Target pose for inverse kinematic calculation." );
	this->addPort("out_joints", out_joints_port_ )
		.doc( "Inverse kinematic result data port." );
	// properties
	this->addProperty("period", period_)
		.doc("Discretization period (s).");
	this->addProperty( "kinematic_chains", chain_names_ )
		.doc( "List of kinematic chains for which poses are calculated. Other chains are ignored.");
	this->addProperty( "n_steps", n_steps_ )
		.doc("Number of Control IK solver ierations per one control period. Effectively reduce control period in n_steps times. Use if control period is too long so causes solver instability.")
		.set(1);
	this->addProperty( "alpha", alpha_ )
		.doc( "Trajectory smoothing over n_steps.")
		.set(1.0);
	this->addProperty( "jac_svd_threshold", jac_svd_threshold_ )
		.doc("Control IK solver assumes jacobian singular values less then this value are assumed to be zero.")
		.set(1e-2);
	this->addProperty( "q_limit_reduction_factor", q_reduction_factor_ )
		.doc("Reduce joints movement limits by given factor to prevent joint getting stuck on limits.")
		.set(0.96);
	this->addProperty( "kp_rot", kp_rot_ )
		.doc("Postion convergence parameter. dp/dt = - Kp_pos p.")
		.set(10.0);
	this->addProperty( "kp_pos", kp_pos_ )
		.doc("Rotation convergence parameter. dR/dt = - Kp_pos R.")
		.set(10.0);
	this->addProperty( "kp_null", kp_null_ )
		.doc( "Null space convergence parameter. dq/dt = - Kp_null q.")
		.set(10.0);
	this->addProperty( "max_pos_vel", max_pos_vel_ )
		.doc( "Maximal velocity.")
		.set(1.0);
	this->addProperty( "max_rot_vel", max_rot_vel_ )
		.doc( "Maximal angular velocity.")
		.set(1.0);
	this->addProperty( "max_jnt_vel", max_jnt_vel_ )
		.doc( "Maximal joint space velocity.")
		.set(2.0);
	this->addProperty( "ignore_ref_twist", ignore_ref_twist_ )
		.doc("Ignore provided reference velocity.")
		.set(false);
	// operations
	this->addOperation("poseToJointState", &KinematicsInv::poseToJointState, this, OwnThread)
		.doc("Process IK request syncronously. Unknown chains are ignored. Return result code: NO_SOLUTION=-1, TOLERANCE_VIOLATION_FLAG=1, LOCALITY_VIOLATION_FLAG=2 ")
		.arg("in", "Desired pose and speed of kinematic chains relative to its bases.")
		.arg("out", "IK result for known kinematic chains");
	this->addOperation("poseToJointStatePublish", &KinematicsInv::poseToJointStatePublish, this, OwnThread)
		.doc("Process IK request syncronously and publish result on out_joints_port. Unknown chains are ignored. Return false if solver fails. In this case seed pose is publised.")
		.arg("in", "Desired pose and speed of kinematic chains relative to its bases.")
		.arg("allow_mode", "Allow solution with specific proerties: TOLERANCE_VIOLATION_FLAG=1, LOCALITY_VIOLATION_FLAG=2");
	// Service: requires
	robot_model_ = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model_));

	this->log(INFO) << "KinematicsInvControl constructed." <<endlog();
}

Eigen::VectorXd KinematicsInvControl::getOrAddVectorProperty(const std::string& name, const Eigen::VectorXd& default_value, const std::string& desc)
{
	unsigned int size = default_value.rows();
	Property< std::vector<double> >	prop = this->getProperty(name);
	// check if property does not present
	if (prop.ready()) {
		// check properties sizes
		if (prop.rvalue().size() != size) {
			std::stringstream ss;
			ss << "Incorrect " << name << " property size. Property size is " << prop.rvalue().size() << ", expected size is " << default_value.rows();
			throw std::runtime_error(ss.str());
		}
		// return value
		return Eigen::Map<Eigen::VectorXd>(&prop.value().front(), size);
	} 
	else {
		log(WARN) << "Property " << name << " is not set. Default value is used." << endlog();
		// add property
		std::vector<double> value(default_value.data(), default_value.data() + size);
		this->properties()->ownProperty( new Property< std::vector<double> >(name, desc, value) );
		// return default value
		return default_value;
	}
}

bool KinematicsInvControl::configureHook()
{
	// check if RobotModel Service presents
	if (!robot_model_->ready() || !robot_model_->isConfigured()) {
		log(ERROR) << "RobotModel service is not ready." << endlog();
		return false;
	}

	// construct chain info structure
	// buffers all essential information: nmae and solver
	chain_data_.clear();
	int n_joints = 0;
	for(auto &name: chain_names_) {
		chain_data_.emplace_back();
		KinematicChainData& data = chain_data_.back();
		// add information about chain
		// check if chain exist
		if (robot_model_->getChainIndex(name) < 0) {
			log(ERROR) << "Chain " << name << " is not registered in robot_model." << endlog();
			return false;
		}
		// assign name
		data.name = name;
		//joint induces
		data.joint_names = robot_model_->getChainJoints(name); // contains fictive joints
		data.joint_induces = robot_model_->getChainJointsInduces(name, true);
		data.size = data.joint_induces.size(); // some joints can be fictive!
		data.size_real = robot_model_->getKDLChain(name, false).getNrOfJoints(); //TODO: optimize?
		// pose tolerance
		data.tolerance.resize(6);
		data.tolerance << 0.001, 0.001, 0.001, 0.03, 0.03, 0.03; 
		data.tolerance = getOrAddVectorProperty(name + "_tolerance", data.tolerance, "Solution tolerance in format [x, y, z, r, p, y].");
		// IK solver 
		Eigen::VectorXd q_min = getOrAddVectorProperty(name + "_q_min", robot_model_->getChainLowerLimits(name).data, "Joints lower limits.");
		Eigen::VectorXd q_max = getOrAddVectorProperty(name + "_q_max", robot_model_->getChainUpperLimits(name).data, "Joints upper limits.");
		Eigen::VectorXd q_opt = getOrAddVectorProperty(name + "_q_opt", 0.5*(q_min + q_max), "Joints optimal pose.");
		Eigen::VectorXd weights = getOrAddVectorProperty(name + "_weights", Eigen::VectorXd::Ones(data.size), "Joints optimal pose.");
		data.solver.reset( new IKControlSolver( robot_model_->getKDLChain(name, true), q_min, q_max, q_opt, weights, jac_svd_threshold_ ) );
		// joint space pose buffer for Solver
		data.state.resize(data.size);
		data.state_filt.resize(data.size);
		// increase joints number
		n_joints += data.size;
	};
	// get number of joints
	n_joints_fullpose_ = robot_model_->listJoints().size();

	if (log(DEBUG)) {
		log() << "Loaded " << chain_names_.size() << " chains: ";
		for( const auto& name : chain_names_ ) log() << name << ", ";
		log() << endlog();
	}

	// init port data
	joints_.name.reserve(n_joints);
	joints_.position.reserve(n_joints);
	joints_.velocity.reserve(n_joints);
	// data samples
	out_joints_port_.setDataSample(joints_);
	in_joints_fullpose_port_.getDataSample(joints_);
	in_limbs_port_.getDataSample(limbs_);
	// reserve memory
	joints_fullpose_.name.reserve(n_joints_fullpose_);
	joints_fullpose_.position.reserve(n_joints_fullpose_);
	joints_fullpose_.velocity.reserve(n_joints_fullpose_);

	this->log(INFO) << "KinematicsInvControl is configured." <<endlog();
	return true;
}

bool KinematicsInvControl::startHook()
{
	// check if current pose is avalable
	//if (in_joints_fullpose_port_.read(joints_fullpose_, true) == NoData || !isValidJointStatePos(joints_fullpose_, n_joints_fullpose_)) {
		//this->log(ERROR) << "KinematicsInvControl: current pose is unknown, unable to start." << endlog();
		//return false;
	//}
	this->log(INFO) << "KinematicsInvControl started." << endlog();
	return true;
}


int KinematicsInvControl::poseToJointState_impl(const sweetie_bot_kinematics_msgs::RigidBodyState& limbs_, sensor_msgs::JointState& joints_) 
{
	// WARNING! Correct limbs_ message is assumed!
	
	// clear message buffer
	joints_.name.clear();
	joints_.position.clear();
	joints_.velocity.clear();

	// get current pose
	if (in_joints_fullpose_port_.read(joints_fullpose_, true) == NoData || !isValidJointStatePos(joints_fullpose_, n_joints_fullpose_)) {
		log(ERROR) << "Current pose on 'in_joints_sorted' port is unavailable or incorrect: " << joints_fullpose_ << endlog();
		return false;
	}

	// process request
	// if IK fails om any point it discards all results
	int result = 0;
	for (int k = 0; k < limbs_.name.size(); k++) {
		int limb_result = 0;
		const std::string& name = limbs_.name[k];
		// check if chain is known
		auto chain_it = std::find_if(chain_data_.begin(), chain_data_.end(), [name](const KinematicChainData& data) { return data.name == name; });
		if (chain_it == chain_data_.end()) {
			// continue silently
			this->log(DEBUG) << "Skip chain " << name << endlog();
			continue;
		}
		joints_.name.insert(joints_.name.end(), chain_it->joint_names.begin(), chain_it->joint_names.end());

		// update current pose
		for(int i = 0; i < chain_it->size; i++) {
			int index = chain_it->joint_induces[i];
			chain_it->state.q.data[i] = joints_fullpose_.position[index];
			chain_it->state.qdot.data[i] = joints_fullpose_.velocity[index];
		}

		// inverse kinematics
		double T = period_ / n_steps_;
		chain_it->state_filt = chain_it->state;
		for(int step = 0; step < n_steps_; step++) {
			bool result;
			// solver step
			if (limbs_.twist.size() > 0 && !ignore_ref_twist_) {
				result = chain_it->solver->step(chain_it->state, limbs_.frame[k], limbs_.twist[k], T, kp_rot_, kp_pos_, kp_null_, q_reduction_factor_, max_rot_vel_, max_pos_vel_, log);
			}
			else {
				result = chain_it->solver->step(chain_it->state, limbs_.frame[k], KDL::Twist::Zero(), T, kp_rot_, kp_pos_, kp_null_, q_reduction_factor_, max_rot_vel_, max_pos_vel_, log);
			}
			if (!result) {
				this->log(DEBUG) << "IK solver step failed." << endlog();
				return NO_SOLUTION;
			}
			// filtering
			chain_it->state_filt.q.data += alpha_ * (chain_it->state.q.data - chain_it->state_filt.q.data);
			chain_it->state_filt.qdot.data += alpha_ * (chain_it->state.qdot.data - chain_it->state_filt.qdot.data);
		}

		// check tolerance
		{
			// calculate result position
			KDL::Frame solution_frame;
			chain_it->solver->solveFK(chain_it->state.q, solution_frame);
			// calculate diff in base frame with origin moved to referece frame
			KDL::Twist diff_twist = KDL::diff(limbs_.frame[k], solution_frame, 1.0);
			// compare with given tolerance
			Eigen::Matrix<double, 6, 1> diff_twist_vec;
			diff_twist_vec.head<3>() = Eigen::Map<Eigen::Vector3d>(diff_twist.vel.data);
			diff_twist_vec.tail<3>() = Eigen::Map<Eigen::Vector3d>(diff_twist.rot.data);
			if ( (diff_twist_vec.array().abs() > chain_it->tolerance.array()).any() ) {
				limb_result |= TOLERANCE_VIOLATION_FLAG;
			}
		}

		if (log(DEBUG)) {
			log() << "IK result: chain " << chain_it->name << ", result: " << limb_result << std::endl;
			log() << "IK result: q: " << chain_it->state.q << ", qdot: " << chain_it->state.qdot << std::endl;
			log() << "IK result: qf: " << chain_it->state_filt.q << ", qdot: " << chain_it->state_filt.qdot << endlog();
		}

		// pack result into JointState message
		joints_.position.insert(joints_.position.end(), chain_it->state_filt.q.data.data(), chain_it->state_filt.q.data.data() + chain_it->size);
		joints_.velocity.insert(joints_.velocity.end(), chain_it->state_filt.qdot.data.data(), chain_it->state_filt.qdot.data.data() + chain_it->size);
		// merge result
		result |= limb_result;
	}
	return result;
}

int KinematicsInvControl::poseToJointState(const sweetie_bot_kinematics_msgs::RigidBodyState& limbs_, sensor_msgs::JointState& joints_) {
	if (!this->isRunning()) {
		log(ERROR) << "poseToJointState: KinematicsInvControl must be running!" << endlog();
		return false;
	}
	// check message 
	if ( !isValidRigidBodyStateNameFrame(limbs_)) {
		log(ERROR) << "poseToJointState: Incorrect RigidBodyState." << endlog();
		return false;
	}
	// call IK solver
	return KinematicsInvControl::poseToJointState_impl(limbs_, joints_);
}


bool KinematicsInvControl::poseToJointStatePublish(const sweetie_bot_kinematics_msgs::RigidBodyState& limbs_, int approx_mode) 
{
	if (!this->isRunning()) {
		log(ERROR) << "poseToJointStatePublish: KinematicsInvControl must be running!" << endlog();
		return false;
	}
	// check message 
	if ( !isValidRigidBodyStateNameFrame(limbs_)) {
		log(ERROR) << "poseToJointStatePublish: Incorrect RigidBodyState. Joints are not publised." << endlog();
		return false;
	}

	// invoke IK solvers, use joints_ as buffer
	int result = poseToJointState_impl(limbs_, joints_);
	bool success = (result & ~approx_mode) == 0;

	if (log(DEBUG)) {
		log() << "IK impl ret: " << result << ", q: " << joints_.position << ", qdot: " << joints_.velocity << endlog();
	}

	if (!success) {
		// IK failed, construct and publish failsafe message
		// because we have no means to tel requester that IK failed
		// TODO undublicate code

		// clear message buffer
		joints_.name.clear();
		joints_.position.clear();
		joints_.velocity.clear();

		for (int k = 0; k < limbs_.name.size(); k++) {
			const std::string& name = limbs_.name[k];
			// check if chain is known
			auto chain_it = std::find_if(chain_data_.begin(), chain_data_.end(), [name](const KinematicChainData& data) { return data.name == name; });
			if (chain_it == chain_data_.end()) {
				// continue silently
				continue;
			}
			// failsafe values: current pose and zero velocity
			joints_.name.insert(joints_.name.end(), chain_it->joint_names.begin(), chain_it->joint_names.end());
			for(int i = 0; i < chain_it->size; i++) {
				int index = chain_it->joint_induces[i];
				joints_.position.push_back(joints_fullpose_.position[index]);
			}
			joints_.velocity.insert(joints_.velocity.end(), chain_it->size, 0.0);
		}
	}
	out_joints_port_.write(joints_);

	return success;
}

void KinematicsInvControl::updateHook()
{
	// Check for IK requests
	int l = 0;
	while ( in_limbs_port_.read(limbs_) == NewData ) {
		// process received message
		poseToJointStatePublish(limbs_);
		l++;
	}
	log(DEBUG) << "Update hook executed: " << l << " limbs processed." <<endlog();
}

void KinematicsInvControl::stopHook() 
{
	log(INFO) << "KinematicsInvControl stoped." <<endlog();
}

void KinematicsInvControl::cleanupHook() 
{
	chain_data_.clear();

	log(INFO) << "KinematicsInvControl cleaning up." <<endlog();
}

} // namespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(sweetie_bot::motion::KinematicsInvControl)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::KinematicsInvControl)
