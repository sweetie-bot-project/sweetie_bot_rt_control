#include <sweetie_bot_orocos_misc/stream_operators.hpp>

#include "kinematics_inv-component.hpp"

#include <cmath>
#include <algorithm>

#include <rtt/Component.hpp>

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>

#include <sweetie_bot_orocos_misc/message_checks.hpp>
#include <sweetie_bot_orocos_misc/joint_state_check.hpp>
#include <sweetie_bot_orocos_misc/get_subservice_by_type.hpp>

using sweetie_bot::logger::Logger;
using namespace RTT;
using namespace KDL;

namespace sweetie_bot {
namespace motion {

KinematicsInv::KinematicsInv(const std::string& name) : 
	TaskContext(name, PreOperational),
	log(logger::categoryFromComponentName(name))
{
	// ports
	this->addEventPort("in_joints_sorted", in_joints_seed_port_)
		.doc( "Initial robot pose for inverse kinematic calculation. Full sorted pose expexted. It is used as result if IK solution not found." );
	this->addEventPort("in_limbs", in_limbs_port_ )
		.doc( "Target pose for inverse kinematic calculation." );
	this->addPort("out_joints", out_joints_port_ )
		.doc( "Inverse kinematic result data port." );
	// properties
	this->addProperty( "kinematic_chains", chain_names_ )
		.doc( "List of kinematic chains for which poses are calculated. Other chains are ignored.");
	this->addProperty( "max_iterations", max_iterations_ )
		.doc( "Maximum number of iterations for instantaneous IK solver.")
		.set(100);
	this->addProperty( "eps_vel", eps_vel_ )
		.doc( "Singular values less then eps is assumed to be zero. Instantaneous IK solvers parameter.")
		.set(1e-2);
	this->addProperty( "zero_vel_at_singularity", zero_vel_at_singularity_ )
		.doc( "Set velocity to zero near singularity.")
		.set(true);
	this->addProperty( "use_ik_pose_as_new_seed", use_ik_pose_as_new_seed_ )
		.doc( "Renew chain seed pose with newly calculated IK pose.")
		.set(false);
	this->addProperty("period", period_)
		.doc("Discretization period (s).");
	this->addProperty( "max_joint_velocity", max_joint_velocity_ )
		.doc( "Maximal allowed joint speed (rad/s). Tis value is used to set LOCALITY_VIOLATION_FLAG. Max joint shift from current pose is equal to max_joint_velocity*period. Set to zero skip max joint shift test.")
		.set(0);
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

	this->log(INFO) << "KinematicsInv constructed." <<endlog();
}

bool KinematicsInv::configureHook()
{
	// check if RobotModel Service presents
	if (!robot_model_->ready() || !robot_model_->isConfigured()) {
		log(ERROR) << "RobotModel service is not ready." << endlog();
		return false;
	}

	// load solver factories
	std::vector<SolverIKFactoryInterface *> solver_ik_factories = getAllSubServicesByType<SolverIKFactoryInterface>(this->provides());
	if (solver_ik_factories.size() == 0) {
		log(ERROR) << "KinematicsInv: at least one 'solver_ik_factory' service should present." << endlog();
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
		// get kinematic chain
		// check if chain exist
		if (robot_model_->getChainIndex(name) < 0) {
			log(ERROR) << "Chain " << name << " is not registered in robot_model." << endlog();
			return false;
		}
		// get kdl_chain
		data.chain.reset( new KDL::Chain( robot_model_->getKDLChain(name, true) ) ); // we need real and virtual joints
		data.name = name;
		//joint induces
		data.joint_names = robot_model_->getChainJoints(name); // contains fictive joints
		data.joint_induces = robot_model_->getChainJointsInduces(name, true);
		data.size = data.chain->getNrOfJoints(); // some joints can be fictive!
		data.size_real = robot_model_->getKDLChain(name, false).getNrOfJoints(); //
		data.jnt_array_pose.resize(data.size);
		data.jnt_array_vel.resize(data.size);
		data.jnt_array_seed_pose.resize(data.size);
		// solvers
		// FK solver
		data.fk_solver.reset( new KDL::ChainFkSolverPos_recursive(*data.chain) );
		// instantaneous IK initialization
		data.ik_vel_solver.reset( new KDL::ChainIkSolverVel_pinv(*data.chain, eps_vel_, max_iterations_) );
		// IK initialization 
		data.ik_solver = getIKSolver(name, *data.chain, solver_ik_factories);
		if (!data.ik_solver) return false;
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
	in_joints_seed_port_.getDataSample(joints_);
	in_limbs_port_.getDataSample(limbs_);
	// reserve memory
	joints_.name.reserve(n_joints_fullpose_);
	joints_.position.reserve(n_joints_fullpose_);
	joints_.velocity.reserve(n_joints_fullpose_);

	this->log(INFO) << "KinematicsInv is configured." <<endlog();
	return true;
}


std::unique_ptr<SolverIKInterface> KinematicsInv::getIKSolver(const std::string& name, const Chain& chain, const std::vector<SolverIKFactoryInterface *>& solver_ik_factories) 
{
	int n_joints = chain.getNrOfJoints();

	//
	// get limit properties
	//
	Property< std::vector<double> >	q_max_prop = this->getProperty(name + "_q_max");
	Property< std::vector<double> >	q_min_prop = this->getProperty(name + "_q_min");
	// check properties and extract its values
	KDL::JntArray q_max(n_joints), q_min(n_joints);
	if (q_max_prop.ready() && q_min_prop.ready()) {
		// check properties sizes
		if (q_max_prop.rvalue().size() != n_joints || q_min_prop.rvalue().size() != n_joints) {
			log(ERROR) << "Incorrect " << name << "_q_max or " << name << "_q_min property size." << endlog();
			return nullptr;
		}
		// correct properties: assign its values to JntArrays
		q_min.data = Eigen::Map<Eigen::VectorXd>(&q_min_prop.value().front(), n_joints);
		q_max.data = Eigen::Map<Eigen::VectorXd>(&q_max_prop.value().front(), n_joints);
	}
	else {
		log(INFO) << name << "_q_max and " << name << "_q_min properties are not provided. Use default values from URDF model." << endlog();
		// get limits from RobotMode
		q_min = robot_model_->getChainLowerLimits(name);
		q_max = robot_model_->getChainUpperLimits(name);
		// check result for sanity
		if (q_min.rows() != n_joints || q_max.rows() != n_joints) {
			log(ERROR) << "Unable to get defult joint limits for chain " << name << "." << endlog();
			return nullptr;
		}
		// now add properties to component interface
		std::vector<double> q_min_vec(n_joints);
		std::vector<double> q_max_vec(n_joints);
		Eigen::Map<Eigen::VectorXd>(&q_min_vec.front(), n_joints) = q_min.data;
		Eigen::Map<Eigen::VectorXd>(&q_max_vec.front(), n_joints) = q_max.data;
		this->properties()->ownProperty( new Property< std::vector<double> >(name + "_q_min", "Lower joints limits", q_min_vec) );
		this->properties()->ownProperty( new Property< std::vector<double> >(name + "_q_max", "Upper joints limits", q_max_vec) );
	}

	//
	// get tolerance property
	//
	Property< std::vector<double> >	tolerance_prop = this->getProperty(name + "_tolerance");
	KDL::Twist tolerance;
	if (tolerance_prop.ready()) {
		if (tolerance_prop.rvalue().size() != 6) {
			log(ERROR) << "Tolerance specifications must contain six elements (chain " << name << "). " << endlog();
			return nullptr;
		}
		// copy tolerance values
		for(int shift = 0; shift < 6; shift++) tolerance[shift] = tolerance_prop.rvalue()[shift];
	}

	//
	// get solver
	//
	for (SolverIKFactoryInterface * factory : solver_ik_factories) {
		std::unique_ptr<SolverIKInterface> solver = factory->getSolver(name, chain, q_min, q_max, tolerance, log);
		if (solver) return solver;
	}
	log(ERROR) << "Unable to find appropriate solver for chain " << name << "." << endlog();
	return nullptr;
}

bool KinematicsInv::startHook()
{
	// get data samples
	//in_limbs_port_.getDataSample(limbs_);
	//in_joints_seed_port_.getDataSample(joints_);
	// read seed pose
	if (in_joints_seed_port_.read(joints_, true) != NoData && isValidJointStatePos(joints_, n_joints_fullpose_)) {
		// update seeds
		for ( KinematicChainData& chain_data : chain_data_ ) {
			for(int i = 0; i < chain_data.size; i++) chain_data.jnt_array_seed_pose.data[i] = joints_.position[chain_data.joint_induces[i]];
		}
	}

	this->log(INFO) << "KinematicsInv started." <<endlog();
	return true;
}


int KinematicsInv::poseToJointState_impl(const sweetie_bot_kinematics_msgs::RigidBodyState& limbs_, sensor_msgs::JointState& joints_) 
{
	// WARNING! Correct limbs_ message is assumed!
	
	// clear message buffer
	joints_.name.clear();
	joints_.position.clear();
	joints_.velocity.clear();

	// process message
	// if IK fails om any point it discards all results
	int result = 0;
	for (int k = 0; k < limbs_.name.size(); k++) {
		const std::string& name = limbs_.name[k];
		int limb_result = 0;
		// check if chain is known
		auto chain_it = std::find_if(chain_data_.begin(), chain_data_.end(), [name](const KinematicChainData& data) { return data.name == name; });
		if (chain_it == chain_data_.end()) {
			// continue silently
			this->log(DEBUG) << "Skip chain " << name << endlog();
			continue;
		}
		joints_.name.insert(joints_.name.end(), chain_it->joint_names.begin(), chain_it->joint_names.end());

		// inverse kinematics
		int ret =  chain_it->ik_solver->solveIK(limbs_.frame[k], chain_it->jnt_array_seed_pose, chain_it->jnt_array_pose, log);
		switch (ret) {
			case SolverIKInterface::NO_SOLUTION:
				this->log(DEBUG) << "IK failed: no solution found." << endlog();
				return NO_SOLUTION;

			case SolverIKInterface::APPROXIMATE_SOLUTION:
				limb_result |= TOLERANCE_VIOLATION_FLAG;
				break;
		}

		// check joints pose change: calculate joints shift
		double max_joint_shift = max_joint_velocity_ * period_;
		if (max_joint_shift > 0) {
			// check if maximal speed is exceeded
			for(int k = 0; k < chain_it->size_real; k++) {
				if ( std::abs(chain_it->jnt_array_seed_pose(k) - chain_it->jnt_array_pose(k)) > max_joint_shift ) {
					// joint shift is too large 
					limb_result |= LOCALITY_VIOLATION_FLAG;
					if (this->log(DEBUG)) {
						log() << "IK failed " << name << ": non local solution found, joint " << k << " shift is greater max_joint_shift: |";
						log() << chain_it->jnt_array_pose(k) << " - " << chain_it->jnt_array_seed_pose(k) << "| > " << max_joint_shift << endlog();
					}
				}
			}
		}

		// use computed pose as new seed
		if (use_ik_pose_as_new_seed_) chain_it->jnt_array_seed_pose = chain_it->jnt_array_pose;

		// pack result into JointState message
		joints_.position.insert(joints_.position.end(), chain_it->jnt_array_pose.data.data(), chain_it->jnt_array_pose.data.data() + chain_it->size);
	
		// check if velocities present
		if (limbs_.twist.size() != 0) {
			// instantaneous inverse kinematics
			// KDL kinematics functions utilize pose twist, so we have perform conversion.
			// Why?!!  Why it is not screw twist?!!
			ret = chain_it->ik_vel_solver->CartToJnt(chain_it->jnt_array_pose, limbs_.twist[k].RefPoint(limbs_.frame[k].p), chain_it->jnt_array_vel); 
			if (ret < 0) {
				this->log(DEBUG) << "Instantaneous IK failed with error code: " << ret << endlog();
				// fill speed with zeros
				return NO_SOLUTION;
			}

			// compare two solutions for local IK
			/* if (log(DEBUG)) {
				log() << "Velocity shift: " << (chain_it->jnt_array_vel.data * period_).transpose() << std::endl;
				log() << "Joint shift:   " << (chain_it->jnt_array_pose.data - chain_it->jnt_array_seed_pose.data).transpose() << endlog();
			} */

			// check singular values 
			if (zero_vel_at_singularity_ && chain_it->ik_vel_solver->getNrZeroSigmas() > (chain_it->size - 6)) {
				// set velocity to zero near singularity
				joints_.velocity.insert(joints_.velocity.end(), chain_it->size, 0.0);
			}
			else {
				// pack result into JointState message
				joints_.velocity.insert(joints_.velocity.end(), chain_it->jnt_array_vel.data.data(), chain_it->jnt_array_vel.data.data() + chain_it->size);
			}

			// check direction violation
			{
				// solve FK
			}
		} 
		else {
			// fill with zeros
			joints_.velocity.insert(joints_.velocity.end(), chain_it->size, 0.0);
		}

		// result overall
		if (log(DEBUG)) {
			log() << "IK for chain: " << chain_it->name  << " result " << limb_result  << endlog();
		} 
		result |= limb_result;
	}
	return result;
}

int KinematicsInv::poseToJointState(const sweetie_bot_kinematics_msgs::RigidBodyState& limbs_, sensor_msgs::JointState& joints_) {
	if (!this->isRunning()) {
		log(ERROR) << "poseToJointState: KinematicsInv must be running!" << endlog();
		return false;
	}
	// check message 
	if ( !isValidRigidBodyStateNameFrame(limbs_)) {
		log(ERROR) << "poseToJointState: Incorrect RigidBodyState." << endlog();
		return false;
	}
	// call IK solver
	return KinematicsInv::poseToJointState_impl(limbs_, joints_);
}


bool KinematicsInv::poseToJointStatePublish(const sweetie_bot_kinematics_msgs::RigidBodyState& limbs_, int allow_mode) 
{
	if (!this->isRunning()) {
		log(ERROR) << "poseToJointStatePublish: KinematicsInv must be running!" << endlog();
		return false;
	}
	// check message 
	if ( !isValidRigidBodyStateNameFrame(limbs_)) {
		log(ERROR) << "poseToJointStatePublish: Incorrect RigidBodyState. Joints are not publised." << endlog();
		return false;
	}

	// invoke IK solvers, use joints_ as buffer
	int result = poseToJointState_impl(limbs_, joints_);
	bool success = (result & ~allow_mode) == 0;

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
				break;
			}
			// failsafe values
			joints_.name.insert(joints_.name.end(), chain_it->joint_names.begin(), chain_it->joint_names.end());
			joints_.position.insert(joints_.position.end(), chain_it->jnt_array_seed_pose.data.data(), chain_it->jnt_array_seed_pose.data.data() + chain_it->size);
			joints_.velocity.insert(joints_.velocity.end(), chain_it->size, 0.0);
		}
	}
	out_joints_port_.write(joints_);

	return success;
}

void KinematicsInv::updateHook()
{
	int j = 0;
	// Check if new seeds arrived
	if ( in_joints_seed_port_.read(joints_, false) == NewData ) {
		if (isValidJointStatePos(joints_, n_joints_fullpose_)) {
			j++;
			// update seeds
			for ( KinematicChainData& chain_data : chain_data_ ) {
				for(int i = 0; i < chain_data.size; i++) chain_data.jnt_array_seed_pose.data[i] = joints_.position[chain_data.joint_induces[i]];
			}
		}
		else {
			log(WARN) << "Incorrect message on in_joints_seed_port. Expected full robot pose." << joints_ << endlog();
		}
	}

	// Check for IK requests
	int l = 0;
	while ( in_limbs_port_.read(limbs_) == NewData ) {
		// process received message
		poseToJointStatePublish(limbs_, 0); // failsafe mode
		l++;
	}
	log(DEBUG) << "Update hook executed: " << j << " joints and " << l << " limbs processed." <<endlog();
}

void KinematicsInv::stopHook() 
{
	log(INFO) << "KinematicsInv stoped." <<endlog();
}

void KinematicsInv::cleanupHook() 
{
	chain_data_.clear();

	log(INFO) << "KinematicsInv cleaning up." <<endlog();
}

} // namespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(sweetie_bot::motion::KinematicsInv)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::KinematicsInv)
