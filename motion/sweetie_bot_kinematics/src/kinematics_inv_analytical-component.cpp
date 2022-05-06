#include "kinematics_inv_analytical-component.hpp"

#include <cmath>
#include <algorithm>

#include <rtt/Component.hpp>

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>

#include <sweetie_bot_orocos_misc/message_checks.hpp>
#include <sweetie_bot_orocos_misc/joint_state_check.hpp>

#include "kdl_ostream.hpp"

using sweetie_bot::logger::Logger;
using namespace RTT;
using namespace KDL;

namespace sweetie_bot {
namespace motion {

KinematicsInvAnalytical::KinematicsInvAnalytical(const std::string& name) : 
	TaskContext(name, PreOperational),
	log(logger::categoryFromComponentName(name))
{
	// ports
	this->addEventPort("in_joints_sorted", in_joints_port_)
		.doc( "Current robot pose. Full sorted pose expexted." );
	this->addEventPort("in_limbs", in_limbs_port_ )
		.doc( "Target pose for inverse kinematic calculation." );
	this->addPort("out_joints", out_joints_port_ )
		.doc( "Inverse kinematic result data port." );
	// properties
	this->addProperty( "kinematic_chains", chain_names_ )
		.doc( "List of kinematic chains for which poses are calculated. Other chains are ignored.");
	this->addProperty( "tolerance_pos", tolerance_pos_ )
		.doc( "Position tolerance (m).")
		.set(1e-4);
	this->addProperty( "max_iterations", max_iterations_ )
		.doc( "Maximum number of iterations for instantaneous IK solver.")
		.set(100);
	this->addProperty( "eps_vel", eps_vel_ )
		.doc( "Singular values less then eps is assumed to be zero. Instantaneous IK solvers parameter.")
		.set(1e-2);
	this->addProperty( "zero_vel_at_singularity", zero_vel_at_singularity_ )
		.doc( "Set velocity to zero near singularity.")
		.set(true);
	this->addProperty( "max_joint_velocity", max_joint_velocity_ )
		.doc( "Maximal allowed joint speed (rad/s). Max joint shift from current pose is equal to max_joint_velocity*period. Set to zero skip max joint shift test.")
		.set(0);
	this->addProperty("period", period_)
		.doc("Discretization period (s).");
	// operations
	this->addOperation("poseToJointState", &KinematicsInvAnalytical::poseToJointState, this, OwnThread)
		.doc("Process IK request syncronously. Unknown chains are ignored. Return true if request succesed. Otherwise result message is incorrect and should be ignored.")
		.arg("in", "Desired pose and speed of kinematic chains relative to its bases.")
		.arg("out", "IK result for known kinematic chains");
	this->addOperation("poseToJointStatePublish", &KinematicsInvAnalytical::poseToJointStatePublish, this, OwnThread)
		.doc("Process IK request syncronously and publish result on out_joints_port. Unknown chains are ignored. Return false and nothing published if solver fails.")
		.arg("in", "Desired pose and speed of kinematic chains relative to its bases.");
	// Service: requires
	robot_model_ = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model_));

	this->log(INFO) << "KinematicsInvAnalytical constructed." <<endlog();
}

bool KinematicsInvAnalytical::configureHook()
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
		data.size_real = robot_model_->getKDLChain(name, false).getNrOfJoints();
		data.jnt_array_pose.resize(data.size);
		data.jnt_array_vel.resize(data.size);
		// check if chain can be processed by analytical solver and get solver
		log(DEBUG) << "Finding solver for chain " << name << endlog();
		data.ik_pos_solver = SolverIKFactory::GetSolver(*data.chain, log);
		if ( !data.ik_pos_solver ) {
			log(ERROR) << name << ": compatible IK solver is not found." << endlog();
			return false;
		}
		// instantaneous IK solver
		data.ik_vel_solver.reset( new KDL::ChainIkSolverVel_pinv(*data.chain, eps_vel_, max_iterations_) );
		// joints limits
		Property< std::vector<double> >	q_max_prop = this->getProperty(name + "_q_max");
		Property< std::vector<double> >	q_min_prop = this->getProperty(name + "_q_min");

		if (q_max_prop.ready() && q_min_prop.ready()) {
			// check properties sizes
			if (q_max_prop.rvalue().size() != data.size || q_min_prop.rvalue().size() != data.size) {
				log(ERROR) << "Incorrect " << name << "_q_max or " << name << "_q_min property size." << endlog();
				return false;
			}
			// correct properties: assign its values to JntArrays
			data.jnt_lower_bounds.data = Eigen::Map<Eigen::VectorXd>(&q_min_prop.value().front(), data.size);
			data.jnt_upper_bounds.data = Eigen::Map<Eigen::VectorXd>(&q_max_prop.value().front(), data.size);
		}
		else {
			log(ERROR) << name << "_q_max and " << name << "_q_min properties are not provided. (Loading values from URDF model is not implemented)." << endlog();
			return false;
		}
	};
	// get number of joints
	n_joints_fullpose_ = robot_model_->listJoints().size();

	if (log(DEBUG)) {
		log() << "Loaded " << chain_names_.size() << " chains: ";
		for( const auto& name : chain_names_ ) log() << name << ", ";
		log() << endlog();
	}

	// init port data
	joints_result_.name.reserve(n_joints);
	joints_result_.position.reserve(n_joints);
	joints_result_.velocity.reserve(n_joints);
	// data samples
	out_joints_port_.setDataSample(joints_result_);
	in_joints_port_.getDataSample(joints_current_);
	in_limbs_port_.getDataSample(limbs_);
	// reserve memory
	joints_current_.name.reserve(n_joints_fullpose_);
	joints_current_.position.reserve(n_joints_fullpose_);
	joints_current_.velocity.reserve(n_joints_fullpose_);

	log(INFO) << "KinematicsInvAnalytical is configured." <<endlog();
	return true;
}

bool KinematicsInvAnalytical::startHook()
{
	// get data samples
	//in_limbs_port_.getDataSample(limbs_);
	// read note that it 
	in_joints_port_.readNewest(joints_current_, true);

	this->log(INFO) << "KinematicsInvAnalytical started." <<endlog();
	return true;
}

bool KinematicsInvAnalytical::poseToJointState_impl(const sweetie_bot_kinematics_msgs::RigidBodyState& limbs_, sensor_msgs::JointState& joints_result_) 
{
	// WARNING! Correct limbs_ message is assumed!
	
	// clear message buffer
	joints_result_.name.clear();
	joints_result_.position.clear();
	joints_result_.velocity.clear();

	// process message
	// if IK fails om any point it discards all results
	for (int k = 0; k < limbs_.name.size(); k++) {
		const std::string& name = limbs_.name[k];
		// check if chain is known
		auto chain_it = std::find_if(chain_data_.begin(), chain_data_.end(), [name](const KinematicChainData& data) { return data.name == name; });
		if (chain_it == chain_data_.end()) {
			// continue silently
			this->log(DEBUG) << "Skip chain " << name << endlog();
			continue;
		}
		joints_result_.name.insert(joints_result_.name.end(), chain_it->joint_names.begin(), chain_it->joint_names.end());

		// inverse kinematics
		bool success = chain_it->ik_pos_solver->solveIK(*chain_it->chain, limbs_.frame[k], chain_it->jnt_lower_bounds, chain_it->jnt_upper_bounds, chain_it->jnt_array_pose, log);
		if (!success) {
			return false;
		}

		// check joints pose change: calculate joints shift
		double max_joint_shift = max_joint_velocity_ * period_;
		if (max_joint_shift > 0 && isValidJointStatePos(joints_current_, n_joints_fullpose_)) {
			// check if maximal speed is exceeded
			for(int k = 0; k < chain_it->size_real; k++) {
				if ( std::abs(joints_current_.position[chain_it->joint_induces[k]] - chain_it->jnt_array_pose(k)) > max_joint_shift ) {
					// joint shift is too large 
					this->log(DEBUG) << "IK failed " << chain_it->name << ": non local solution found, joint " << k << " shift is greate max_joint_shift." << endlog();
					return false;
				}
			}
		}

		// pack result into JointState message
		joints_result_.position.insert(joints_result_.position.end(), chain_it->jnt_array_pose.data.data(), chain_it->jnt_array_pose.data.data() + chain_it->size);
	
		// check if velocities present
		if (limbs_.twist.size() != 0) {
			// instantaneous inverse kinematics
			// KDL kinematics functions utilize pose twist, so we have perform conversion.
			// Why?!!  Why it is not screw twist?!!
			int ret = chain_it->ik_vel_solver->CartToJnt(chain_it->jnt_array_pose, limbs_.twist[k].RefPoint(limbs_.frame[k].p), chain_it->jnt_array_vel); 
			if (ret < 0) {
				this->log(DEBUG) << "Instantaneous IK failed with error code: " << ret << endlog();
				// fill speed with zeros
				return false;
			}

			// compare two solutions for local IK
			if (log(DEBUG)) {
				log() << "Velocity shift: " << (chain_it->jnt_array_vel.data * period_).transpose() << std::endl;
				Eigen::VectorXd shift(chain_it->size);
				for(int k = 0; k < chain_it->size; k++) shift[k] =  chain_it->jnt_array_pose(k) - joints_current_.position[chain_it->joint_induces[k]];
				log() << "Joint shift:   " << shift.transpose() << endlog();
			}

			// check singular values 
			if (zero_vel_at_singularity_ && chain_it->ik_vel_solver->getNrZeroSigmas() > 0) {
				// set velocity to zero near singularity
				joints_result_.velocity.insert(joints_result_.velocity.end(), chain_it->size, 0.0);
			}
			else {
				// pack result into JointState message
				joints_result_.velocity.insert(joints_result_.velocity.end(), chain_it->jnt_array_vel.data.data(), chain_it->jnt_array_vel.data.data() + chain_it->size);
			}
		} 
		else {
			// fill with zeros
			joints_result_.velocity.insert(joints_result_.velocity.end(), chain_it->size, 0.0);
		}
	}
	return true;
}

bool KinematicsInvAnalytical::poseToJointState(const sweetie_bot_kinematics_msgs::RigidBodyState& limbs_, sensor_msgs::JointState& joints_) {
	if (!this->isRunning()) {
		log(ERROR) << "poseToJointState: KinematicsInvAnalytical must be running!" << endlog();
		return false;
	}
	// check message 
	if ( !isValidRigidBodyStateNameFrame(limbs_)) {
		log(ERROR) << "poseToJointState: Incorrect RigidBodyState." << endlog();
		return false;
	}
	// call IK solver
	return KinematicsInvAnalytical::poseToJointState_impl(limbs_, joints_);
}


bool KinematicsInvAnalytical::poseToJointStatePublish(const sweetie_bot_kinematics_msgs::RigidBodyState& limbs_) 
{
	if (!this->isRunning()) {
		log(ERROR) << "poseToJointStatePublish: KinematicsInvAnalytical must be running!" << endlog();
		return false;
	}
	// check message 
	if ( !isValidRigidBodyStateNameFrame(limbs_)) {
		log(ERROR) << "poseToJointStatePublish: Incorrect RigidBodyState. Joints are not publised." << endlog();
		return false;
	}

	// invoke IK solvers, use joints_ as buffer
	bool success = poseToJointState_impl(limbs_, joints_result_);

	if (success) {
		out_joints_port_.write(joints_result_);
	}

	return success;
}

void KinematicsInvAnalytical::updateHook()
{
	// Check if new current pose is arrived
	if ( in_joints_port_.read(joints_current_, false) == NewData ) {
		if (!isValidJointStatePos(joints_current_, n_joints_fullpose_)) {
			log(WARN) << "Incorrect message on in_joints_port. Expected full robot pose." << joints_current_ << endlog();
		}
	}

	// Check for IK requests
	int l = 0;
	while ( in_limbs_port_.read(limbs_) == NewData ) {
		// process received message
		poseToJointStatePublish(limbs_);
		l++;
	}
	log(DEBUG) << "Update hook executed: " << l << " limbs processed." <<endlog();
}

void KinematicsInvAnalytical::stopHook() 
{
	log(INFO) << "KinematicsInvAnalytical stoped." <<endlog();
}

void KinematicsInvAnalytical::cleanupHook() 
{
	chain_data_.clear();

	log(INFO) << "KinematicsInvAnalytical cleaning up." <<endlog();
}

} // namespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(sweetie_bot::motion::KinematicsInvAnalytical)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::KinematicsInvAnalytical)
