#include "kinematics_inv_trac_ik-component.hpp"

#include <rtt/Component.hpp>

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>

#include <sweetie_bot_orocos_misc/message_checks.hpp>
#include <sweetie_bot_orocos_misc/joint_state_check.hpp>

using sweetie_bot::logger::Logger;
using namespace RTT;
using namespace KDL;

namespace sweetie_bot {
namespace motion {

KinematicsInvTracIK::KinematicsInvTracIK(string const& name) : 
	TaskContext(name, PreOperational),
	log(logger::categoryFromComponentName(name))
{
	// ports
	this->addEventPort("in_joints_seed_sorted", in_joints_seed_port_)
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
	this->addProperty( "eps_pos", eps_pos_ )
		.doc( "Singular values less then eps is assumed to be zero. TracIk solver parameter.")
		.set(1e-4);
	this->addProperty( "eps_vel", eps_vel_ )
		.doc( "Singular values less then eps is assumed to be zero. Instantaneous IK solvers parameter.")
		.set(1e-2);
	this->addProperty( "zero_vel_at_singularity", zero_vel_at_singularity_ )
		.doc( "Set velocity to zero near singularity.")
		.set(true);
	this->addProperty( "timeout_pos", timeout_ )
		.doc( "TracIK solver timeout (s).")
		.set(0.003);
	this->addProperty( "use_ik_pose_as_new_seed", use_ik_pose_as_new_seed_ )
		.doc( "Renew chain seed pose with newly calculated IK pose.")
		.set(false);
	// operations
	this->addOperation("poseToJointState", &KinematicsInvTracIK::poseToJointState, this, OwnThread)
		.doc("Process IK request syncronously. Unknown chains are ignored. Return true if request succesed. Otherwise result message is incorrect and should be ignored.")
		.arg("in", "Desired pose and speed of kinematic chains relative to its bases.")
		.arg("out", "IK result for known kinematic chains");
	this->addOperation("poseToJointStatePublish", &KinematicsInvTracIK::poseToJointStatePublish, this, OwnThread)
		.doc("Process IK request syncronously and publish result on out_joints_port. Unknown chains are ignored. Return false if solver fails. In this case seed pose is publised.")
		.arg("in", "Desired pose and speed of kinematic chains relative to its bases.");
	// Service: requires
	robot_model_ = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model_));

	this->log(INFO) << "KinematicsInvTracIK constructed." <<endlog();
}

bool KinematicsInvTracIK::configureHook()
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
		KinematicChainData data;
		// add information about chain
		// get kinematic chain
		// check if chain exist
		if (robot_model_->getChainIndex(name) < 0) {
			log(ERROR) << "Chain " << name << " is not registered in robot_model." << endlog();
			return nullptr;
		}
		// get kdl_chain
		Chain chain = robot_model_->getKDLChain(name, true); // we need real and virtual joints

		data.name = name;
		//joint induces
		data.joint_names = robot_model_->listJoints(name); // contains fictive joints
		data.index_begin = robot_model_->getJointIndex(data.joint_names.front());
		data.size = chain.getNrOfJoints(); // some joints can be fictive!
		data.jnt_array_pose.resize(data.size);
		data.jnt_array_vel.resize(data.size);
		data.jnt_array_seed_pose.resize(data.size);
		// solvers
		// instantaneous IK initialization
		data.ik_vel_solver = make_shared<KDL::ChainIkSolverVel_pinv>(chain, eps_vel_, max_iterations_);
		// IK initialization
		data.ik_solver = getIKSolver(name, chain);
		// save data
		chain_data_.push_back(data);
	};
	// get number of joints
	n_joints_fullpose_ = robot_model_->listJoints("").size();
		
	// init port data
	joints_.name.reserve(n_joints);
	joints_.position.reserve(n_joints);
	joints_.velocity.reserve(n_joints);
	// data samples
	out_joints_port_.setDataSample(joints_);
	// reserve memory
	joints_.name.reserve(n_joints_fullpose_);
	joints_.position.reserve(n_joints_fullpose_);
	joints_.velocity.reserve(n_joints_fullpose_);

	this->log(INFO) << "KinematicsInvTracIK is configured." <<endlog();
	return true;
}


std::shared_ptr<TRAC_IK::TRAC_IK> KinematicsInvTracIK::getIKSolver(const string& name, const Chain& chain) 
{
	int n_joints = chain.getNrOfJoints();
	// check limit properties
	Property< std::vector<double> >	q_max_prop = this->getProperty(name + "_q_max");
	Property< std::vector<double> >	q_min_prop = this->getProperty(name + "_q_min");

	if (q_max_prop.ready() && q_min_prop.ready()) {
		// check properties sizes
		if (q_max_prop.rvalue().size() != n_joints || q_min_prop.rvalue().size() != n_joints) {
			log(ERROR) << "Incorrect " << name << "_q_max or " << name << "_q_min property size." << endlog();
			return nullptr;
		}
		// correct properties: assign its values to JntArrays
		JntArray q_min(n_joints), q_max(n_joints); 
		q_min.data = Eigen::Map<Eigen::VectorXd>(&q_min_prop.value().front(), n_joints);
		q_max.data = Eigen::Map<Eigen::VectorXd>(&q_max_prop.value().front(), n_joints);
		// init IK
		return make_shared<TRAC_IK::TRAC_IK>(chain, q_min, q_max, timeout_, eps_pos_, TRAC_IK::Speed);
	}
	else {
		// hard way: use URDF to extract joints limits, because robot_model does not provides them
		log(INFO) << name << "_q_max and " << name << "_q_min properties are not provided. Load values from URDF model." << endlog();
		// this function tries to access ROS parameter server directly...
		// TODO add chain limits to robot_model
		auto ik_solver = make_shared<TRAC_IK::TRAC_IK>(robot_model_->getChainProperty(name, "first_link"), robot_model_->getChainProperty(name, "last_link_virtual"), "robot_description", timeout_, eps_pos_, TRAC_IK::Speed);
		// get limits and check if initialization successed
		JntArray q_min, q_max; 
		if (!ik_solver->getKDLLimits(q_min, q_max) || q_min.rows() != n_joints || q_max.rows() != n_joints) {
			log(ERROR) << "Unable to initialize TracIk solver for chain " << name << ". first_link: " << robot_model_->getChainProperty(name, "first_link") << " last_link_virtual: " << robot_model_->getChainProperty(name, "last_link_virtual") << endlog();
			return nullptr;
		}
		// now add properties to component interface
		std::vector<double> q_min_vec(n_joints);
		std::vector<double> q_max_vec(n_joints);
		Eigen::Map<Eigen::VectorXd>(&q_min_vec.front(), n_joints) = q_min.data;
		Eigen::Map<Eigen::VectorXd>(&q_max_vec.front(), n_joints) = q_max.data;
		this->properties()->ownProperty( new Property< std::vector<double> >(name + "_q_min", "Lower joints limits", q_min_vec) );
		this->properties()->ownProperty( new Property< std::vector<double> >(name + "_q_max", "Upper joints limits", q_max_vec) );

		return ik_solver;
	}
}

bool KinematicsInvTracIK::startHook()
{
	// get data samples
	in_limbs_port_.getDataSample(limbs_);
	in_joints_seed_port_.getDataSample(joints_);
	// read seed pose
	if (in_joints_seed_port_.read(joints_, true) != NoData && isValidJointStatePos(joints_, n_joints_fullpose_)) {
		// update seeds
		for ( KinematicChainData& chain_data : chain_data_ ) {
			chain_data.jnt_array_seed_pose.data = Eigen::Map<Eigen::VectorXd>( &joints_.position[chain_data.index_begin], chain_data.size );
		}
	}

	this->log(INFO) << "KinematicsInvTracIK started." <<endlog();
	return true;
}


bool KinematicsInvTracIK::poseToJointState_impl(const sweetie_bot_kinematics_msgs::RigidBodyState& limbs_, sensor_msgs::JointState& joints_) 
{
	// WARNING! Correct limbs_ message is assumed!
	
	// clear message buffer
	joints_.name.clear();
	joints_.position.clear();
	joints_.velocity.clear();

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
		joints_.name.insert(joints_.name.end(), chain_it->joint_names.begin(), chain_it->joint_names.end());

		// inverse kinematics
		int ret =  chain_it->ik_solver->CartToJnt(chain_it->jnt_array_seed_pose, limbs_.frame[k], chain_it->jnt_array_pose); //, tolerances);
		if (ret < 0) {
			this->log(DEBUG) << "IK failed with error code: " << ret <<endlog();
			return false;
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
				return false;
			}
			// check singular values 
			if (zero_vel_at_singularity_ && chain_it->ik_vel_solver->getNrZeroSigmas() > (chain_it->size - 6)) {
				// set velocity to zero near singularity
				joints_.velocity.insert(joints_.velocity.end(), chain_it->size, 0.0);
			}
			else {
				// pack result into JointState message
				joints_.velocity.insert(joints_.velocity.end(), chain_it->jnt_array_vel.data.data(), chain_it->jnt_array_vel.data.data() + chain_it->size);
			}
		} 
		else {
			// fill with zeros
			// TODO more effective
			joints_.velocity.insert(joints_.velocity.end(), chain_it->size, 0.0);
		}
	}
	return true;
}

bool KinematicsInvTracIK::poseToJointState(const sweetie_bot_kinematics_msgs::RigidBodyState& limbs_, sensor_msgs::JointState& joints_) {
	if (!this->isRunning()) {
		log(ERROR) << "poseToJointState: KinematicsInvTracIK must be running!" << endlog();
		return false;
	}
	// check message 
	if ( !isValidRigidBodyStateNameFrame(limbs_)) {
		log(ERROR) << "poseToJointState: Incorrect RigidBodyState." << endlog();
		return false;
	}
	// call IK solver
	return KinematicsInvTracIK::poseToJointState_impl(limbs_, joints_);
}


bool KinematicsInvTracIK::poseToJointStatePublish(const sweetie_bot_kinematics_msgs::RigidBodyState& limbs_) 
{
	if (!this->isRunning()) {
		log(ERROR) << "poseToJointStatePublish: KinematicsInvTracIK must be running!" << endlog();
		return false;
	}
	// check message 
	if ( !isValidRigidBodyStateNameFrame(limbs_)) {
		log(ERROR) << "poseToJointStatePublish: Incorrect RigidBodyState. Joints are not publised." << endlog();
		return false;
	}

	// invoke IK solvers, use joints_ as buffer
	bool success = poseToJointState_impl(limbs_, joints_);

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

void KinematicsInvTracIK::updateHook()
{
	// Check if new seeds arrived
	if ( in_joints_seed_port_.read(joints_, false) == NewData ) {
		if (isValidJointStatePos(joints_, n_joints_fullpose_)) {
			// update seeds
			for ( KinematicChainData& chain_data : chain_data_ ) {
				chain_data.jnt_array_seed_pose.data = Eigen::Map<Eigen::VectorXd>( &joints_.position[chain_data.index_begin], chain_data.size );
			}
		}
		else {
			log(WARN) << "Incorrect message on in_joints_seed_port. Expected full robot pose." << joints_ << endlog();
		}
	}

	// Check for IK requests
	while ( in_limbs_port_.read(limbs_) == NewData ) {
		// process received message
		poseToJointStatePublish(limbs_);
	}

}

void KinematicsInvTracIK::stopHook() 
{
	log(INFO) << "KinematicsInvTracIK stoped." <<endlog();
}

void KinematicsInvTracIK::cleanupHook() 
{
	chain_data_.clear();

	log(INFO) << "KinematicsInvTracIK cleaning up." <<endlog();
}

} // namespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(sweetie_bot::motion::KinematicsInvTracIK)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::KinematicsInvTracIK)
