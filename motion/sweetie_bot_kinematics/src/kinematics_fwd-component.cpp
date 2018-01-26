#include "kinematics_fwd-component.hpp"

#include <rtt/Component.hpp>
#include <iostream>

#include <sweetie_bot_orocos_misc/joint_state_check.hpp>
#include <sweetie_bot_orocos_misc/get_subservice_by_type.hpp>

using sweetie_bot::logger::Logger;
using namespace RTT;
using namespace KDL;

namespace sweetie_bot {
namespace motion {

KinematicsFwd::KinematicsFwd(string const& name) : 
	TaskContext(name, PreOperational),
	log(logger::categoryFromComponentName(name))
{
	this->addEventPort( "in_joints_sorted", in_joints_port )
		.doc( "Full robot pose to perform forward kinematics calculation." );
	this->addPort( "out_limbs_fixed", out_limbs_port )
		.doc( "Kinematic chains' end segments poses and velocities in cartesian coordinates." );

	this->addProperty( "kinematic_chains", chain_names )
		.doc( "List of kinematic chains for which poses are calculated.");

	// Service: reqires
	robot_model = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));

	this->log(INFO) << "KinematicsFwd is constructed." << endlog();
}

bool KinematicsFwd::configureHook()
{
	// check if RobotModel Service presents
	if (!robot_model->ready() || !robot_model->isConfigured()) {
		log(ERROR) << "RobotModel service is not ready." << endlog();
		return false;
	}

	// construct chain info structure
	// buffers all essential information: nmae and solver
	chain_data.clear();
	limbs.name.clear();
	for(auto &name: chain_names) {
		KinematicChainData data;
		// add information about chain
		// get kinematic chain
		Chain chain = robot_model->getKDLChain(name, false); // we need only real joints
		if (chain.segments.size() == 0) {
			this->log(ERROR) << "Kinematic chain " << name << " does not exist." << endlog();
			continue;
		}
		data.name = name;
		//joint induces
		//TODO make induces calculation more effective 
		//TODO remove redundancy
		vector<string> chain_joints = robot_model->listJoints(name); // contains fictive joints
		data.index_begin = robot_model->getJointIndex(chain_joints.front());
		data.size = chain.getNrOfJoints(); // some joints can be fictive!
		data.jnt_array_vel.resize(data.size);
		// solvers
		// data.fk_solver = make_shared<ChainFkSolverPos_recursive>(chain);
		data.fk_vel_solver = make_shared<ChainFkSolverVel_recursive>(chain);
		// save data
		chain_data.push_back(data);
		// add chain to output list buffer
		limbs.name.push_back(name);
	};
	// get number of joints
	n_joints = robot_model->listJoints("").size();
		
	// init port data
	limbs.frame.resize(limbs.name.size());
	limbs.twist.resize(limbs.name.size());
	joints.name.resize(n_joints);
	joints.position.resize(n_joints);
	joints.velocity.resize(n_joints);
	// data samples
	out_limbs_port.setDataSample(limbs);

	this->log(INFO) << "KinematicsFwd is configured." <<endlog();
	return true;
}

bool KinematicsFwd::startHook(){
  this->log(INFO) << "KinematicsFwd is started." <<endlog();
  return true;
}

void KinematicsFwd::updateHook()
{
	if ( in_joints_port.read(joints, false) == NewData && isValidJointStatePos(joints, n_joints) ) {
		// we received valid JointState message
		// calculate tip pose for each chain
		for (int k = 0; k < chain_data.size(); k++ ) {
			KinematicChainData& data = chain_data[k];
			// get chain pose in joint space
			data.jnt_array_vel.q.data = Eigen::VectorXd::Map( &joints.position[data.index_begin], data.size );
			if ( joints.velocity.size() != 0 ) {
				data.jnt_array_vel.qdot.data = Eigen::VectorXd::Map( &joints.velocity[data.index_begin], data.size );
			} 
			else {
				KDL::SetToZero(data.jnt_array_vel.qdot);
			}
			// calculate chain pose in absolute coordinates
			// TODO add Jacobian calulation
			// TODO exclude extra copy
			KDL::FrameVel frame_vel;
			int ret = data.fk_vel_solver->JntToCart(data.jnt_array_vel, frame_vel);
			if (ret < 0){
				this->log(ERROR) << "ForwardKinematics returned = " << ret << " for chain " << data.name  << endlog();
				continue;
			}
			// fill message
			limbs.frame[k] = frame_vel.value();
			limbs.twist[k] = frame_vel.deriv();
		}
		// publish message
		out_limbs_port.write(limbs);
	}
}

void KinematicsFwd::stopHook() 
{
	this->log(INFO) << "KinematicsFwd is stoped." <<endlog();
}

void KinematicsFwd::cleanupHook() 
{
	chain_data.clear();
	this->log(INFO) << "KinematicsFwd is cleaned up." <<endlog();
}

} // namespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(sweetie_bot::motion::KinematicsFwd)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::KinematicsFwd)
