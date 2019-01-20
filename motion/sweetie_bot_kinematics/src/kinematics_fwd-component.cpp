#include "kinematics_fwd-component.hpp"

#include <rtt/Component.hpp>
#include <iostream>
#include <ros/time.h>

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
	this->addProperty( "virtual_links", virtual_links )
		.doc( "If kinematics chain ends with virtual links publish pose of the last virtual link instead real one.")
		.set(true);

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
		// add new data structure
		chain_data.emplace_back();
		KinematicChainData& data = chain_data.back();
		// set information about chain
		// get kinematic chain
		data.chain = unique_ptr<KDL::Chain>(new KDL::Chain(robot_model->getKDLChain(name, virtual_links)));  // if virtual_links is true receive chain with virtual links
		if (data.chain->getNrOfSegments() == 0) {
			this->log(ERROR) << "Kinematic chain " << name << " does not exist." << endlog();
			return false;
		}
		data.name = name;
		//joint induces
		//TODO make induces calculation more effective 
		//TODO remove redundancy
		vector<string> chain_joints = robot_model->listJoints(name); // contains fictive joints
		data.index_begin = robot_model->getJointIndex(chain_joints.front());
		data.jnt_array_vel.resize(data.chain->getNrOfJoints()); // some joints can be fictive!
		// solvers
		data.fk_vel_solver = unique_ptr<ChainFkSolverVel_recursive>(new ChainFkSolverVel_recursive(*data.chain));
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
	in_joints_port.getDataSample(joints);

	this->log(INFO) << "KinematicsFwd is configured." <<endlog();
	return true;
}

bool KinematicsFwd::startHook(){
  this->log(INFO) << "KinematicsFwd is started." <<endlog();
  return true;
}

void KinematicsFwd::updateHook()
{
	int k;
	if ( in_joints_port.read(joints, false) == NewData && isValidJointStatePos(joints, n_joints) ) {
		// we received valid JointState message
		// calculate tip pose for each chain
		for (k = 0; k < chain_data.size(); k++ ) {
			KinematicChainData& data = chain_data[k];
			// get chain pose in joint space
			data.jnt_array_vel.q.data = Eigen::VectorXd::Map( &joints.position[data.index_begin], data.chain->getNrOfJoints() );
			if ( joints.velocity.size() != 0 ) {
				data.jnt_array_vel.qdot.data = Eigen::VectorXd::Map( &joints.velocity[data.index_begin], data.chain->getNrOfJoints() );
			} 
			else {
				KDL::SetToZero(data.jnt_array_vel.qdot);
			}
			// calculate chain pose
			KDL::FrameVel frame_vel;
			int ret = data.fk_vel_solver->JntToCart(data.jnt_array_vel, frame_vel);
			if (ret < 0){
				this->log(ERROR) << "ForwardKinematics returned = " << ret << " for chain " << data.name << endlog();
				continue;
			}
			// fill message
			limbs.frame[k] = frame_vel.value();
			// KDL kinematics functions utilize pose twist, so we have perform conversion and move origin to base_link center.
			// Why?!!  Why it is not screw twist?!!
			limbs.twist[k] = frame_vel.deriv().RefPoint(-frame_vel.value().p);
		}
		// publish message
		limbs.header.stamp = ros::Time::now();
		out_limbs_port.write(limbs);
	}
	log(DEBUG) << "Update hook executed: " << k << " joints processed." <<endlog();
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
