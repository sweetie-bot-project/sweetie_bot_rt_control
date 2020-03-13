#include "kinematics_fwd_force-component.hpp"

#include <Eigen/SVD>

#include <rtt/Component.hpp>
#include <ros/time.h>

#include <sweetie_bot_orocos_misc/joint_state_check.hpp>
#include <sweetie_bot_orocos_misc/get_subservice_by_type.hpp>

using sweetie_bot::logger::Logger;
using namespace RTT;
using namespace KDL;

namespace sweetie_bot {
namespace motion {

KinematicsFwdForce::KinematicsFwdForce(const std::string& name) : 
	TaskContext(name, PreOperational),
	log(logger::categoryFromComponentName(name))
{
	// PORTS
	this->addEventPort( "in_joints_sorted", in_joints_port )
		.doc( "Full robot pose to perform forward kinematics calculation." );
	this->addPort( "out_limbs_fixed", out_limbs_port )
		.doc( "Kinematic chains' end segments poses and velocities in cartesian coordinates." );

	// PROPERTIES
	this->addProperty( "kinematic_chains", chain_names )
		.doc("List of kinematic chains for which poses are calculated.");
	this->addProperty( "virtual_links", virtual_links )
		.doc("If kinematics chain ends with virtual links publish pose of the last virtual link instead real one.")
		.set(true);
	this->addProperty("threshold", threshold)
		.doc("Eigen JacobiSVD::solve threshold during wrench calculation. Can be used to handle singular poses.")
		.set(0.01);
	this->addProperty("weight_force", weight_force)
		.doc("Force component of wrench weight during least square method application: wrench_norm = weight_force * force_norm + weight_torque * torque_norm.")
		.set(1.0);
	this->addProperty("weight_torque", weight_torque)
		.doc("Torque component of wrench weight during least square method application: wrench_norm = weight_force * force_norm + weight_torque * torque_norm.")
		.set(10.0);

	// SERVICE: reqiures
	robot_model = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));

	this->log(INFO) << "KinematicsFwdForce is constructed." << endlog();
}

bool KinematicsFwdForce::configureHook()
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
		data.chain.reset( new KDL::Chain(robot_model->getKDLChain(name, virtual_links)) );  // if virtual_links is true receive chain with virtual links
		if (data.chain->getNrOfSegments() == 0) {
			this->log(ERROR) << "Kinematic chain " << name << " does not exist." << endlog();
			return false;
		}
		data.name = name;
		//joint induces
		data.joint_induces = robot_model->getChainJointsInduces(name, virtual_links);
		data.jnt_array.resize(data.chain->getNrOfJoints()); 
		data.jac.resize(data.chain->getNrOfJoints()); 
		// solvers
		data.fk_pos_solver.reset( new ChainFkSolverPos_recursive(*data.chain) );
		data.fk_jac_solver.reset( new ChainJntToJacSolver(*data.chain) );
		// add chain to output list buffer
		limbs.name.push_back(name);
	};
	// get number of joints
	n_joints = robot_model->listJoints().size();

	// init port data
	limbs.frame.resize(limbs.name.size());
	limbs.twist.resize(limbs.name.size());
	limbs.wrench.resize(limbs.name.size());
	// data samples
	out_limbs_port.setDataSample(limbs);
	in_joints_port.getDataSample(joints);

	this->log(INFO) << "KinematicsFwdForce is configured." <<endlog();
	return true;
}

bool KinematicsFwdForce::startHook(){
	this->log(INFO) << "KinematicsFwdForce is started." <<endlog();
	return true;
}

void KinematicsFwdForce::updateHook()
{
	if ( in_joints_port.read(joints, false) == NewData) {
		// check message
		if ( ! isValidJointStatePos(joints, n_joints) ) {
			log(DEBUG) << "JointState message is not valid! n_joints = " << n_joints << endlog();
			return;
		}

		Eigen::JacobiSVD< Eigen::Matrix<double, Eigen::Dynamic, 6> > jac_svd;
		// we received valid JointState message
		// calculate tip pose for each chain
		for (int k = 0; k < chain_data.size(); k++ ) {
			KinematicChainData& data = chain_data[k];
			// get chain pose in joint space
			for(int i = 0; i < data.chain->getNrOfJoints(); i++) data.jnt_array.data[i] = joints.position[data.joint_induces[i]];
			// solve FK
			int ret = data.fk_pos_solver->JntToCart(data.jnt_array, limbs.frame[k]);
			if (ret < 0){
				this->log(ERROR) << "ForwardKinematics returned = " << ret << " for chain " << data.name << endlog();
				continue;
			}
			// calculate Jacobian
			ret = data.fk_jac_solver->JntToJac(data.jnt_array, data.jac);
			if (ret < 0){
				this->log(ERROR) << "JntToJac returned  = " << ret << " for chain " << data.name << endlog();
				continue;
			}
			// transform velocity
			if (joints.velocity.size() > 0) {
				// get chain velocity in joint space
				for(int i = 0; i < data.chain->getNrOfJoints(); i++) data.jnt_array.data[i] = joints.velocity[data.joint_induces[i]];
				// calculate velocity using Jacobian
				KDL::Twist twist;
				KDL::MultiplyJacobian(data.jac, data.jnt_array, twist);
				// KDL kinematics functions utilize pose twist, so to get srew twist we have to perform conversion and move origin to base_link center.
				limbs.twist[k] = twist.RefPoint(-limbs.frame[k].p);
			}
			else {
				KDL::SetToZero(limbs.twist[k]);
			}
			// transform force
			if (joints.effort.size() > 0) {
				// get chain force in joint space
				for(int i = 0; i < data.chain->getNrOfJoints(); i++) data.jnt_array.data[i] = joints.effort[data.joint_induces[i]];
				// apply weight coefficient 
				data.jac.data.topRows<3>() *= weight_force / weight_torque;
				// SVD decomposition
				jac_svd.compute(data.jac.data.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV); // transpose reallocation
				jac_svd.setThreshold(threshold);
				// calculate wrench
				Eigen::Matrix<double,6,1> w_bar = jac_svd.solve(data.jnt_array.data);
				// return wrench
				limbs.wrench[k].force = KDL::Vector(w_bar[3], w_bar[4], w_bar[5]);
				limbs.wrench[k].torque = KDL::Vector(w_bar[1], w_bar[2], w_bar[3]) * (weight_force / weight_torque);
			}
			else {
				KDL::SetToZero(limbs.wrench[k]);
			}
		}
		// publish message
		limbs.header.stamp = ros::Time::now();
		out_limbs_port.write(limbs);
	}
}

void KinematicsFwdForce::stopHook() 
{
	this->log(INFO) << "KinematicsFwdForce is stoped." <<endlog();
}

void KinematicsFwdForce::cleanupHook() 
{
	chain_data.clear();
	this->log(INFO) << "KinematicsFwdForce is cleaned up." <<endlog();
}

} // namespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(sweetie_bot::motion::KinematicsFwdForce)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::KinematicsFwdForce)
