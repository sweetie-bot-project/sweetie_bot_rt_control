#include "dynamics_inv_simple-component.hpp"

#include <algorithm>

#include <rbdl/rbdl_utils.h>

#include <rtt/Component.hpp>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <ros/time.h>

#include <sweetie_bot_orocos_misc/joint_state_check.hpp>
#include <sweetie_bot_orocos_misc/message_checks.hpp>

#include <eigen_conversions/eigen_kdl.h>

using namespace RTT;
using namespace Eigen;
using namespace RigidBodyDynamics;

namespace sweetie_bot {
namespace motion {

DynamicsInvSimple::DynamicsInvSimple(const std::string& name) : 
	TaskContext(name, PreOperational),
	log(logger::categoryFromComponentName(name))
{
	// input ports
	this->addPort( "in_joints_ref_sorted", in_joints_ref_port )
		.doc( "Full reference robot pose in joint space (without information about floating base). Fields `position` and `velocity` must present." );
	this->addPort( "in_joints_real_sorted", in_joints_real_port )
		.doc( "Full measured robot pose in joint space (without information about floating base)." );
	this->addEventPort( "in_base_ref", in_base_ref_port )
		.doc( "Robot base reference position and velocity. This port triggers calulations when first in control cycle is received.");
	this->addPort( "in_supports_sorted", in_supports_port )
		.doc( "Active contact list." );
	this->addPort( "sync_step", sync_port )
		.doc( "Syncronization message indicates start of new control cycle." );

	// output ports
	this->addPort( "out_joints_accel_sorted", out_joints_accel_port )
		.doc( "Robot pose in joint space with drives effort and accelerations." );
	this->addPort( "out_wrenches_fixed", out_wrenches_port )
		.doc( "Reaction forces for each contact in the world frame." );
	this->addPort( "out_base", out_base_port )
		.doc( "Base link pose and sum of reaction forces in the  world frame." );
	this->addPort( "out_balance", out_balance_port )
		.doc( "Information about robot balance." );

	// properties
	this->addProperty( "robot_description", robot_description )
		.doc( "Simplified robot URDF (without fictive joints and massless links).");
	this->addProperty( "legs", legs )
		.doc("List of end effectors which can be in contact and be affected by external forces (Kinematic chains names, legs).");
	this->addProperty( "use_ref_joint_position", use_ref_joint_position )
		.doc("Use reference joints pose during calculation of inverse dynamics instead of real.")
		.set(true);
	this->addProperty( "tolerance", tolerance )
		.doc("Tolerance for contact jacobain rank estimation.")
		.set(1e-4);
	this->addProperty( "period", period )
		.doc("Discretization period for acceleration calculation.  ");

	// Service: reqiures
	robot_model = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));

	this->log(INFO) << "DynamicsInvSimple is constructed." << endlog();
}

bool DynamicsInvSimple::configureHook()
{
	// check if RobotModel Service presents
	if (!robot_model->ready() || !robot_model->isConfigured()) {
		log(ERROR) << "RobotModel service is not ready." << endlog();
		return false;
	}

	// load URDF robot model
	if ( ! RigidBodyDynamics::Addons::URDFReadFromString(robot_description.c_str(), &rbdl_model, true, false) ) {
        log(ERROR) << "Unable to parse URDF model in `robot_description` property." << endlog();
        return false;
    }
	log(INFO) << "Loaded URDF model. Movable bodies: " << rbdl_model.mBodies.size() << " joints: " << rbdl_model.mJoints.size() << " DOF: " << rbdl_model.dof_count 
		<< " q_size: " << rbdl_model.q_size << "qdot_size: " << rbdl_model.qdot_size << endlog();

	unsigned int rdbl_q_index = 0;
	// floating base is modeled with to joints. Check it!
	if (rbdl_model.mJoints.size() <= 1 || rbdl_model.mBodies.size() <= 2) {
        log(ERROR) << "Model contains not enough movable bodies to have floating platform." << endlog();
		return false;
	}
	// mJoint[0] is related to root link
	if ( rbdl_model.mJoints[1].mJointType != RigidBodyDynamics::JointTypeTranslationXYZ || rbdl_model.mJoints[2].mJointType != RigidBodyDynamics::JointTypeSpherical || rbdl_model.GetBodyName(2) != "base_link" ) {
		log(ERROR) << "First two joint must have JointTranslationXYZ and JointSpherical types (JointTypeFloatingBase). Name of link2 must be 'base_link'. Actual names: " 
			<< rbdl_model.GetBodyName(0) << ", " << rbdl_model.GetBodyName(1) << ", " << rbdl_model.GetBodyName(2) << "; types: " << rbdl_model.mJoints[1].mJointType << ", " << rbdl_model.mJoints[2].mJointType << endlog();
        return false;
	}

	// get joints names
	joints_accel.name = robot_model->listJoints();
	n_fullpose_joints = joints_accel.name.size();

	// construct index
	joint_index.assign(rbdl_model.qdot_size, -1); 
	//rbdl_index.assign(n_joints, -1); 

	// now check model structure: all joints are 1 DOF and present in RobotModel
	KDL::Tree kdl_tree = robot_model->getKDLTree();
	// now iterate over bodies and get coresponding induces
	for(int id = 3; id < rbdl_model.mBodies.size(); id++) {
		std::string body_name = rbdl_model.GetBodyName(id);
		// find it in KDL
		auto it = kdl_tree.getSegment(body_name);
		if (it == kdl_tree.getSegments().end()) {
			log(ERROR) << "Unknown link " << body_name << " with id = " << id << endlog();
			return false;
		}
		// all KDL joints has 1 DOF
		std::string joint_name = it->second.segment.getJoint().getName();
		// check DOF of RBDL
		if ( rbdl_model.mJoints[id].mDoFCount != 1 ) {
			log(ERROR) << "Joint of body " << body_name << " (id = " << id << ") has " << rbdl_model.mJoints[id].mDoFCount << " > 1 DOF. MultiDOF joints are not supported." << endlog();
			return false;
		}
		// construct index
		int fullpose_index = robot_model->getJointIndex(joint_name);
		if (fullpose_index < 0) {
			log(ERROR) << "Joint " << joint_name << " is not registered in RobotModel." << endlog();
			return false;
		}
		joint_index[rbdl_model.mJoints[id].q_index] = fullpose_index;
		//rbdl_index[fullpose_index] = rbdl_model.mJoints[id-1].q_index;

		if (log(DEBUG)) {
			log() << "body_id: " << id << " body: " << body_name << " joint: " << joint_name << " fullpose_index: " << fullpose_index << " rbdl_index: " << rbdl_model.mJoints[id].q_index << endlog();
		}
	};
	if (std::any_of(joint_index.begin()+6, joint_index.end(), [](int index) { return index < 0; })) {
		log(ERROR) << "Not all joints induces known. Aborting configuration." << endlog();
	}

	// get chains' names
	contacts.clear();
	for( const std::string& name : legs ) {
		contacts.emplace_back();

		ContactState& state = contacts.back();
		// trivial parameters
		state.limb_name = name;
		state.index = robot_model->getChainIndex(name);
		if (state.index < 0) {
			log(ERROR) << "Chain " << name << " is not registered in RobotModel." << endlog();
			return false;
		}
		state.body_id = rbdl_model.GetBodyId(robot_model->getChainProperty(name, "last_link").c_str());
		state.contact_name = robot_model->getChainDefaultContact(name);
		state.contact_points = robot_model->getContactPoints(state.contact_name);
	}

	//reserve memory
	// rbdl robot state
	Q.resize(rbdl_model.q_size);
	Q[rbdl_model.q_size - 1] = 1.0; // the last element is quaternion element
	QDot.resize(rbdl_model.qdot_size);
	QDDot.resize(rbdl_model.qdot_size);
	tau.resize(rbdl_model.qdot_size);
	tau_full.resize(rbdl_model.qdot_size);
	// contacts
	// reserve memory for contact jacobian: max three points per contact
	Jc_reserved.resize(9*contacts.size(), rbdl_model.qdot_size); //TODO RT dynamic allocation
	JcDotQDot_reserved.resize(9*contacts.size()); //TODO RT dynamic allocation
	lambda_reserved.resize(9*contacts.size());
	// reserve memory for temporary variables
	QDot_0.resize(rbdl_model.qdot_size);    // zero vector of qdot_size
	QDot_0.setZero();
	Jc_point.resize(3, rbdl_model.qdot_size);   // temporary matrix to hold point jacobian

	// port buffers
	joints_accel.position.resize(n_fullpose_joints);
	joints_accel.velocity.resize(n_fullpose_joints);
	joints_accel.acceleration.assign(n_fullpose_joints, 0.0);
	joints_accel.effort.assign(n_fullpose_joints, 0.0);
	wrenches.header.frame_id = "base_link";
	wrenches.name = legs;
	wrenches.frame.reserve(wrenches.name.size());
	wrenches.twist.reserve(wrenches.name.size());
	wrenches.wrench.reserve(wrenches.name.size());
	base.header.frame_id = "odom_combined";
	base.name.resize(1);
	base.frame.resize(1);
	base.twist.resize(1);
	base.wrench.resize(1);
	balance.header.frame_id = "odom_combined";
	balance.support_points.reserve(wrenches.name.size());
		
	// data samples
	out_joints_accel_port.setDataSample(joints_accel);
	out_wrenches_port.setDataSample(wrenches);
	out_base_port.setDataSample(base);
	out_balance_port.setDataSample(balance);

	this->log(INFO) << "DynamicsInvSimple is configured." <<endlog();
	return true;
}

bool DynamicsInvSimple::startHook()
{
	// reset state
	QDot.setZero();
	joints_accel.acceleration.assign(n_fullpose_joints, 0.0);
	// clear 
	in_joints_ref_port.getDataSample(joints_ref);
	in_joints_real_port.getDataSample(joints_real);
	in_supports_port.getDataSample(supports);
	in_base_ref_port.getDataSample(base);
	this->log(INFO) << "DynamicsInvSimple is started." <<endlog();
	return true;
}

// read ports and return true if torque should be calculated
bool DynamicsInvSimple::checkPorts() 
{
	bool trigger_update = true;
	
	// joint state
	in_joints_ref_port.read(joints_ref, false);
	if (!isValidJointStatePosVel(joints_ref, n_fullpose_joints)) {
		log(ERROR) << "Invalid JointState message on in_joints_ref_sorted port." << endlog();
		trigger_update = false;
	}

	in_joints_ref_port.read(joints_real, false);
	if (!isValidJointStatePosVel(joints_real, n_fullpose_joints)) {
		log(ERROR) << "Invalid JointState message on in_joints_real_sorted port." << endlog();
		trigger_update = false;
	}

	// support state
	in_supports_port.read(supports, false);
	if (!isValidSupportStateNameSuppCont(supports) || supports.name.size() < contacts.size()) {
		log(ERROR) << "Invalid SupportState message on in_supports_sorted port." << endlog();	
		trigger_update = false;
	}

	// base link pose
	RTT::os::Timer::TimerId tmp;
	if (in_base_ref_port.read(base, false) == NewData && sync_port.read(tmp) == NewData) {
		if (!isValidRigidBodyStateNameFrameTwist(base, 1) || base.name[0] != "base_link") {
			log(ERROR) << "Invalid RigidBodyState message on in_base port. Information about base_link is expected." << endlog();	
			trigger_update = false;
		}
	}
	else trigger_update = false;

	return trigger_update;
}

void DynamicsInvSimple::updateStateFromPortsBuffers()
{
	// joint state
	for(int i = 6; i < rbdl_model.qdot_size; i++) {
		// pose
		if (use_ref_joint_position) {
			Q[i] = joints_ref.position[joint_index[i]];
		}
		else {
			Q[i] = joints_real.position[joint_index[i]];
		}
		// acceleration calculation
		QDDot[i] = (joints_ref.velocity[joint_index[i]] - QDot[i]) / period;
		// velocity
		QDot[i] = joints_ref.velocity[joint_index[i]];
	}
	// base
	Q.head<3>() = Map<Vector3d>(base.frame[0].p.data);
	Eigen::Quaternion<double> rot;
	tf::quaternionKDLToEigen(base.frame[0].M, rot);
	rot.normalize();
	Q[3] = rot.x(); Q[4] = rot.y(); Q[5] = rot.z(); Q[rbdl_model.q_size-1] = rot.w();
	// base acceeleration: twist time derivative
	// Convert srew twist to base_link origin speed and angular velocity in base_link frame, because this is what RBDL expects
	KDL::Vector base_vel = base.twist[0].rot*base.frame[0].p + base.twist[0].vel;
	KDL::Vector base_rot = base.frame[0].M.Inverse(base.twist[0].rot);
	// estimate acceleration
	QDDot.head<3>() = ( Map<Vector3d>(base_vel.data) - QDot.head<3>() ) / period;
	QDDot.segment<3>(3) = ( Map<Vector3d>(base_rot.data) - QDot.segment<3>(3) ) / period;
	// base speed
	QDot.head<3>() = Map<Vector3d>(base_vel.data);
	QDot.segment<3>(3) = Map<Vector3d>(base_rot.data);
	// support state
	for( ContactState& contact : contacts) {
		contact.is_active = supports.support[contact.index] > 0.0;
		if (contact.is_active) {
			// update contact if necessary
			if (contact.contact_name != supports.contact[contact.index]) {
				contact.contact_points.clear();
				robot_model->addContactPointsToBuffer(contact.contact_name, contact.contact_points);
			}
		}
	}
}

void DynamicsInvSimple::inverseDynamic()
{
	// ALL CALCULATIONS IS PERFORMED IN WORLD FRAME!!! 
	// It is the only inertial frame in system.

	UpdateKinematics(rbdl_model, Q, QDot, QDot_0); // set acceleration to zero, to calculate time derivative of Jc using CalcPointAcceleration()

	// calculate contact jacobian
	int n_Jc_rows = 0;
	for( ContactState& contact : contacts ) 
		if (contact.is_active) {
			int n_points = std::min<unsigned int>(contact.contact_points.size(), 3); // no more, then 3 point per contact
			for ( int k = 0; k < n_points ; k++ ) {
				Map<Vector3d> point(contact.contact_points[k].data);
				// calculate point jacobian 
				Jc_point.setZero();
				CalcPointJacobian(rbdl_model, Q, contact.body_id, point, Jc_point, false);
				Jc_reserved.middleRows(n_Jc_rows, 3) = Jc_point;
				// calculate gamma = JcDot*QDot
				JcDotQDot_reserved.segment<3>(n_Jc_rows) = CalcPointAcceleration(rbdl_model, Q, QDot, QDot_0, contact.body_id, point, false);
				// move index
				n_Jc_rows += 3;
			}
		}

	if (n_Jc_rows == 0) {
		// no contacts ... maybe robot has wings.
		// simply calculate inverse dynamics
		InverseDynamics(rbdl_model, Q, QDot, QDDot, tau);

		if(log(DEBUG)) {
			log() << "Simple WBC: QDot_size = " << rbdl_model.qdot_size << " no contacts." << endlog();
		}
	}
	else {
		// TODO preallocation? or real time allocator? some operation still reques memory allocation
		//
		// submatrices of preallocated objects
		Map<MatrixXd,0,OuterStride<> > Jc(Jc_reserved.data(), n_Jc_rows, rbdl_model.qdot_size, OuterStride<>(Jc_reserved.outerStride()));
		Map<VectorXd> JcDotQDot(JcDotQDot_reserved.data(), n_Jc_rows);
		Map<VectorXd> lambda(lambda_reserved.data(), n_Jc_rows);
		
		// compute SVD decomposition
		Jc_decomposition.setThreshold(tolerance);
		Jc_decomposition.compute(Jc, ComputeThinU | ComputeThinV);  // reallocate on resize!!!

		// now normalize QDDot to conform contact constarins: Jc*QDDot + JcDot*QDot = 0
		QDDot -= Jc_decomposition.solve(JcDotQDot + Jc*QDDot);  // allocate for temporary!!!  
		// solve inverse dynamic: tau_full = H*QDDot + N(Q,QDot)
		InverseDynamics(rbdl_model, Q, QDot, QDDot, tau_full);

		//
		// lambda -> tau
		// Minimize lambda, then calculate tau
		//
		JcT6_decomposition.compute(Jc.transpose().topRows<6>(), ComputeThinU | ComputeThinV); // reallocate on resize

		// calculate lambda that zeros first 6 elements of tau_full --- uncontrolled DOFs.
		// Ns*Jc'*lambda = Ns*(H*QDDot + N(Q,QDDot)), where Ns selects 6 first rows.
		lambda = JcT6_decomposition.solve(tau_full.head<6>());

		// calculate coresponding tau
		// tau = H*QDDot + N(Q,QDDot) - Jc'*lambda
		tau = tau_full - Jc.transpose()*lambda;  // allocate for temporary!!!

		// check if motion eqution is really fullfilled
		if ( ! tau.head<6>().isZero(tolerance) ) {
			log(WARN) << "Unable to achive control goal! tau_full residual: " << tau.head<6>().transpose() << endlog();
		}

		// FINISED!
		if(log(DEBUG)) {
			log(DEBUG) << "Simple WBC: QDot_size = " << rbdl_model.qdot_size << " Jc_rows = " << n_Jc_rows << " Jc_rank = " << Jc_decomposition.rank() << endlog();
			// calulate dignostics
			// this cod is really not real-time
			MatrixXd H(rbdl_model.qdot_size, rbdl_model.qdot_size); 
			VectorXd N(rbdl_model.qdot_size); 
			VectorXd err(rbdl_model.qdot_size); 
			VectorXd cerr(n_Jc_rows); 
			// model matirces 
			H.setZero(); N.setZero();
			CompositeRigidBodyAlgorithm(rbdl_model, Q, H, false);
			NonlinearEffects(rbdl_model, Q, QDot, N);
			// motion equation residual
			err = H*QDDot + N - tau - Jc.transpose()*lambda;
			log(DEBUG) << "Motion equation residual: " << err.norm() << endlog();
			// velocity and acceleration constraints residual
			cerr = Jc*QDot;
			log(DEBUG) << "Velocity constraints residual: " << cerr.norm() << endlog();
			cerr = Jc*QDDot + JcDotQDot;
			log(DEBUG) << "Acceleration constraints residual: " << cerr.norm() << endlog();
			log(DEBUG) << "Q = [\n" << Q << " ];\nQDot = [\n" << QDot << " ];\nQDDot = [\n" << QDDot << " ];" << endlog();
			log(DEBUG) << "tau = [\n" << tau << " ];\ntau_full = [\n" << tau_full + tau << " ];\nlambda = [\n"  << lambda  << " ];" << endlog();
			log(DEBUG) << "Jc = [\n" << Jc << "]; " <<  endlog();
			log(DEBUG) << "H = [\n" << H << " ];\nN = [ " << N << " ];" <<  endlog();
		}
	}

}

void DynamicsInvSimple::publishStateToPorts()
{
	// joint state
	joints_accel.header.stamp = ros::Time::now();
	joints_accel.position = joints_ref.position;
	joints_accel.velocity = joints_ref.velocity;
	for (int i = 6; i < rbdl_model.qdot_size; i++) {
		joints_accel.acceleration[joint_index[i]] = QDDot[i];
		joints_accel.effort[joint_index[i]] = tau[i];
	}
	// publish
	out_joints_accel_port.write(joints_accel);

	// wrenches and base are calculated and published in world frame
	int point_index = 0;
	wrenches.header.stamp = joints_accel.header.stamp;
	// wrenches already has the same fields' order as legs array
	wrenches.frame.clear(); 
	wrenches.twist.clear(); 
	wrenches.wrench.clear(); 
	base.header.stamp = joints_accel.header.stamp;
	base.wrench.clear();
	base.wrench.push_back(KDL::Wrench::Zero());
	balance.header.stamp = joints_accel.header.stamp;
	balance.support_points.clear();
	for( ContactState& contact : contacts ) {
		// frame positions
		KDL::Frame T;
		Map< Matrix<double,3,3,Eigen::RowMajor> >(T.M.data) = CalcBodyWorldOrientation(rbdl_model, Q, contact.body_id, false);
		Map<Vector3d>(T.p.data) = CalcBodyToBaseCoordinates(rbdl_model, Q, contact.body_id, Vector3d::Zero(), false);
		// wrenches.frame.emplace_back(base.frame[0].Inverse()*T); // move to base_link frame
		wrenches.frame.emplace_back(T);

		// frame velocities
		// RBDL returns pose twist in world frame, convert to srew twist in base_link frame
		SpatialVector_t twist = CalcPointVelocity6D(rbdl_model, Q, QDDot, contact.body_id, Vector3d::Zero(), false);
		// add twist to message
		wrenches.twist.emplace_back(KDL::Vector(twist[3], twist[4], twist[5]), KDL::Vector(twist[0], twist[1], twist[2])); // vel, rot  (field order in rbdl is reversed)
		wrenches.twist.back() = wrenches.twist.back().RefPoint(-T.p); // move reference point to frame origin
		// wrenches.twist.back() = base.frame[0].Inverse(wrenches.twist.back() - base.twist[0]); // subtract base_link velocity and change to base_link_frame

		// wrenches
		wrenches.wrench.emplace_back(KDL::Wrench::Zero());
		KDL::Wrench& wrench = wrenches.wrench.back();
		if (contact.is_active) {
			int n_points = std::min<unsigned int>(contact.contact_points.size(), 3); // no more, then 3 point per contact
			for ( int k = 0; k < n_points ; k++ ) {
				KDL::Vector point_world;
				// calculate point postion in world coordinates
				Map<Vector3d>(point_world.data)  = CalcBodyToBaseCoordinates(rbdl_model, Q, contact.body_id, Map<Vector3d>(contact.contact_points[k].data), false);
				// add to support points list: only first point
				if (k == 0) balance.support_points.push_back(point_world);
				// calculate wrench in world coordinates
				KDL::Vector force = KDL::Vector(lambda_reserved[point_index], lambda_reserved[point_index+1], lambda_reserved[point_index+2]);
				wrench.force += force;
				wrench.torque += point_world * force;

				point_index += 3;
			}
			base.wrench[0] += wrench;
			//wrench = base.frame[0].Inverse(wrench); // convert to base_link frame
		}
	}
	// publish
	out_wrenches_port.write(wrenches);
	out_base_port.write(base);

	// balance information
	// calculte center of mass
	{ 
		Vector3_t CoM;
		Utils::CalcCenterOfMass(rbdl_model, Q, QDot, nullptr, balance.mass, CoM, nullptr, nullptr, nullptr, nullptr, false);
		//Utils::CalcCenterOfMass(rbdl_model, Q, QDot, balance.mass, CoM, nullptr, nullptr, false);
		Map<Vector3d>(balance.CoM.data) = CoM;
	}
	// calculate CoP (actual center of pressure of contact forces)
	if (base.wrench[0].force.z() > 0) {
		balance.CoP[0] = - base.wrench[0].torque.y() / base.wrench[0].force.z();
		balance.CoP[1] =   base.wrench[0].torque.x() / base.wrench[0].force.z();
		balance.CoP[2] =   0.0;
	}
	else balance.CoP = balance.CoM;
	// calculate ZMP (desired center of pressure)
	KDL::Wrench wrench;
	// residual wrench
	Map<Vector3d>(wrench.force.data) = tau.head<3>();
	Map<Vector3d>(wrench.torque.data) = tau.segment<3>(3);
	wrench.torque += base.frame[0].p * wrench.force;
	// and wrench of reaction forces
	wrench += base.wrench[0];
	// calculate ZMP location
	if (wrench.force.z() > 0) {
		// calculate ZMP
		balance.ZMP[0] = - wrench.torque.y() / wrench.force.z();
		balance.ZMP[1] =   wrench.torque.x() / wrench.force.z();
		balance.ZMP[2] =   0.0;
	}
	else balance.ZMP = balance.CoM;
	// publish
	out_balance_port.write(balance);
}


void DynamicsInvSimple::updateHook()
{
	if (checkPorts()) {
		// update state variables: Q, QDot, QDot, contacts
		updateStateFromPortsBuffers(); 
		// inverse dynamics: Q, QDot, QDDot, contacts -> QDDot, Jc_reserved, JcDotQDot_reserved, tau, lambda_reserved
		inverseDynamic();
		//TODO sanity checks
		// publish results
		publishStateToPorts();
	}
}

void DynamicsInvSimple::stopHook() 
{
	this->log(INFO) << "DynamicsInvSimple is stoped." <<endlog();
}

void DynamicsInvSimple::cleanupHook() 
{
	// delete model
	rbdl_model = Model();

	this->log(INFO) << "DynamicsInvSimple is cleaned up." <<endlog();
}

} // namespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(sweetie_bot::motion::DynamicsInvSimple)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::DynamicsInvSimple)
