#include "dynamics_inv_simple-component.hpp"

#include <algorithm>

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

DynamicsInvSimple::DynamicsInvSimple(string const& name) : 
	TaskContext(name, PreOperational),
	log(logger::categoryFromComponentName(name))
{
	// input ports
	this->addPort( "in_joints_sorted", in_joints_port )
		.doc( "Full robot pose in joint space (without information about floating base). Fields `position` and `velocity` must present." );
	this->addEventPort( "in_base", in_base_port )
		.doc( "Robot base position and velocity. This port triggers calulations when first in control cycle is received.");
	this->addPort( "in_supports_sorted", in_supports_port )
		.doc( "Active contact list." );
	this->addPort( "sync_step", sync_port )
		.doc( "Syncronization message indicates start of new control cycle." );

	// output ports
	this->addPort( "out_joints_accel_sorted", out_joints_accel_port )
		.doc( "Robot pose in joint space with drives effort and accelerations." );
	//this->addPort( "out_supports_sorted", out_supports_port )
	//	.doc( "Weight distribution beetween support legs." );
	this->addPort( "out_wrenches_fixed", out_wrenches_port )
		.doc( "Reaction forces for each contact." );

	// properties
	this->addProperty( "robot_description", robot_description )
		.doc( "Simplified robot URDF (without fictive joints and massless links).");
	this->addProperty( "legs", legs )
		.doc("List of end effectors which can be in contact and be affected by external forces (Kinematic chains names, legs).");
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
	// floating base is modelled with to joints. Check it!
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
	joints_accel.name = robot_model->listJoints("");
	n_fullpose_joints = joints_accel.name.size();

	// construct index
	joint_index.assign(rbdl_model.qdot_size, -1); 
	//rbdl_index.assign(n_joints, -1); 

	// now check model structure: all joints are 1 DOF and present in RobotModel
	KDL::Tree kdl_tree = robot_model->getKDLTree();
	// now iterate over bodies and get coresponding induces
	for(int id = 3; id < rbdl_model.mBodies.size(); id++) {
		string body_name = rbdl_model.GetBodyName(id);
		// find it in KDL
		auto it = kdl_tree.getSegment(body_name);
		if (it == kdl_tree.getSegments().end()) {
			log(ERROR) << "Unknown link " << body_name << " with id = " << id << endlog();
			return false;
		}
		// all KDL joints has 1 DOF
		string joint_name = it->second.segment.getJoint().getName();
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
	// contacts
	// reserve memory for contact jacobian: max tree points per contact
	Jc_reserved.resize(9*contacts.size(), rbdl_model.qdot_size); //TODO RT dynamic allocation
	JcDotQDot_reserved.resize(9*contacts.size()); //TODO RT dynamic allocation
	lambda_reserved.resize(9*contacts.size());
	// port buffers
	joints_accel.position.resize(n_fullpose_joints);
	joints_accel.velocity.resize(n_fullpose_joints);
	joints_accel.acceleration.assign(n_fullpose_joints, 0.0);
	joints_accel.effort.assign(n_fullpose_joints, 0.0);
	wrenches.header.frame_id = "base_link";
	wrenches.name = legs;
	wrenches.wrench.resize(wrenches.name.size());
		
	// data samples
	out_joints_accel_port.setDataSample(joints_accel);
	in_supports_port.getDataSample(supports);
	//out_supports_port.setDataSample(supports);
	out_wrenches_port.setDataSample(wrenches);

	this->log(INFO) << "DynamicsInvSimple is configured." <<endlog();
	return true;
}

bool DynamicsInvSimple::startHook()
{
	// reset state
	QDot.setZero();
	joints_accel.acceleration.assign(n_fullpose_joints, 0.0);
	// clear 
	in_joints_port.getDataSample(joints);
	in_supports_port.getDataSample(supports);
	in_base_port.getDataSample(base);
	this->log(INFO) << "DynamicsInvSimple is started." <<endlog();
	return true;
}

// read ports and return true if torque should be calculated
bool DynamicsInvSimple::checkPorts() 
{
	bool trigger_update = true;
	
	// joint state
	in_joints_port.read(joints, false);
	if (!isValidJointStatePosVel(joints, n_fullpose_joints)) {
		log(ERROR) << "Invalid JointState message on in_joints_sorted port." << endlog();	
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
	if (in_base_port.read(base, false) == NewData && sync_port.read(tmp) == NewData) {
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
		Q[i] = joints.position[joint_index[i]];
		// acceleration calculation
		QDDot[i] = (joints.velocity[joint_index[i]] - QDot[i]) / period;
		// velocity
		QDot[i] = joints.velocity[joint_index[i]];
	}
	// base
	Q.head<3>() = Map<Vector3d>(base.frame[0].p.data);
	Eigen::Quaternion<double> rot;
	tf::quaternionKDLToEigen(base.frame[0].M, rot);
	rot.normalize();
	Q[3] = rot.x(); Q[4] = rot.y(); Q[5] = rot.z(); Q[rbdl_model.q_size-1] = rot.w();
	// base twist
	QDot.head<3>() = Map<Vector3d>(base.twist[0].vel.data);
	QDot.segment<3>(3) = Map<Vector3d>(base.twist[0].rot.data);
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
	// It is only inertial frame in system.
	
	// TODO preallocation?
	VectorN_t QDDot_0;
	QDDot_0.setZero(rbdl_model.qdot_size);
	MatrixXd Jc_point(3, rbdl_model.qdot_size);

	// update rbdl model: only psition is necessary to calculate jacobian
	UpdateKinematics(rbdl_model, Q, QDot, QDDot_0); // set acceleration to zero, to calculate dotJc using CalcPointAcceleration()

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
				JcDotQDot_reserved.segment<3>(n_Jc_rows) = CalcPointAcceleration(rbdl_model, Q, QDot, QDDot_0, contact.body_id, point, false);
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
		// submatrices of preallocated objects
		Map<MatrixXd,0,OuterStride<> > Jc(Jc_reserved.data(), n_Jc_rows, rbdl_model.qdot_size, OuterStride<>(Jc_reserved.outerStride()));
		Map<VectorXd> JcDotQDot(JcDotQDot_reserved.data(), n_Jc_rows);
		Map<VectorXd> lambda(lambda_reserved.data(), n_Jc_rows);
		// TODO preallocation?
		MatrixXd JcPInv(rbdl_model.qdot_size, n_Jc_rows); 
		//MatrixXd Nc(rbdl_model.qdot_size, rbdl_model.qdot_size); 
		//MatrixXd NcS(rbdl_model.qdot_size, rbdl_model.qdot_size); 
		VectorXd tau_full(rbdl_model.qdot_size); 
		
		// compute SVD decomposition
		// TODO preallocation? or real time allocator?
		// TODO resize only on size change
		Eigen::JacobiSVD<MatrixXd> JcSVD(Jc.rows(), Jc.cols(), ComputeThinU | ComputeThinV);
		JcSVD.setThreshold(tolerance);
		JcSVD.compute(Jc);
		JcPInv = JcSVD.solve(MatrixXd::Identity(n_Jc_rows, n_Jc_rows));

		// now normalize QDDot to conform contact constarins: Jc*QDDot + JcDot*QDot = 0
		QDDot -= JcPInv*(JcDotQDot + Jc*QDDot);

		// solve inverse dynamic: tau_full = H*QDDot + N(Q,QDot)
		InverseDynamics(rbdl_model, Q, QDot, QDDot, tau_full);
		
		// calculate null projector
		// Nc.setIdentity(rbdl_model.qdot_size, rbdl_model.qdot_size);
		// Nc -= JcPInv*Jc; 
	
		// apply null projector and calcualte tau: Nc*S*tau = Nc*(H*QDDot + N(Q,QDot))
		// S matrix selects only actuated joints
		// NcS.setZero();
		// NcS.bottomRightCorner(rbdl_model.qdot_size-6,rbdl_model.qdot_size-6) = Nc.bottomRightCorner(rbdl_model.qdot_size-6,rbdl_model.qdot_size-6);

		//	
		// tau -> lambda
		// Minimize tau, then calculate lambda
		
		/*// TODO preallocation? or real time allocator?
		// TODO resize only on size change
		//Eigen::LDLT<MatrixXd> NcSLDLT(rbdl_model.qdot_size);
		Eigen::JacobiSVD<MatrixXd> NcSLDLT(rbdl_model.qdot_size, rbdl_model.qdot_size, ComputeThinU | ComputeThinV);
		NcSLDLT.compute(NcS);
		tau = NcSLDLT.solve(Nc*tau_full);

		//now calculate reaction forces: JcT*lambda = H*QDDot + N(Q,QDDot) - tau
		tau_full -= tau;
		lambda = JcPInv.transpose()*tau_full;*/

		//
		// lambda -> tau
		// Minimize lambda, then calculate tau
		//
		// calculate lambda which zeros first 5 elements of tau_full --- uncontrolled DOFs.
		// Ns*Jc'*lambda = Ns*(H*QDDot + N(Q,QDDot)), Ns selects 6 first rows.
		// tau.head<6>() = tau_full.head<6>(); 
		// tau.tail(rbdl_model.qdot_size-6).setZero();
		lambda = JcPInv.transpose().leftCols<6>() * tau_full.head<6>(); // not optimal computations TODO: use submatrices

		// calculate coresponding tau
		// tau = H*QDDot + N(Q,QDDot) - Jc'*lambda
		tau = tau_full - Jc.transpose()*lambda;

		// FINISED!
		if(log(DEBUG)) {
			log(DEBUG) << "Simple WBC: QDot_size = " << rbdl_model.qdot_size << " Jc_rows = " << n_Jc_rows << " Jc_rank = " << JcSVD.rank() << endlog();
			//calulate dignostics
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
			//log(DEBUG) << "Q = " << std::endl << Q << "\nQDot = " << std::endl << QDot << "\nNc*QDot = " << std::endl << Nc*QDot << "\nQDDot = " << std::endl << QDDot << endlog();
			log(DEBUG) << "Q = " << std::endl << Q << "\nQDot = " << std::endl << QDot << "\nQDDot = " << std::endl << QDDot << endlog();
			log(DEBUG) << "tau = " << std::endl << tau << "\ntau_full = " << std::endl << tau_full + tau << "\nlambda = " << std::endl << lambda  << endlog();
			//log(DEBUG) << "Jc = " << std::endl << Jc << "\nJcPinv = " << JcPInv<< "\nNcS = " << NcS <<  endlog();
			log(DEBUG) << "Jc = " << std::endl << Jc << "\nJcPinv = " << JcPInv <<  endlog();
			log(DEBUG) << "H = " << std::endl << H << "\nN = " << N <<  endlog();
		}
	}

}

void DynamicsInvSimple::publishStateToPorts()
{
	// joint state
	joints_accel.header.stamp = ros::Time::now();
	joints_accel.position = joints.position;
	joints_accel.velocity = joints.velocity;
	for (int i = 6; i < rbdl_model.qdot_size; i++) {
		joints_accel.acceleration[joint_index[i]] = QDDot[i];
		joints_accel.effort[joint_index[i]] = tau[i];
	}
	// publish
	out_joints_accel_port.write(joints_accel);

	// wrenches calculations is performed in world coordinates
	int point_index = 0;
	wrenches.wrench.clear(); // has the same fields' order as legs array
	//double reaction_force_z_total = 0.0;
	for( ContactState& contact : contacts ) {
		KDL::Wrench wrench = KDL::Wrench::Zero();
		//double reaction_force_z = 0.0;
		if (contact.is_active) {
			int n_points = std::min<unsigned int>(contact.contact_points.size(), 3); // no more, then 3 point per contact
			for ( int k = 0; k < n_points ; k++ ) {
				KDL::Vector point_world;
				// calculate point postion in world coordinates
				Map<Vector3d>(point_world.data)  = CalcBodyToBaseCoordinates(rbdl_model, Q, contact.body_id, Map<Vector3d>(contact.contact_points[k].data), false);
				// calculate wrench in world coordinates
				wrench.force += KDL::Vector(lambda_reserved[point_index], lambda_reserved[point_index+1], lambda_reserved[point_index+2]);
				wrench.torque += point_world * wrench.force;
				// CoP calculation 
				// PROBLEM: in case of general contact model position of CoP cannot be storedin support structure
				// so out_support_port is removed. Reaction forces wrenches should be used to calculate CoP position.
				// TODO remove commented out code.
				//reaction_force_z += wrench.force.z();	

				point_index += 3;
			}
		}
		// assign to RigidBodyState in base_link coordinates
		wrenches.wrench.push_back(base.frame[0].Inverse(wrench));
		// assign support coefficients
		// supports.support[contact.index] = reaction_force_z;
	}
	// publish
	out_wrenches_port.write(wrenches);

	// supports
	//for( ContactState& contact : contacts ) supports.support[contact.index] /= reaction_force_z_total;
	//out_supports_port.write(supports);
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
	// buffer cleanup?
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
