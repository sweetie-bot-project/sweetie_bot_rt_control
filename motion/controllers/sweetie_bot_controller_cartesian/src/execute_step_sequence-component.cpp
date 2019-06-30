#include "execute_step_sequence-component.hpp"

#include <rtt/Component.hpp>
#include <ros/time.h>

#include <sweetie_bot_orocos_misc/get_subservice_by_type.hpp>
#include <sweetie_bot_orocos_misc/message_checks.hpp>
#include <sweetie_bot_orocos_misc/math.hpp>

using namespace RTT;


inline std::ostream& operator<<(std::ostream& s, const KDL::Vector& v) 
{
	s << "[" << v.x() << " " << v.y() << " " << v.z() << " ]";
	return s;
}

inline std::ostream& operator<<(std::ostream& s, const KDL::Twist& v) 
{
	s << "[ rot = " << v.rot << ", vel = " << v.vel << " ]";
	return s;
}
inline std::ostream& operator<<(std::ostream& s, const KDL::Rotation& R) 
{
	s << std::endl;
	s << R(0,0) << " " << R(0,1) << " " << R(0,2) << std::endl;
	s << R(1,0) << " " << R(1,1) << " " << R(1,2) << std::endl;
	s << R(2,0) << " " << R(2,1) << " " << R(2,2) << std::endl;
	return s;
}

namespace sweetie_bot {
namespace motion {
namespace controller {

ExecuteStepSequence::ExecuteStepSequence(std::string const& name)  : 
	TaskContext(name),
	log(logger::categoryFromComponentName(name)),
	action_server(this->provides()), // action server client
	poseToJointStatePublish("poseToJointStatePublish", this->engine()) // operation caller
{
	// ports input 
	this->addEventPort("sync", sync_port)
		.doc("Timer syncronization event. This event triggers controller execution cycle.");
	this->addPort("in_limbs_fixed", in_limbs_port)
		.doc("Robot limbs postions. They are used to check path tolerance.");
	this->addPort("in_base", in_base_port)
		.doc("Robot base link pose in world frame. It is used to check path tolerance.");
	//this->addPort("in_balance", in_balance_port)
	//	.doc("Information about robot balance.");

	// ports input 
	this->addPort("out_base_ref", out_base_ref_port)
		.doc("Base next position to achive target given current robot state. It is computed by component each control cycle.");
	this->addPort("out_limbs_ref", out_limbs_ref_port)
		.doc("Limbs next position to achive target given current robot state. It is computed by component each control cycle. ");
	this->addPort("out_supports", out_supports_port)
		.doc("Active contact list.");

	// properties
	this->addProperty("period", period)
		.doc("Discretization period (s)");
	// operations: provided
	// operations: required
	this->requires()->addOperationCaller(poseToJointStatePublish); // kinematics service

	// Service: reqires
	robot_model = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));

	// action server hook registration
	action_server.setGoalHook(boost::bind(&ExecuteStepSequence::newGoalHook, this, _1));
	action_server.setCancelHook(boost::bind(&ExecuteStepSequence::cancelGoalHook, this));

	log(INFO) << "ExecuteStepSequence is constructed!" << endlog();
}
#
bool ExecuteStepSequence::dataOnPortHook( RTT::base::PortInterface* portInterface ) 
{
	// Process EventPorts messages callbacks if component is in configured state,
	// so actionlib hooks works even if component is stopped.
	//
	// WARNING: works only in OROCOS 2.9 !!!
	//
    return this->isConfigured();
}

bool ExecuteStepSequence::configureHook()
{
	// INITIALIZATION
	resource_client = getSubServiceByType<ResourceClientInterface>(this->provides().get());
	if (!resource_client) {
		log(ERROR) << "ResourceClient plugin is not loaded." << endlog();
		return false;
	}
	resource_client->setResourceChangeHook(boost::bind(&ExecuteStepSequence::resourceChangedHook, this));
	resource_client->setStopOperationalHook(boost::bind(&ExecuteStepSequence::stopOperationalHook, this));

	// Start action server: publish feedback
	if (!action_server.start(true)) {
		log(ERROR) << "Unable to start action_server." << endlog();
		return false;
	}

	// check if RobotModel Service presents
	if (!robot_model->ready() || !robot_model->isConfigured()) {
		log(ERROR) << "RobotModel service is not ready." << endlog();
		return false;
	}

	// check poseToJointStatePublish caller
	if (poseToJointStatePublish.ready()) {
		log(INFO) << "poseToJointStatePublish operation is connected. Syncronous IK interface is used." << endlog();
	}
	else {
		log(INFO) << "poseToJointStatePublish operation is disconnected. Asyncronous IK interface via port is used." << endlog();
	}

	// allocate persistent buffers
	base_ref.name.resize(1); base_ref.name[0] = "base_link";
	base_ref.frame.resize(1);
	base_ref.twist.resize(1);

	log(INFO) << "ExecuteStepSequence is configured !" << endlog();
	return true;
}


bool ExecuteStepSequence::readPorts() 
{
	// check if limb data is available
	if (in_limbs_port.read(limbs_full, true) == NoData) {
		log(WARN) << "Limbs pose is unknown (in_limbs_port)." << endlog();
		return false;
	}
	if (!isValidRigidBodyStateNameFrame(limbs_full)) {
		log(WARN) << "Incorrect message on  in_limbs_port." << endlog();
		return false;
	}
	// check if base_link pose data is available
	if (in_base_port.read(base, true) == NoData) {
		log(WARN) << "Base pose is unknown (in_base_port)." << endlog();
		return false;
	}
	if (!isValidRigidBodyStateNameFrame(base, 1)) {
		log(WARN) << "Incorrect message on in_base_port." << endlog();
		return false;
	}
	// everything is ok
	return true;
}

void ExecuteStepSequence::rejectPending(const std::string& where, const std::string& msg, int code)
{
	log(code == Result::SUCCESSFUL ? INFO : ERROR) << where << ": reject pending goal: " << msg << endlog();
	goal_result.error_code = code;
	goal_result.error_string = msg;
	action_server.rejectPending(goal_result, goal_result.error_string);	
}

void ExecuteStepSequence::abortActive(const std::string& where, const std::string& msg, int code)
{
	log(code == Result::SUCCESSFUL ? INFO : ERROR) << where << ": abort active goal: " << msg << endlog();
	goal_result.error_code = code;
	goal_result.error_string = msg;
	action_server.abortActive(goal_result, goal_result.error_string);	
}

void ExecuteStepSequence::newGoalHook(const Goal& pending_goal) 
{
	log(INFO) << "newGoalHook: new pending goal." << endlog();

	// REJECT if another step sequence is being executed
	if ( ! resource_client->isNonOperational() ) {
		rejectPending("newGoalHook", "active goal is present",  Result::UNABLE_TO_APPEND);
		return;
	}

	// check goal consistency and construct trajectory cache
	try {
		trajectory = std::make_shared<CartesianTrajectoryCache>(action_server.getPendingGoal(), robot_model, period);
	}
	catch (std::exception& e) {
		rejectPending("newGoalHook", e.what(),  Result::UNABLE_TO_APPEND);
		return;
	}
	// get current robot pose
	if (!readPorts()) {
		rejectPending("newGoalHook", "unable to check tolerance",  Result::INTERNAL_ERROR);
		return;
	}
	// check tolerance. Note, we check only limbs positions.
	int invalid_limb_index = trajectory->checkPathToleranceFullpose(limbs_full);
	if (invalid_limb_index >= 0) {
		rejectPending("newGoalHook", "path tolerance is violated for " + limbs_full.name[invalid_limb_index] ,  Result::TOLERANCE_VIOLATED);
		return;
	}

	// we have correct goal so perform resource request
	log(INFO) << "newGoalHook: request resources." << endlog();
	resource_client->resourceChangeRequest(trajectory->getRequiredChains());

	// prepare output buffers
	trajectory->prepareEndEffectorStateBuffer(limbs);
	trajectory->prepareSupportStateBuffer(supports);

	// start if component is not running
	start();
}

bool ExecuteStepSequence::resourceChangedHook()
{
	bool has_resources;
	bool success;
	// there are only two possibilies: there is pending goal or there is not.
	if (action_server.isPending() && trajectory) {
		// we have valid pending goal
		has_resources = resource_client->hasResources(trajectory->getRequiredChains());
		if (has_resources) {
			// accept pending goal
			goal_result.error_code = Result::SUCCESSFUL;
			goal_result.error_string = "";
			success = action_server.acceptPending(goal_result);
			// setup goal
			if (success) {
				log(INFO) << "resourceChangedHook: pending goal is accepted."  << endlog();
				return true;
			}
			else {
				log(WARN) << "resourceChangedHook: unable to activate pending goal." << endlog();
			}
		}
		else {
			// we do not have all resource -> reject pending
			rejectPending("resourceChangedHook", "not enought resources.", Result::INTERNAL_ERROR);
		}
	}
	// see if we can still pursue active goal
	if (action_server.isActive() && trajectory) {
		has_resources = resource_client->hasResources(trajectory->getRequiredChains());
		if (has_resources) {
			// contine active goal
			log(INFO) << "resourceChangedHook: continue pursue active goal." << endlog();
			return true;
		}
		else {
			// abort active goal
			goal_result.error_code = Result::INTERNAL_ERROR;
			goal_result.error_string = "Lost necessary resources.";
			action_server.abortActive(goal_result);
			log(INFO) << "resourceChangedHook: not enough resources, active goal is aborted." << endlog();
			return false;
		}
	}
	log(WARN) << "resourceChangedHook: abnomal state: isActive = " << (int) action_server.isActive() << " goal_active = " << (int) (trajectory != 0) 
		<<  (int) action_server.isPending() << " goal_pending = " << endlog();
	return false;
}

void ExecuteStepSequence::cancelGoalHook() 
{
	log(INFO) << "cancelGoalHoook: abort active goal." << endlog();
	// cancel active goal (or do nothing)
	goal_result.error_code = Result::SUCCESSFUL;
	goal_result.error_string = "Canceled by user request.";
	action_server.cancelActive(goal_result);
	// stop
	resource_client->stopOperational();
}

bool ExecuteStepSequence::startHook()
{
	// data samples
	in_limbs_port.getDataSample(limbs_full);
	in_base_port.getDataSample(base);

	// now update hook will be periodically executed
	log(INFO) << "ExecuteStepSequence is started !" << endlog();
	return true;
}


void ExecuteStepSequence::updateHook()
{
	// check messages on resource_assigment port
	resource_client->step();

	// syncronize with sync messages
	{
		RTT::os::Timer::TimerId unused;
		if (sync_port.read(unused) != NewData) return;
	}

	if (this->resource_client->isOperational() && this->trajectory) {
		// read pose of robot
		readPorts();
		// check tolerance
		int invalid_limb_index = trajectory->checkPathToleranceFullpose(limbs_full);
		if (invalid_limb_index >= 0) {
			abortActive("updateHook:", "path tolerance violated for " + limbs_full.name[invalid_limb_index], Result::TOLERANCE_VIOLATED);
			resource_client->stopOperational();
			return;
		}
		// get desired pose	
		bool base_pose_available = trajectory->getBaseState(base_ref);
		trajectory->getEndEffectorState(limbs);
		trajectory->getSupportState(supports);
		// publish new reference position
		ros::Time stamp = ros::Time::now();
		base_ref.header.stamp = stamp;
		limbs.header.stamp = stamp;
		// pass result to kinematics
		if (poseToJointStatePublish.ready()) {
			// syncronous interface
			bool ik_success = poseToJointStatePublish(limbs);
			if (!ik_success) {
				abortActive("updateHook:", "IK failed at t = " + std::to_string(trajectory->getTime()), Result::INTERNAL_ERROR);
				resource_client->stopOperational();
				return;
			}
		} 
		else {
			// async interface: assume always success
			out_limbs_ref_port.write(limbs);
		}
		if (base_pose_available) out_base_ref_port.write(base_ref);
		out_supports_port.write(supports);

		// move time marker forward
		if (!trajectory->step()) {
			// execution is finished successfully
			log(INFO) << "Step sequence is executed successfully." << endlog();
			goal_result.error_code = Result::SUCCESSFUL;
			goal_result.error_string = "";
			action_server.succeedActive(goal_result, goal_result.error_string);	
			resource_client->stopOperational();
		}
	}
}

void ExecuteStepSequence::stopOperationalHook() {
	trajectory.reset();
	// stop
	this->stop();
}

void ExecuteStepSequence::stopHook() 
{
	// TODO check if this realization is thread safe in presence of EventPorts triggered in configure state.
	// detect if stop() is called externaly and not from stopOperationalHook()
	if (resource_client->isOperational()) { // prevent recursion 
		abortActive("stopHook",  "stopOperational is triggered by external cause.", Result::SUCCESSFUL);
		rejectPending("stopHook",  "stopOperational is triggered by external cause.", Result::SUCCESSFUL);
		resource_client->stopOperational();
	}
	log(INFO) << "ExecuteStepSequence is stopped!" << endlog();
}

void ExecuteStepSequence::cleanupHook() 
{
	resource_client = 0; 
	//action_server.shutdown(); 
	log(INFO) << "ExecuteStepSequence cleaning up !" << endlog();
}


} // namespace controller
} // namespace motion
} // namespace sweetie_bot


/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ExecuteStepSequence)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::controller::ExecuteStepSequence)
