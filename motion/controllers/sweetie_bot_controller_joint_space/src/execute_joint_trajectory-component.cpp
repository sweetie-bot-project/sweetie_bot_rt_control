#include "execute_joint_trajectory-component.hpp"

#include <rtt/Component.hpp>

#include <sweetie_bot_orocos_misc/joint_state_check.hpp>

using namespace RTT;

namespace sweetie_bot {
namespace motion {
namespace controller {


ExecuteJointTrajectory::ExecuteJointTrajectory(std::string const& name) : 
	ExecuteJointTrajectoryBase(name),
	action_server(this->provides())
{
	// action server hook registration
	action_server.setGoalHook(boost::bind(&ExecuteJointTrajectory::newGoalHook, this, _1));
	action_server.setCancelHook(boost::bind(&ExecuteJointTrajectory::cancelGoalHook, this));

    log(INFO) << "ExecuteJointTrajectory is constructed!" << endlog();
}

bool ExecuteJointTrajectory::configureHook()
{
	if (!ExecuteJointTrajectoryBase::configureHook()) return false;
	// ExecuteJointTrajectoryBase checked resource_client and robot_model. So we can now use them.
	
	resource_client->setResourceChangeHook(boost::bind(&ExecuteJointTrajectory::resourceChangedHook, this));
	resource_client->setStopOperationalHook(boost::bind(&ExecuteJointTrajectory::stopOperationalHook, this));

	// Start action server: publish feedback
	if (!action_server.start(true)) {
		log(ERROR) << "Unable to start action_server." << endlog();
		return false;
	}

	log(INFO) << "ExecuteJointTrajectory is configured !" << endlog();
	return true;
}

void ExecuteJointTrajectory::newGoalHook(const Goal& pending_goal) 
{
	log(INFO) << "newGoalHook: new pending goal." << endlog();

	// convert FollowJointTrajectoryGoal -> JointTrajectoryCache immediztelly
	try {
		goal_pending = std::make_shared<JointTrajectoryCache>(pending_goal, robot_model); // TODO possible throw on malformed  goal: any better solutions?
	}
	catch (std::invalid_argument& e) {
		log(ERROR) << "Error during goal analisys:" << e.what() << endlog();
		goal_result.error_code = Result::INVALID_GOAL;
		goal_result.error_string = e.what();
		action_server.rejectPending(goal_result);
		return;
	}
	//check start conditions
	in_joints_port.read(actual_fullpose, false);
	if (isValidJointStateNamePos(actual_fullpose, n_joints_fullpose)) {
		// check tolerance
		int invalid_joint_index = goal_pending->checkPathToleranceFullpose(actual_fullpose, 0.0);
		if (invalid_joint_index >= 0) {
			// unable to start: tolerance error
			goal_result.error_code = Result::PATH_TOLERANCE_VIOLATED;
			goal_result.error_string = "Joint pose error tolerance is violated at start: " + actual_fullpose.name[invalid_joint_index];
			action_server.rejectPending(goal_result);
			log(INFO) << "newGoalHook: " << goal_result.error_string << endlog();
			return;
		}
	}
	else {
		log(WARN) << "newGoalHook: unable to get actual pose, tolerance check bypassed. " << endlog();
	}
	// now we have valid pending_goal
	// but we will do nothing until get necessary resources
	log(INFO) << "newGoalHook: request resources." << endlog();
	resource_client->resourceChangeRequest(goal_pending->getRequiredChains());
	// start if component is not running
	start();
}

bool ExecuteJointTrajectory::resourceChangedHook()
{
	bool has_resources;
	bool success;
	// there are only two possibilies: there is pending goal or there is not.
	if (action_server.isPending() && goal_pending) {
		// we have valid pending goal
		has_resources = resource_client->hasResources(goal_pending->getRequiredChains());
		if (has_resources) {
			// active goal (if present) reult
			goal_result.error_code = Result::SUCCESSFUL;
			goal_result.error_string = "Preemted by new goal";
			// abort active if it presents
			success = action_server.acceptPending(goal_result);
			// setup goal
			if (success) {
				goal_active = goal_pending; 
				goal_pending.reset();
				// reset state
				time_from_start = 0;
				goal_active->prepareJointStateBuffer(ref_pose);
				goal_active->prepareJointStateBuffer(actual_pose);
				goal_active->prepareSupportStateBuffer(support_state);
				log(INFO) << "resourceChangedHook: pending goal is accepted."  << endlog();
				return true;
			}
			else {
				log(WARN) << "resourceChangedHook: unable to activate pending goal." << endlog();
			}
		}
		else {
			// we do not have all resource -> reject pending
			goal_result.error_code = Result::INVALID_JOINTS; //TODO really this result?
			goal_result.error_string = "Unable to acquire necessary resources.";
			action_server.rejectPending(goal_result);
			log(INFO) << "resourceChangedHook: not enough resources, pending goal is rejected." << endlog();
		}
	}
	// see if we can still pursue active goal
	if (action_server.isActive() && goal_active) {
		has_resources = resource_client->hasResources(goal_active->getRequiredChains());
		if (has_resources) {
			// contine active goal
			log(INFO) << "resourceChangedHook: continue pursue active goal." << endlog();
			return true;
		}
		else {
			// abort active goal
			goal_result.error_code = Result::INVALID_JOINTS; //TODO really this result?
			goal_result.error_string = "Lost necessary resources.";
			action_server.abortActive(goal_result);
			log(INFO) << "resourceChangedHook: not enough resources, active goal is aborted." << endlog();
			return false;
		}
	}
	log(WARN) << "resourceChangedHook: abnomal: isActive = " << (int) action_server.isActive() << " goal_active = " << (int) (goal_active != 0) 
		<<  (int) action_server.isPending() << " goal_pending = " << (int) (goal_pending != 0) << endlog();
	return false;
}

void ExecuteJointTrajectory::cancelGoalHook() 
{
	log(INFO) << "cancelGoalHoook: abort active goal." << endlog();
	// cancel active goal (or do nothing)
	goal_result.error_code = Result::SUCCESSFUL;
	goal_result.error_string = "Canceled by user request.";
	action_server.cancelActive(goal_result);
	// stop
	resource_client->stopOperational();
}

bool ExecuteJointTrajectory::startHook()
{
	ExecuteJointTrajectoryBase::startHook();	// clear sync port queue 
	log(INFO) << "ExecuteJointTrajectory is started !" << endlog();
	return true;
}

void ExecuteJointTrajectory::operationalHook(bool on_target)
{
	if(log(DEBUG)) {
		log() << "Execute goal t = " << time_from_start << " goal_time = " << goal_active->getGoalTime() << " ref [ ";
		for(auto it = ref_pose.position.begin(); it != ref_pose.position.end(); it++) log() << *it << ", ";
		log() << "] actual [ ";
		for(auto it = actual_pose.position.begin(); it != actual_pose.position.end(); it++) log() << *it << ", ";
		log() << "] supports: ";
		for(int i = 0; i < support_state.name.size(); i++) log() << support_state.name[i] << ": " << support_state.support[i] << ", ";
		log() << endlog();
	}
	// we in operational state and have valid active goal
	// check tolearnce and post feedback
	if (on_target) { 
		// trajectory execution is finished
		int invalid_joint_index = goal_active->checkGoalTolerance(actual_pose, ref_pose);
		if ( invalid_joint_index <  0 ) {
			// error in tolerance bounds
			goal_result.error_code = Result::SUCCESSFUL;
			goal_result.error_string = "Success.";
			action_server.succeedActive(goal_result);
			log(INFO) << "Goal is achived !" << endlog();
			// execution is finihed
			if (! resource_client->isPending()) resource_client->stopOperational();
		}
		else if ( time_from_start >= goal_active->getGoalTime() + setling_time ) {
			// error is not into tolerance bounds and setling_time is elasped
			goal_result.error_code = Result::GOAL_TOLERANCE_VIOLATED;
			goal_result.error_string = "Goal position error exceeds limit: " + actual_pose.name[invalid_joint_index];
			action_server.abortActive(goal_result);
			log(INFO) << "Goal is not achived: " << goal_result.error_string << endlog();
			// execution is finihed
			if (! resource_client->isPending()) resource_client->stopOperational();
		}
	}
	else {
		// check path tolerance
		int invalid_joint_index = goal_active->checkPathTolerance(actual_pose, ref_pose);
		if (invalid_joint_index >= 0) {
			goal_result.error_code = Result::PATH_TOLERANCE_VIOLATED;
			goal_result.error_string = "Path position error exceeds limit: " + actual_pose.name[invalid_joint_index];
			action_server.abortActive(goal_result);
			log(INFO) << "Goal is not achived: " << goal_result.error_string << endlog();
			// execution is finihed
			if (! resource_client->isPending()) resource_client->stopOperational();
			return;
		}
		// check time tolerance
		double time_overflow = goal_active->checkGoalTimeTolerance(time_from_start);
		if (time_overflow > 0) {
			goal_result.error_code = Result::GOAL_TOLERANCE_VIOLATED;
			goal_result.error_string = "Goal time error exceeds limit.";
			action_server.abortActive(goal_result);
			log(INFO) << "Goal is not achived: " << goal_result.error_string << endlog();
			// execution is finihed
			if (! resource_client->isPending()) resource_client->stopOperational();
			return;
		}
	}
}
			

void ExecuteJointTrajectory::stopOperationalHook() {
	// abort any active or pending goal: if a
	goal_active.reset();
	goal_pending.reset();
	// stop
	this->stop();
}

void ExecuteJointTrajectory::stopHook() 
{
	if (resource_client->isOperational()) { // prevent recursion
		goal_result.error_code = Result::SUCCESSFUL;
		goal_result.error_string = "stopOperational is triggered by external cause.";
		action_server.abortActive(goal_result);
		action_server.rejectPending(goal_result);
		resource_client->stopOperational();
		log(INFO) << "ExecuteJointTrajectory is stopped!" << endlog();
	}
}

void ExecuteJointTrajectory::cleanupHook() 
{

	resource_client = 0; 
	//action_server.shutdown(); 
	log(INFO) << "ExecuteJointTrajectory cleaning up !" << endlog();
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

ORO_CREATE_COMPONENT(sweetie_bot::motion::controller::ExecuteJointTrajectory)
