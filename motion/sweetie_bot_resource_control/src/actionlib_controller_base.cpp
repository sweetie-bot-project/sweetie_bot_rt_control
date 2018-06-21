#include "sweetie_bot_resource_control/actionlib_controller_base.hpp"

#include <rtt/Component.hpp>

#include "sweetie_bot_orocos_misc/get_subservice_by_type.hpp"

using namespace RTT;
using namespace std;

inline std::ostream& operator<<(std::ostream& s, const std::vector<std::string>& strings) 
{
	s << "[ ";
	for(auto it = strings.begin(); it != strings.end(); it++) s << *it << ", ";
	s << " ]";
	return s;
}

namespace sweetie_bot {
namespace motion {
namespace controller {


ActionlibControllerBase::ActionlibControllerBase(std::string const& name)  : 
	TaskContext(name, RTT::base::TaskCore::PreOperational),
	log(logger::categoryFromComponentName(name)),
	action_server(this->provides())
{
	// ports
	// PORTS: input
	this->addEventPort("sync", sync_port)
		.doc("Timer syncronization event. This event triggers controller execution cycle.");

	// PORTS: output

	// properties
	this->addProperty("controlled_chains", controlled_chains).
		doc("List of controlled joint groups (kinematic chains, resources).");
	this->addProperty("period", period)
		.doc("Discretization period (s)");

	// operations: provided
	this->addOperation("rosSetOperational", &ActionlibControllerBase::rosSetOperational, this)
		.doc("ROS compatible start/stop operation (std_srvs::SetBool).");

	// Service: requires
	//robot_model = new sweetie_bot::motion::RobotModel(this);
	//this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));

	// action server hook registration
	action_server.setGoalHook(boost::bind(&ActionlibControllerBase::newGoalHook, this, _1));
	action_server.setCancelHook(boost::bind(&ActionlibControllerBase::cancelGoalHook, this));
}

bool ActionlibControllerBase::configureHook()
{
	// INITIALIZATION
	// check if ResourceClient Service presents
	resource_client = getSubServiceByType<ResourceClientInterface>(this->provides().get());
	if (!resource_client) {
		log(ERROR) << "ResourceClient plugin is not loaded." << endlog();
		return false;
	}
	resource_client->setResourceChangeHook(boost::bind(&ActionlibControllerBase::resourceChangeHook, this));
	// check if RobotModel Service presents
	//if (!robot_model->ready() || !robot_model->isConfigured()) {
		//log(ERROR) << "RobotModel service is not ready." << endlog();
		//return false;
	//}
	// Start action server: do not publish feedback
	if (!action_server.start(false)) {
		log(ERROR) << "Unable to start action_server." << endlog();
		return false;
	}
	// call configureHook() actual implementation in subclass
	return configureHook_impl();
}

bool ActionlibControllerBase::dataOnPortHook( RTT::base::PortInterface* portInterface ) 
{
	// Process EventPorts messages callbacks if component is in configured state,
	// so actionlib hooks works even if component is stopped.
	//
	// WARNING: works only in OROCOS 2.9 !!!
	//
    return this->isConfigured();
}

/**
 * activation or deactivation is requested via actionlib interface
 */
void ActionlibControllerBase::newGoalHook(const Goal& pending_goal) 
{
	if (pending_goal.operational == false) {
		//deactivation requested 
		log(INFO) << "actionlib: deactivation request. Component isRunning(): " << isRunning() 
			<< ", resource client state: " << resource_client->getState() << ", actionlib isActive(): " << action_server.isActive() << endlog();
		
		// active goal result
		goal_result.error_code = Result::SUCCESSFUL;
		goal_result.error_string = "Aborted by external request.";
		// abort active if it presents
		action_server.acceptPending(goal_result);
		// stop componet if it was running
		if (isRunning()) {
			goal_result.error_string = "Stopped.";
			action_server.succeedActive(goal_result);
			// this function Stop 
			stop(); 
		}
		else {
			goal_result.error_string = "Already is stopped.";
			action_server.succeedActive(goal_result);
			// transit resource client to non-operation state
			if (!resource_client->isNonOperational()) resource_client->stopOperational();
		}
	}
	else {
		// activation requested
		log(INFO) << "actionlib: activation request. Component isRunning(): " << isRunning() 
			<< ", resource client state: " << resource_client->getState() << ", actionlib isActive(): " << action_server.isActive() << endlog();
		
		// check resource set
		bool resource_set_is_good;
		// TODO better API to simpilfy check if resource set is default
		if (pending_goal.resources.size() > 0) resource_set_is_good = checkResourceSet_impl(pending_goal.resources);
		else  resource_set_is_good = checkResourceSet_impl(controlled_chains); // default resource set

		if (resource_set_is_good) {
			goal_result.error_code = Result::INVALID_RESOURCE_SET;
			goal_result.error_string = "Resource set is rejected.";
			action_server.rejectPending(goal_result);
			return;
		}

		// everything is good: accept goal and request resources
		// active goal result
		goal_result.error_code = Result::SUCCESSFUL;
		goal_result.error_string = "Aborted by external request.";
		action_server.acceptPending(goal_result);

		// forward activation process
		if (isRunning()) {
			const std::vector<std::string>& resources = action_server.getActiveGoal()->resources;
			if (resources.size() > 0) resource_client->resourceChangeRequest(resources); // nonemty resource set
			else resource_client->resourceChangeRequest(controlled_chains); // default resource set
		}
		else {
			// start() request necessary resources from active goal and start component.
			start();
		}
	}
}
/* 
 * Tries to make the controller operational. 
 *
 * Sends a resource request. Later a reply to which will be processed by the plugin's 
 * callback resourceChangedHook. If it returns true controller will become operational.
 * in the updateHook if all resources were allocated or not, if some are lacking.
 * This is checked using the controller's resourceChangedHook.
 */
bool ActionlibControllerBase::startHook()
{
	// request resources
	if (action_server.isActive()) {
		const std::vector<std::string>& resources = action_server.getActiveGoal()->resources;
		// request resources
		if (resources.size() > 0) resource_client->resourceChangeRequest(resources); // nonemty resource set
		else resource_client->resourceChangeRequest(controlled_chains); // default resource set
	}
	else {
		// check if resource set is sane
		if (!checkResourceSet_impl(controlled_chains)) return false;
		// request default resources
		resource_client->resourceChangeRequest(controlled_chains); 
	}
	// clear sync port buffer
	RTT::os::Timer::TimerId timer_id;
	sync_port.readNewest(timer_id);
	// pass controll to actual implementation
	bool is_started = startHook_impl();
	if (!is_started) {
		resource_client->stopOperational(); // force resource_client to NONOPERATIONAL state
		// abort current goal
		if (action_server.isActive()) {
			goal_result.error_code = Result::COMPONENT_IS_NOT_READY; // is this normal behavior?
			goal_result.error_string = "Aborted during the start.";
			action_server.abortActive(goal_result);
		}
	}
	return is_started;
}


/**
 * called when the resource set is changed (by this or other component request).
 */
bool ActionlibControllerBase::resourceChangeHook() 
{
	// if there is active goal resource set must be exactly the same
	if (action_server.isActive()) {
		if (!resource_client->hasResources(action_server.getActiveGoal()->resources)) {
			log(INFO) << "actionlib: not all necessary resources available to satisfy avtive goal." << endlog();
			// not all necessary resources present 
			goal_result.error_code = Result::SUCCESSFUL; // is this normal behavior?
			goal_result.error_string = "Not all recources is available.";
			action_server.abortActive(goal_result);
			// exit operational state: causes stop() call if component isRunning()
			return false;
		}

		// pass control to actual implementation and pass list of requested resources
		bool is_operational = resourceChangedHook_impl(action_server.getActiveGoal()->resources);
		if (!is_operational) {
			goal_result.error_code = Result::SUCCESSFUL; // is this normal behavior?
			goal_result.error_string = "Aborted by resource set change.";
			action_server.abortActive(goal_result);
		}
		return is_operational;
	}
	else {
		// check if desired set od controlled_chains is sane.
		if (!checkResourceSet_impl(controlled_chains)) return false;
		// call implemetation of resourceChangedHook.
		return resourceChangedHook_impl(controlled_chains);
	}
}

void ActionlibControllerBase::updateHook()
{
	// let resource_client do it stuff
	resource_client->step();	

	// syncronize with sync messages
	{
		RTT::os::Timer::TimerId unused;
		if (sync_port.read(unused) != NewData) return;
	}

	// main operational 
	int state = resource_client->getState();
	if (state & ResourceClient::OPERATIONAL) {
		// call update step implementation from subclass
		updateHook_impl();
	}
	else if (state == ResourceClient::NONOPERATIONAL) {
		this->stop();
	}
}

/* 
 * Preempts the controllers and releases its resources.
 */
void ActionlibControllerBase::stopHook() 
{
	// pass control to actual implementation of stopHook()
	stopHook_impl();
	// user calls stop() directly 
	if (!resource_client->isNonOperational()) resource_client->stopOperational();
	// abort active goal
	if (action_server.isActive()) {
		goal_result.error_code = Result::SUCCESSFUL;
		goal_result.error_string = "Stopped.";
		action_server.abortActive(goal_result);
	}
}

void ActionlibControllerBase::cancelGoalHook() 
{
	log(INFO) << "actionlib: cancel request for active goal. Actionlib isActive() " << action_server.isActive()<< endlog();
	// cancel active goal (or do nothing)
	goal_result.error_code = Result::SUCCESSFUL;
	goal_result.error_string = "Canceled by cancel request.";
	action_server.cancelActive(goal_result);
	// stop only if we canceled active goal.
	stop();
}

void ActionlibControllerBase::cleanupHook() 
{
	cleanupHook_impl();
}

/**
 * ROS comaptible start/stop operation.
 */
bool ActionlibControllerBase::rosSetOperational(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
	if (req.data) {
		resp.success = isRunning() || start();
		resp.message = "start() is called.";
	}
	else {
		stop();
		resp.success = true;
		resp.message = "stop() is called.";
	}
	return true;
}

// minimal hook implementation
bool ActionlibControllerBase::configureHook_impl()
{
	log(INFO) << "ActionlibControllerBase is configured." << endlog();
	return true;
}

bool ActionlibControllerBase::checkResourceSet_impl(const std::vector<std::string>& resource_set)
{
	return true;
}

bool ActionlibControllerBase::startHook_impl()
{
	log(INFO) << "ActionlibControllerBase is started." << endlog();
	return true;
}

bool ActionlibControllerBase::resourceChangedHook_impl(const std::vector<std::string>& requested_resource_set)
{
	log(INFO) << "ActionlibControllerBase resource set changed to " << resource_client->listResources() << ", requisted set is " << requested_resource_set << endlog();
	return true;
}

void ActionlibControllerBase::updateHook_impl()
{}

void ActionlibControllerBase::stopHook_impl()
{
	log(INFO) << "ActionlibControllerBase is stopped." << endlog();
}

void ActionlibControllerBase::cleanupHook_impl()
{
	log(INFO) << "ActionlibControllerBase is cleaned up." << endlog();
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot
