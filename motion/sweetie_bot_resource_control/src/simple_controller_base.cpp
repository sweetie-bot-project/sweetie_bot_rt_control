#include "sweetie_bot_resource_control/simple_controller_base.hpp"

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


SimpleControllerBase::SimpleControllerBase(std::string const& name)  : 
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
	this->addProperty("controlled_chains", default_resource_set).
		doc("List of controlled joint groups (kinematic chains, resources).");
	this->addProperty("period", period)
		.doc("Discretization period (s)");

	// operations: provided
	this->addOperation("rosSetOperational", &SimpleControllerBase::rosSetOperational, this)
		.doc("ROS compatible start/stop operation (std_srvs::SetBool).");

	// Service: requires
	//robot_model = new sweetie_bot::motion::RobotModel(this);
	//this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));

	// action server hook registration
	action_server.setGoalHook(boost::bind(&SimpleControllerBase::newGoalHook, this, _1));
	action_server.setCancelHook(boost::bind(&SimpleControllerBase::cancelGoalHook, this));
}

bool SimpleControllerBase::configureHook()
{
	// INITIALIZATION
	// check if ResourceClient Service presents
	resource_client = getSubServiceByType<ResourceClientInterface>(this->provides().get());
	if (!resource_client) {
		log(ERROR) << "ResourceClient plugin is not loaded." << endlog();
		return false;
	}
	resource_client->setResourceChangeHook(boost::bind(&SimpleControllerBase::resourceChangeHook, this));
	// check if RobotModel Service presents
	//if (!robot_model->ready() || !robot_model->isConfigured()) {
		//log(ERROR) << "RobotModel service is not ready." << endlog();
		//return false;
	//}
	// Start action server: do not publish feedback
	if (!action_server.start(false)) {
		log(WARN) << "Unable to start action_server. Possible there are unconnected ports." << endlog();
		//return false;
	}
	// reset sate chnge reason
	start_stop_reason = OROCOS_START_STOP;
	// call configureHook() actual implementation in subclass
	return configureHook_impl();
}

bool SimpleControllerBase::dataOnPortHook( RTT::base::PortInterface* portInterface ) 
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
void SimpleControllerBase::newGoalHook(const Goal& pending_goal) 
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
			start_stop_reason = ACTIONLIB;
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
		if (!processResourceSet_impl(pending_goal.resources, desired_resource_set)) {
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
			resource_client->resourceChangeRequest(desired_resource_set);
		}
		else {
			// start() request necessary resources from active goal and start component.
			start_stop_reason = ACTIONLIB;
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
bool SimpleControllerBase::startHook()
{
	// request resources
	if (action_server.isActive()) {
		// if goal is active desired_resource_set already contains by correct value
	}
	else {
		// we was activated without action, so use default resources set.
		// check if default resource set is sane and fill desired_resource_set
		if (!processResourceSet_impl(default_resource_set, desired_resource_set)) return false;
		// request desired resources
	}
	resource_client->resourceChangeRequest(desired_resource_set); 
	// clear sync port buffer
	RTT::os::Timer::TimerId timer_id;
	sync_port.readNewest(timer_id);
	// pass controll to actual implementation
	bool is_started = startHook_impl(start_stop_reason);
	start_stop_reason = OROCOS_START_STOP;
	// check activation result	
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
bool SimpleControllerBase::resourceChangeHook() 
{
	bool is_operational;
	if (action_server.isActive()) {
		// pass control to actual implementation: pass SetOperationalGoal resources field and set of requested resources
		is_operational = resourceChangedHook_impl(action_server.getActiveGoal()->resources, desired_resource_set);
		if (!is_operational) {
			// inform client about controller deactivation
			goal_result.error_code = Result::SUCCESSFUL; // is this normal behavior?
			goal_result.error_string = "Aborted by resource set change.";
			action_server.abortActive(goal_result);
		}
		else {
			// inform client about resource set change
			goal_feedback.resources = resource_client->listResources();
			action_server.publishFeedback(goal_feedback);
		}
	}
	else {
		// pass control to actual implementation: pass default resources set because (SetOperationalGoal is not available) and set of requested resources.
		is_operational = resourceChangedHook_impl(default_resource_set, desired_resource_set);
	}

	log(DEBUG) << "SimpleControllerBase: resourceChangedHook_impl returned " << is_operational << endlog();
	return is_operational;
}

void SimpleControllerBase::updateHook()
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
		start_stop_reason = RESOURCE_CLIENT;
		stop();
	}
}

/* 
 * Preempts the controllers and releases its resources.
 */
void SimpleControllerBase::stopHook() 
{
	// pass control to actual implementation of stopHook()
	stopHook_impl(start_stop_reason);
	start_stop_reason = OROCOS_START_STOP;
	// user calls stop() directly 
	if (!resource_client->isNonOperational()) resource_client->stopOperational();
	// abort active goal
	if (action_server.isActive()) {
		goal_result.error_code = Result::SUCCESSFUL;
		goal_result.error_string = "Stopped.";
		action_server.abortActive(goal_result);
	}
}

void SimpleControllerBase::cancelGoalHook() 
{
	log(INFO) << "actionlib: cancel request for active goal. Actionlib isActive() " << action_server.isActive()<< endlog();
	// cancel active goal (or do nothing)
	goal_result.error_code = Result::SUCCESSFUL;
	goal_result.error_string = "Canceled by cancel request.";
	action_server.cancelActive(goal_result);
	// stop only if we canceled active goal.
	start_stop_reason = ACTIONLIB;
	stop();
}

void SimpleControllerBase::cleanupHook() 
{
	cleanupHook_impl();
}

/**
 * ROS comaptible start/stop operation.
 */
bool SimpleControllerBase::rosSetOperational(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
	if (req.data) {
		if (isRunning()) {
			start_stop_reason = ROS_SET_OPERATIONAL;
			resp.success = start();
			resp.message = "start() is called.";
		}
		else {
			resp.success = false;
			resp.message = "Already started.";
		}
	}
	else {
		start_stop_reason = ROS_SET_OPERATIONAL;
		stop();
		resp.success = true;
		resp.message = "stop() is called.";
	}
	return true;
}

// minimal hook implementation
bool SimpleControllerBase::configureHook_impl()
{
	log(INFO) << "SimpleControllerBase is configured." << endlog();
	return true;
}

bool SimpleControllerBase::processResourceSet_impl(const std::vector<std::string>& set_operational_goal_resources, std::vector<std::string>& resources_to_request)
{
	resources_to_request = set_operational_goal_resources;
	return true;
}

bool SimpleControllerBase::startHook_impl(StateChangeReason reason)
{
	log(INFO) << "SimpleControllerBase is started (reason = " << reason << ")." << endlog();
	return true;
}

bool SimpleControllerBase::resourceChangedHook_impl(const std::vector<std::string>& set_operational_goal_resources, const std::vector<std::string>& requested_resources)
{
	log(INFO) << "SimpleControllerBase resource set changed to " << resource_client->listResources() << ", requested set is " << requested_resources << endlog();
	return true;
}

void SimpleControllerBase::updateHook_impl()
{}

void SimpleControllerBase::stopHook_impl(StateChangeReason reason)
{
	log(INFO) << "SimpleControllerBase is stopped (reason = " << reason << ")." << endlog();
}

void SimpleControllerBase::cleanupHook_impl()
{
	log(INFO) << "SimpleControllerBase is cleaned up." << endlog();
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot
