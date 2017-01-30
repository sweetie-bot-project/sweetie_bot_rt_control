#include "controller_actionlib_template-component.hpp"

#include <rtt/Component.hpp>
#include <rtt/os/TimeService.hpp>

using namespace RTT;
using RTT::os::TimeService;

namespace sweetie_bot {
namespace motion {
namespace controller {

ControllerActionlibTemplate::ControllerActionlibTemplate(std::string const& name) : 
	TaskContext(name),
	action_server(this->provides()),
	log("sweetie.motion.controller." + name)
{

	// ports
	// properties
	std::vector<std::string> res = { "leg1", "leg2", "leg3", "leg4" };
	this->addProperty("required_resources", resources_required).
		doc("List of required resources.").
		set(res);
	// operations: provided
	this->addOperation("resourceChangedHook", &ControllerActionlibTemplate::resourceChangedHook, this).
		doc("Check if all necessary resources present and component ready to be set operational.");
	// action server hook registration
	action_server.setGoalHook(boost::bind(&ControllerActionlibTemplate::newGoalHook, this, _1));
	action_server.setCancelHook(boost::bind(&ControllerActionlibTemplate::cancelGoalHook, this));

    log(INFO) << "ControllerActionlibTemplate is constructed!" << endlog();
}

bool ControllerActionlibTemplate::dataOnPortHook( RTT::base::PortInterface* portInterface ) 
{
	// Process EventPorts messages callbacks if component is in configured state,
	// so actionlib hooks works even if component is stopped.
	//
	// WARNING: works only in OROCOS 2.9 !!!
	//
    return this->isConfigured();
}

bool ControllerActionlibTemplate::configureHook()
{
	// Get ResourceClient plugin interface.
	Service::ProviderNames subservices;

	subservices = this->provides()->getProviderNames();
	for(Service::ProviderNames::const_iterator name = subservices.begin(); name != subservices.end(); name++) {
        resource_client = dynamic_cast<ResourceClientInterface*>(this->provides()->getService(*name).get());
        log(DEBUG) << "Trying to load " << *name << " is ResourceClient: " << (resource_client != 0) << endlog();
		if (resource_client) break;
	}
	if (!resource_client) {
		log(ERROR) << "ResourceClient plugin is not loaded." << endlog();
		return false;
	}
	resource_client->setResourceChangeHook(boost::bind(&ControllerActionlibTemplate::resourceChangedHook, this));
	resource_client->setStopOperationalHook(boost::bind(&ControllerActionlibTemplate::stopOperationalHook, this));

	// Start action server: publish feedback
	if (!action_server.start(true)) {
		log(ERROR) << "Unable to start action_server." << endlog();
		return false;
	}

	// TODO get properties
	// TODO check if necessary ports and operations are connected
	// TODO allocate memory
	// TODO set ports data samples

	log(INFO) << "ControllerActionlibTemplate is configured !" << endlog();
	return true;
}

void ControllerActionlibTemplate::newGoalHook(const Goal& pending_goal) 
{
	log(INFO) << "newGoalHook: new pending goal: moving_time = " << pending_goal.speed << " speed = " << pending_goal.speed << endlog();

	// TODO sanity check
	if (pending_goal.moving_time > 1000) {
		log(WARN) << "newGoalHook: incorrect pending goal moving_time: " << pending_goal.moving_time << endlog();
		action_server.rejectPending(Result(), "Goal is not valid.");
		return;
	}
	// TODO check start conditions
	// TODO long preparations to goal activation

	if (resource_client->isOperational()) {
		log(INFO) << "newGoalHook: abort active goal." << endlog();
		// we are in operational state and own all necessary resources
		// or we can reject pending goal
		
		result.moving_time = TimeService::Instance()->getSeconds(start_time);
		action_server.acceptPending(result, "Aborted by new goal.");
		//TODO replace active goal
		goal = pending_goal;
		start_time = TimeService::Instance()->getTicks();
	}
	else {
		log(INFO) << "newGoalHook: request resources." << endlog();
		// request resources
		resource_client->resourceChangeRequest(resources_required);
		start();
	}
}

bool ControllerActionlibTemplate::resourceChangedHook()
{
	bool success;
	bool has_resources = resource_client->hasResources(resources_required);

	if (has_resources) {
		log(INFO) << "resourceChangedHook: componet has necessary resoures." << endlog();

		boost::shared_ptr<const Goal> pending_goal = action_server.getPendingGoal();
		if (pending_goal) {
			log(INFO) << "resourceChangedHook: have valid pending goal." << endlog();

			// set active goal result if it presents
			if (action_server.isActive()) 
				result.moving_time = TimeService::Instance()->getSeconds(start_time);
			
			// TODO: check startup conditions if necessary
			if (action_server.isActive()) {
				//TODO replace active goal
				start_time = TimeService::Instance()->getTicks();
			}
			else {
				// TODO prepare to pursue active goal
				start_time = TimeService::Instance()->getTicks();
			}
			goal = *pending_goal;

			// accept pending goal and abort active
			success = action_server.acceptPending(result, "New goal is arrived.");
			if (success) {
				log(INFO) << "resourceChangedHook: new active goal: moving_time = " << goal.speed << " speed = " << goal.speed << endlog();
			}
			else {
				log(ERROR) << "resourceChangedHook: unable to activate pending goal." << endlog();
			}
			return success;
		}
		else {
			success = action_server.isActive();
			log(INFO) << "resourceChangedHook: have no pending goal, active goal status = " << (int) success  << endlog();
			// exit opertional state if there is no goal
			return success;
		}
	}
	else { // if(has_resources)
		log(INFO) << "resourceChangedHook: there are not enough resources." << endlog();
		// reject pending goal
		action_server.rejectPending(result, "Not enough resources.");
		// abort active goal
		if (action_server.isActive()) {
			// set result and abort
			result.moving_time = TimeService::Instance()->getSeconds(start_time);
			action_server.abortActive(result, "Not enough resources.");
			log(INFO) << "resourceChangedHook: active goal is aborted." << endlog();
		}
		return false;
	}
}	

void ControllerActionlibTemplate::cancelGoalHook() 
{
	log(INFO) << "cancelGoalHoook: abort active goal." << endlog();

	// set action result
	result.moving_time = TimeService::Instance()->getSeconds(start_time);
	// cancel, abort or success action
	action_server.cancelActive(result, "Canceled by user request.");
	// stop
	resource_client->stopOperational();
}

bool ControllerActionlibTemplate::startHook()
{
	log(INFO) << "ControllerActionlibTemplate is started !" << endlog();
	return true;
}

void ControllerActionlibTemplate::updateHook()
{
	log(DEBUG) << "ControllerActionlibTemplate executes updateHook !" << endlog();
	// check messages on resource_assigment port
	resource_client->step();

	if (resource_client->isOperational()) {
		// check goal is not necessary: we will be notified via cancelGoalHook.
		// Also we have goal buffer, so using action_server.getActiveGoal() is not necessary.
		log(DEBUG) << "Moving to goal: speed = " << goal.speed << endlog();

		// do useful staff
		double elasped = TimeService::Instance()->getSeconds(start_time);
		if (elasped < goal.moving_time) {
			feedback.moving_time = elasped;
			action_server.publishFeedback(feedback);
		}
		else {
			log(DEBUG) << "Goal is achived !" << endlog();
			// succeed goal
			result.moving_time = elasped;
			action_server.succeedActive(result, "Goal is reached.");
			// exit operational state
			resource_client->stopOperational();
		}
	} 
}

void ControllerActionlibTemplate::stopOperationalHook() {
	log(DEBUG) << "Exit OPERATIONAL state." << endlog();
	// TODO: clean up 
	// stop
	this->stop();
}

void ControllerActionlibTemplate::stopHook() 
{
	if (resource_client->isOperational()) { // prevent recursion
		resource_client->stopOperational()
		log(INFO) << "ControllerActionlibTemplate is stopped!" << endlog();
	}
}

void ControllerActionlibTemplate::cleanupHook() 
{

	resource_client = 0; 
	//action_server.shutdown(); 
	log(INFO) << "ControllerActionlibTemplate cleaning up !" << endlog();
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

ORO_CREATE_COMPONENT(sweetie_bot::motion::controller::ControllerActionlibTemplate)
