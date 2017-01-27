#include "controller_actionlib_template-component.hpp"

#include <rtt/Component.hpp>

using namespace RTT;

namespace sweetie_bot {
namespace motion {
namespace controller {

ControllerActionlibTemplate::ControllerActionlibTemplate(std::string const& name) : 
	TaskContext(name),
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

	// action server startup
	action_server.addPorts(this->provides());
	action_server.registerGoalCallback(boost::bind(&ControllerActionlibTemplate::goalCallback, this, _1));
	action_server.registerCancelCallback(boost::bind(&ControllerActionlibTemplate::cancelCallback, this, _1));

    log(INFO) << "ControllerActionlibTemplate is constructed!" << endlog();
}

bool ControllerActionlibTemplate::dataOnPortHook( RTT::base::PortInterface* portInterface ) 
{
    //return this->isConfigured();
    //return this->isRunning();
    return true;
}

bool ControllerActionlibTemplate::resourceChangedHook()
{
	const std::vector<std::string> res = { "leg1", "leg2" }; // no memory allocation in runtime!
	bool has_resources = resource_client->hasResources(res);

	if (!goal.isValid()) {
		log(INFO) << getName() << ": resourceChangedHook: action_server: no valid goal." << endlog();
		return false;
	}
	if (has_resources) {
		log(INFO) << getName() << ": resourceChangedHook: necessary resources are acquaired." << endlog();

		switch (goal.getGoalStatus().status) {
			case actionlib_msgs::GoalStatus::PENDING:
				goal.setAccepted();
				//TODO goal start
				log(INFO) << getName() << ": resourceChangedHook: action_server: accept goal." << endlog();
				return true;

			case actionlib_msgs::GoalStatus::ACTIVE:
				log(INFO) << getName() << ": resourceChangedHook: action_server: goal remains active." << endlog();
				return true;

			default:
				log(INFO) << getName() << ": resourceChangedHook: action_server: no valid goal." << endlog();
				return false;
		}
	}
	else {
		log(INFO) << getName() << ": resourceChangedHook: there are not enough resources." << endlog();

		switch (goal.getGoalStatus().status) {
			case actionlib_msgs::GoalStatus::PENDING:
				goal.setRejected();
				//TODO goal cleanup
				log(INFO) << getName() << ": resourceChangedHook: action_server: reject goal." << endlog();
				return false;

			case actionlib_msgs::GoalStatus::ACTIVE:
				goal.setAborted();
				//TODO goal stop
				//TODO goal cleanup
				log(INFO) << getName() << ": resourceChangedHook: action_server: goal remains active." << endlog();
				return false;

			default:
				log(INFO) << getName() << ": resourceChangedHook: action_server: no valid goal." << endlog();
				return false;
		}
	}
}

bool ControllerActionlibTemplate::configureHook()
{
	// Get ResourceClient plugin interface.
	Service::ProviderNames subservices;

	subservices = this->provides()->getProviderNames();
	for(Service::ProviderNames::const_iterator name = subservices.begin(); name != subservices.end(); name++) {
        log(INFO) << "Trying to load " << *name << endlog();
        resource_client = dynamic_cast<ResourceClientInterface*>(this->provides()->getService(*name).get());
		if (resource_client) break;
	}
	if (!resource_client) {
		log(ERROR) << getName() << ": ResourceClient plugin is not loaded." << endlog();
		return false;
	}

	// Start action server.
	action_server.start();
	action_server.initialize();

	log(INFO) << getName() << " is configured !" << endlog();
	return true;
}

void ControllerActionlibTemplate::goalCallback(GoalHandle gh) 
{
	if ( goal.isValid() )
    {
		switch ( goal.getGoalStatus().status ) 
        {
			case actionlib_msgs::GoalStatus::ACTIVE:
				//TODO goal stop
				//TODO goal cleanup
				resource_client->stopOperational();
				this->stop();
				
				goal.setCanceled();
				log(INFO) << getName() << ": action_server: active goal is canceled by new goal." << endlog();
				break;

			case actionlib_msgs::GoalStatus::PENDING:
				//TODO goal cleanup
				this->stop();

				goal.setCanceled();
				log(INFO) << getName() << ": action_server: pending goal is canceled by new goal." << endlog();
				break;
		}
	}
	goal = gh;
	log(INFO) << getName() << ": action_server: received new goal." << endlog();
	
	//TODO goal execution conditions check
	this->start();
	//request resources
	const std::vector<std::string> res = { "leg1", "leg2", "head" }; // no memory allocation in runtime!
	resource_client->requestResources(res);
}

void ControllerActionlibTemplate::cancelCallback(GoalHandle gh) 
{
    if (goal == gh) {
		switch ( goal.getGoalStatus().status ) {
			case actionlib_msgs::GoalStatus::ACTIVE:
				//TODO goal stop
				//TODO goal cleanup
				resource_client->stopOperational();
				this->stop();

				goal.setCanceled();
				log(INFO) << getName() << ": action_server: active goal is canceled by client request." << endlog();
				break;

			case actionlib_msgs::GoalStatus::PENDING:
				//TODO goal cleanup
				this->stop();

				goal.setCanceled();
				log(INFO) << getName() << ": action_server: pending goal is canceled by client request." << endlog();
				break;

            default:
                log(WARN) << ": action_server: cancel callback received strange stuff." << endlog();
		}
	}
	else {
		log(ERROR) << "ActionServer: cancel request to unknown goal." << endlog();
	}
}

bool ControllerActionlibTemplate::startHook()
{
	log(INFO) << getName() <<" is started !" << endlog();

	return true;
}

void ControllerActionlibTemplate::updateHook()
{
	log(DEBUG) << getName() << " executes updateHook !" << endlog();

	if (resource_client->isOperational()) {
		log(DEBUG) << getName() << " do useful staff." << endlog();
	} 
}

void ControllerActionlibTemplate::stopHook() 
{
	resource_client->stopOperational();
	log(INFO) << getName() << " is stopped!" << endlog();
}

void ControllerActionlibTemplate::cleanupHook() 
{

	resource_client = 0; 
	// action_server.stop(); // no such method
	log(INFO) << "ControllerActionlibTemplate cleaning up !" << endlog();
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

ORO_CREATE_COMPONENT(sweetie_bot::motion::controller::ControllerActionlibTemplate)
