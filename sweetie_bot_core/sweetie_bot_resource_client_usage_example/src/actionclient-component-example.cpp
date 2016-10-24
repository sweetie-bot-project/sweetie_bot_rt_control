#include "actionclient-component-example.hpp"

#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include <sweetie_bot_resource_control/sweetie_bot_resource_control_service.hpp>


using namespace RTT;

ControllerActionlibTemplate::ControllerActionlibTemplate(std::string const& name) : 
	TaskContext(name, RTT::base::TaskCore::PreOperational)
{
    log(Info) << "Instantiating a ControllerActionlibTemplate object" << endlog();

	// action server startup
	action_server.addPorts(this->provides());
	action_server.registerGoalCallback(boost::bind(&ControllerActionlibTemplate::goalCallback, this, _1));
	action_server.registerCancelCallback(boost::bind(&ControllerActionlibTemplate::cancelCallback, this, _1));
}

bool ControllerActionlibTemplate::resourceChangedHook()
{
	const std::vector<std::string> res = { "leg1", "leg2" }; // no memory allocation in runtime!
	bool has_resources = resource_client->hasResources(res);

	if (!goal.isValid()) {
		log(Info) << getName() << ": resourceChangedHook: action_server: no valid goal." << endlog();
		return false;
	}
	if (has_resources) {
		log(Info) << getName() << ": resourceChangedHook: necessary resources are acquaired." << endlog();

		switch (goal.getGoalStatus().status) {
			case actionlib_msgs::GoalStatus::PENDING:
				goal.setAccepted();
				//TODO goal start
				log(Info) << getName() << ": resourceChangedHook: action_server: accept goal." << endlog();
				return true;

			case actionlib_msgs::GoalStatus::ACTIVE:
				log(Info) << getName() << ": resourceChangedHook: action_server: goal remains active." << endlog();
				return true;

			default:
				log(Info) << getName() << ": resourceChangedHook: action_server: no valid goal." << endlog();
				return false;
		}
	}
	else {
		log(Info) << getName() << ": resourceChangedHook: there are not enough resources." << endlog();

		switch (goal.getGoalStatus().status) {
			case actionlib_msgs::GoalStatus::PENDING:
				goal.setRejected();
				//TODO goal cleanup
				log(Info) << getName() << ": resourceChangedHook: action_server: reject goal." << endlog();
				return false;

			case actionlib_msgs::GoalStatus::ACTIVE:
				goal.setAborted();
				//TODO goal stop
				//TODO goal cleanup
				log(Info) << getName() << ": resourceChangedHook: action_server: goal remains active." << endlog();
				return false;

			default:
				log(Info) << getName() << ": resourceChangedHook: action_server: no valid goal." << endlog();
				return false;
		}
	}
}

bool ControllerActionlibTemplate::configureHook()
{
	// Get ResourceClient plugin interface.
	Service::ProviderNames subservices;

	subservices = this->provides()->getProviderNames();
	for(Service::ProviderNames::const_iterator name = subservices.begin(); 
            name != subservices.end(); name++) {
        log(Warning) << "Trying to load " << *name << endlog();
        this->provides()->getService(*name).get();
	}

    this->getProvider<ResourceClient>("resource_client");
    resource_client = dynamic_cast<ResourceClientInterface2*>( 
            this->provides()->getService("resource_client").get() );

	if (!resource_client) {
		log(Error) << getName() << ": ResourceClient plugin is not loaded." << endlog();
		return false;
	}

	// Start action server.
	action_server.start();

	log(Info) << getName() << " is configured !" << endlog();
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
				log(Info) << getName() << ": action_server: active goal is canceled by new goal." << endlog();
				break;

			case actionlib_msgs::GoalStatus::PENDING:
				//TODO goal cleanup
				this->stop();

				goal.setCanceled();
				log(Info) << getName() << ": action_server: pending goal is canceled by new goal." << endlog();
				break;
		}
	}
	goal = gh;
	log(Info) << getName() << ": action_server: received new goal." << endlog();
	
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
				log(Info) << getName() << ": action_server: active goal is canceled by client request." << endlog();
				break;

			case actionlib_msgs::GoalStatus::PENDING:
				//TODO goal cleanup
				this->stop();

				goal.setCanceled();
				log(Info) << getName() << ": action_server: pending goal is canceled by client request." << endlog();
				break;
		}
	}
	else {
		log(Error) << "ActionServer: cancel request to unknown goal." << endlog();
	}
}

bool ControllerActionlibTemplate::startHook()
{
	Logger::log(Logger::Info) << getName() <<" is started !" << endlog();

	return true;
}

void ControllerActionlibTemplate::updateHook()
{
	log(Debug) << getName() << " executes updateHook !" << endlog();

	if (resource_client->isOperational()) {
		log(Debug) << getName() << " do useful staff." << endlog();
	} 
}

void ControllerActionlibTemplate::stopHook() 
{
	resource_client->stopOperational();
	log(Info) << getName() << " is stopped!" << endlog();
}

void ControllerActionlibTemplate::cleanupHook() 
{

	resource_client = 0; 
	// action_server.stop(); // no such method
	Logger::log(Logger::Info) << "ControllerActionlibTemplate cleaning up !" <<Logger::endl;
}

ORO_CREATE_COMPONENT(ControllerActionlibTemplate)
