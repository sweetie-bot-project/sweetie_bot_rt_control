#include "controller_template-component.hpp"

#include <rtt/Component.hpp>

using namespace RTT;

namespace sweetie_bot {
namespace motion {
namespace controller {
	//
//TODO move somewhere

/**
 * @brief Find OROCOS subservice which implements given interface.
 * Find OROCOS subservice which implements given interface ServiceInterface.
 * @param service Pointer to parent service.
 * @return Pointer to found subservice or zero if nothing found.
 **/
template<class ServiceInterface> ServiceInterface * getSubServiceByInterface(Service * service) 
{
	if (!service) return nullptr;

	ServiceInterface * found_service;
	Service::ProviderNames subservices;

	subservices = service->getProviderNames();
	for(Service::ProviderNames::const_iterator name = subservices.begin(); name != subservices.end(); name++) {
        found_service = dynamic_cast<ServiceInterface*>(service->getService(*name).get());
		if (found_service) return found_service;
	}
	return nullptr;
}


ControllerTemplate::ControllerTemplate(std::string const& name)  : 
	TaskContext(name, RTT::base::TaskCore::PreOperational),
	log(logger::categoryFromComponentName(name))
{
	// ports
	// properties
	std::vector<std::string> res = { "leg1", "leg2", "leg3", "leg4" };
	this->addProperty("required_resources", resources_required).
		doc("List of required resources.").
		set(res);
	// operations: provided
	this->addOperation("rosSetOperational", &ControllerTemplate::rosSetOperational, this)
		.doc("ROS compatible start/stop operation (std_srvs::SetBool).");
	// other actions
	log(INFO) << "ControllerTemplate is constructed!" << endlog();
}

/*	
 *	@brief Hook is called by `resource_client` to check if all necessary resources present and component is ready to be set operational.
 *  
 *	It determines if the controller can function normally in the given situation or not. 
 *	This usually involves checks if all the required resources have been allocated to it.
 *
 *	@return true if component is ready to become opertional. Otherwise resources would be released and state would be switched to non-operational.
 */
bool ControllerTemplate::resourceChangedHook()
{
	log(INFO) << "ControllerTemplate resourceChangedHook is being run!" << endlog();
	// unable to run if dont have a leg.
	if (resource_client->hasResource("leg1")) log(INFO) << "ControllerTemplate have a leg." << endlog();
	else log(INFO) << "ControllerTemplate do not have a leg." << endlog();
	// check if all resources present (if resourceChangedHook is not declared, resources client check this condition automatically).
	if (!resource_client->hasResources(resources_required)) {
		log(INFO) << "ControllerTemplate do NOT HAVE all resources." << endlog();
		return false;
	}
	log(INFO) << "ControllerTemplate HAVE all resources." << endlog();
	// TODO perform opertional condition checks and state reset if necessary
	return true;
}

bool ControllerTemplate::configureHook()
{
	// INITIALIZATION
	// check if ResourceClient Service presents: try to find it amoung loaded services
	resource_client = getSubServiceByInterface<ResourceClientInterface>(this->provides().get());
	if (!resource_client) {
		log(ERROR) << "ResourceClient plugin is not loaded." << endlog();
		return false;
	}
	resource_client->setResourceChangeHook(boost::bind(&ControllerTemplate::resourceChangedHook, this));

	// TODO get properties
	// TODO check if necessary ports and operations are connected
	// TODO allocate memory
	// TODO set ports data samples
	log(INFO) << "ControllerTemplate is configured !" << endlog();
	return true;
}

/* 
 * Tries to make the controller operational. 
 *
 * Sends a resource request. Later a reply to which will be processed by the plugin's 
 * callback resourceChangedHook. If it returns true controller will become operational.
 * in the updateHook if all resources were allocated or not, if some are lacking.
 * This is checked using the controller's resourceChangedHook.
 */
bool ControllerTemplate::startHook()
{
	// reset state variables
	// check if controller can be set operational in current conditions
	// request resources
	resource_client->resourceChangeRequest(resources_required);

	// now update hook will be periodically executed
	log(INFO) << "ControllerTemplate is started !" << endlog();
	return true;
}

void ControllerTemplate::updateHook()
{
	log(DEBUG) << "ControllerTemplate executes updateHook!" << endlog();
	// let resource_client do it stuff
	resource_client->step();	

	// main operational 
	int state = resource_client->getState();
	if (state & ResourceClient::OPERATIONAL) {
		log(DEBUG) << "ControllerTemplate is operational in updateHook!" << endlog();
	}
	else if (state == ResourceClient::NONOPERATIONAL) {
		log(INFO) << "ControllerTemplate is exiting  operational state !" << endlog();
		this->stop();
	}
}

/* 
 * Preempts the controllers and releases its resources.
 */
void ControllerTemplate::stopHook() 
{
	// user calls stop() directly 
	if (resource_client->isOperational()) resource_client->stopOperational();
	// deinitialization
	// release all resources
	log(INFO) << "ControllerTemplate is stopped!" << endlog();
}

void ControllerTemplate::cleanupHook() 
{
	// free memory, close files and etc
	log(INFO) << "ControllerTemplate cleaning up !" << endlog();
}

/**
 * ROS comaptible start/stop operation.
 */
bool ControllerTemplate::rosSetOperational(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
	if (req.data) {
		resp.success = start();
		resp.message = "start() is called.";
	}
	else {
		stop();
		resp.success = true;
		resp.message = "stop() is called.";
	}
	return true;
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ControllerTemplate)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::controller::ControllerTemplate)
