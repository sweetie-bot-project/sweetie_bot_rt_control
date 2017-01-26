#include "controller_template-component.hpp"

#include <rtt/Component.hpp>

using namespace RTT;

namespace sweetie_bot {
namespace motion {
namespace controller {

ControllerTemplate::ControllerTemplate(std::string const& name)  : 
	TaskContext(name, RTT::base::TaskCore::PreOperational),
	log("sweetie.motion.controller.template")
	//log("sweetie.motion.controller." + name)
{
	resource_client = getProvider<ResourceClient>("resource_client").get();
	// ports
	// properties
	std::vector<std::string> res = { "leg1", "leg2", "leg3", "leg4" };
	this->addProperty("required_resources", resources_required).
		doc("List of required resources.").
		set(res);
	// operations
	this->addOperation("resourceChangedHook", &ControllerTemplate::resourceChangedHook, this).
		doc("Check if all necessary resources present and component ready to be set operational.");
	// services
	if (!resource_client) {
		log(ERROR) << getName() << "Unable to load `resource_client` service!" << endlog();
		this->exception();
	}
	// other actions
	log(INFO) << "ControllerTemplate is constructed!" << endlog();
}

/*	Determines if the controller can function normally in the given situation or not. 
 *	Usually it just checks if all the required resources have been allocated to it.
 *	The isOperational flag will be set by the plugin.
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
	// perform opertional condition checks and state reset if necessary
	return true;
}

bool ControllerTemplate::configureHook()
{
	// INITIALIZATION
	// get properties
	// check if necessary ports and operations are connected
	// allocate memory
	// set ports data samples
	log(INFO) << "ControllerTemplate is configured !" << endlog();
	return true;
}

/* 
 * Tries to make the controller operational. 
 *
 * Sends a resource request, a reply to which will be processed by the plugin's
 * callback and the controller will be activated and start doing its task
 * in the updateHook if all resources were allocated or not, if some are lacking.
 * This is checked using the controller's resourceChangedHook.
 */
bool ControllerTemplate::startHook()
{
	// reset state variables
	// check if controller can be set operational in current conditions
	// request resources
	resource_client->requestResources(resources_required);

	// now update hook will be periodically executed
	log(INFO) << "ControllerTemplate is started !" << endlog();
	return true;
}

void ControllerTemplate::updateHook()
{
	// let resource_client do it stuff
	resource_client->step();	

	// main operational 
	if (resource_client->isOperational()) {
		log(DEBUG) << "ControllerTemplate executes updateHook!" << endlog();
	}
}

/* 
 * Preempts the controllers and releases its resources.
 */
void ControllerTemplate::stopHook() 
{
	// deinitialization
	// release all resources
	resource_client->stopOperational();
	log(INFO) << "ControllerTemplate is stopped!" << endlog();
}

void ControllerTemplate::cleanupHook() 
{
	// free memory, close files and etc
	log(INFO) << "ControllerTemplate cleaning up !" << endlog();
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
