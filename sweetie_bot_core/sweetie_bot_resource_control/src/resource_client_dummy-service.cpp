#include <sweetie_bot_resource_control/resource_client-service.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

#include <sweetie_bot_resource_control_msgs/ResourceRequest.h>
#include <sweetie_bot_resource_control_msgs/ResourceRequesterState.h>
#include <sweetie_bot_resource_control_msgs/ResourceAssignment.h>


#include <sweetie_bot_logger/logger.hpp>

#include <algorithm>

using namespace RTT;
using namespace std;
using namespace sweetie_bot_resource_control_msgs;


namespace sweetie_bot {

namespace motion {

class ResourceClientDummyService : 
	public ResourceClientInterface, public RTT::Service 
{
	protected:
		bool is_operational; 
		std::set<std::string> owned_resources;
		bool (*resourceChangedHook)();
		OperationCaller<bool()> resourceChangedHook_call;

#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::LoggerRTT log;
#endif

	public:
		ResourceClientDummyService(TaskContext* owner);

		bool isOperational() {
			return is_operational;
		}

		bool requestResources(const std::vector<std::string>& resource_list);

		bool stopOperational();

		bool hasResource(const std::string& resource) {
			return owned_resources.count(resource);
		}

		bool hasResources(const std::vector<std::string>& resource_list);

		void step() {}

	    virtual void setResourceChangeHook(bool (*resourceChangedHook_)()) {
			resourceChangedHook = resourceChangedHook_;
		}
};
		

ResourceClientDummyService::ResourceClientDummyService(TaskContext* owner) :
	Service("resource_client", owner),
	is_operational(false),
	resourceChangedHook_call("resourceChangedHook"),
	log("swetie.motion.resource_control")
{
	doc("Dummy resource control client plugin. Always assumes that it owns requested recources.");
	// OPERATONS
	// for component internal use only
	this->addOperation("requestResources", &ResourceClientDummyService::requestResources, this)
		.doc("Send resource request to arbiter.")
		.arg("resource_list", "Vector with names of resources.");
	this->addOperation("stopOperational", &ResourceClientDummyService::stopOperational, this)
		.doc("Deactivate contoller and inform artbiter about state change.");
	// the external interface: OwnThread is necessary
	this->addOperation("isOperational", &ResourceClientDummyService::isOperational, this, OwnThread)
		.doc("Return true if controller is in operational state.");
	this->addOperation("hasResource", &ResourceClientDummyService::hasResource, this, OwnThread)
		.doc("Check if controller owns resource.")
		.arg("resource", "Resource name.");
	this->addOperation("hasResources", &ResourceClientDummyService::hasResources, this, OwnThread)
		.doc("Check if controller owns all resourses from list.")
		.arg("resource", "Resource name list.");
	this->addOperation("step", &ResourceClientDummyService::step, this).
		doc("Process incomming resource assigment. Call this operation periodically.");

	if (!owner) throw std::invalid_argument("ResourceClientDummyService: you must provide owner TaskContext to contructor.");
	
	resourceChangedHook_call = owner->provides()->getLocalOperation("resourceChangedHook");
	resourceChangedHook_call.setCaller(owner->engine());

	log(INFO) << "[" << getOwner()->getName()  << "] ResourceDummyService is loaded!" << endlog();
}


bool ResourceClientDummyService::requestResources(const std::vector<std::string>& resource_list)
{
	// potentially not real-time: malloc/free calls.
	owned_resources.clear(); 
	std::copy(resource_list.begin(), resource_list.end(), std::inserter(owned_resources, owned_resources.begin())); 

	is_operational = true;
	if (resourceChangedHook) is_operational = resourceChangedHook();
	else if (resourceChangedHook_call.ready()) is_operational = resourceChangedHook_call();

	log(INFO) << "[" << getOwner()->getName()  << "] ResourceDummyService: ResourceAssignment processed, opertional = " << is_operational << endlog();
	return true;
}

bool ResourceClientDummyService::stopOperational()
{
	owned_resources.clear();
	is_operational = false;

	log(INFO) << "[" << getOwner()->getName()  << "] ResourceDummyService: exits operational state. " << endlog();
	return true;
}

bool ResourceClientDummyService::hasResources(const std::vector<std::string>& resource_list) {
	for(std::vector<std::string>::const_iterator res = resource_list.begin(); res != resource_list.end(); res++) {
		if ( !owned_resources.count(*res) ) return false;
	}
	return true;
}

} // namespace motion 

} // namespace sweetie_bot

ORO_SERVICE_NAMED_PLUGIN(sweetie_bot::motion::ResourceClientDummyService, "resource_client_dummy")
