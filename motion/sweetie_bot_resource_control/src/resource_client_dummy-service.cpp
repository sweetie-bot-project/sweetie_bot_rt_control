#include "sweetie_bot_resource_control/resource_client.hpp"

#include <rtt/RTT.hpp>
#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

#include <sweetie_bot_resource_control_msgs/typekit/ResourceRequest.h>
#include <sweetie_bot_resource_control_msgs/typekit/ResourceRequesterState.h>
#include <sweetie_bot_resource_control_msgs/typekit/ResourceAssignment.h>


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
		boost::function <bool()> resourceChangeHook;
		//OperationCaller<bool()> resourceChangeHook_call;
		boost::function <void()> stopOperationalHook;
		//OperationCaller<void()> stopOperationalHook_call;

#ifdef SWEETIEBOT_LOGGER
		logger::SWEETIEBOT_LOGGER log;
#else
		logger::LoggerRTT log;
#endif

	public:
		ResourceClientDummyService(TaskContext* owner);

		bool isOperational() const {
			return is_operational;
		}

		bool isPending() const {
			return false;
		}

		bool isNonOperational() const
		{
			return !is_operational;
		}

		int getState() const {
			if (is_operational) return ResourceClient::OPERATIONAL;
			else return ResourceClient::NONOPERATIONAL;
		}

		unsigned long resourceChangeRequest(const std::vector<std::string>& resource_list);

		bool stopOperational();

		bool hasResource(const std::string& resource) const {
			return owned_resources.count(resource);
		}

		bool hasResources(const std::vector<std::string>& resource_list) const;

		std::vector<std::string> listResources() const;

		void step() {}

	    void setResourceChangeHook(boost::function <bool()> resourceChangeHook_) {
			resourceChangeHook = resourceChangeHook_;
		}

		void setStopOperationalHook(boost::function<void()> stopOperationalHook_) {
			stopOperationalHook = stopOperationalHook_;
		}

};
		

ResourceClientDummyService::ResourceClientDummyService(TaskContext* owner) :
	Service("resource_client", owner),
	is_operational(false),
	//resourceChangeHook_call("resourceChangeHook"),
	log(logger::getDefaultCategory("swetie_bot.motion") + ".resource_control")
{
	doc("Dummy resource control client plugin. Always assumes that it owns requested recources.");
	// OPERATONS
	// for component internal use only
	this->addOperation("resourceChangeRequest", &ResourceClientDummyService::resourceChangeRequest, this, OwnThread)
		.doc("Send resource request to arbiter.")
		.arg("resource_list", "Vector with names of resources.");
	this->addOperation("stopOperational", &ResourceClientDummyService::stopOperational, this)
		.doc("Deactivate contoller and inform artbiter about state change.");
	// the external interface: OwnThread is necessary
	this->addOperation("isOperational", &ResourceClientDummyService::isOperational, this)
		.doc("Return true if controller is in operational state.");
	this->addOperation("isPending", &ResourceClientDummyService::isPending, this)
		.doc("Always return false.");
	this->addOperation("getState", &ResourceClientDummyService::getState, this).
		doc("Returns state of ResourceClient: 0=NONOPERATIONAL, 1=PENDING, 2=OPERATIONAL.");
	this->addOperation("hasResource", &ResourceClientDummyService::hasResource, this, OwnThread)
		.doc("Check if controller owns resource.")
		.arg("resource", "Resource name.");
	this->addOperation("hasResources", &ResourceClientDummyService::hasResources, this, OwnThread)
		.doc("Check if controller owns all resourses from list.")
		.arg("resource", "Resource name list.");
	this->addOperation("listResources", &ResourceClientDummyService::listResources, this).
		doc("Return list of owned resources.");
	this->addOperation("step", &ResourceClientDummyService::step, this).
		doc("Process incomming resource assigment. Call this operation periodically.");

	if (!owner) throw std::invalid_argument("ResourceClientDummyService: you must provide owner TaskContext to contructor.");
	
	//resourceChangeHook_call = owner->provides()->getLocalOperation("resourceChangeHook");
	//resourceChangeHook_call.setCaller(owner->engine());
	//stopOperationalHook_call = owner->provides()->getLocalOperation("stopOperationalHook");
	//stopOperationalHook_call.setCaller(owner->engine());

	log(INFO) << "[" << getOwner()->getName()  << "] ResourceDummyService is loaded!" << endlog();
}


unsigned long ResourceClientDummyService::resourceChangeRequest(const std::vector<std::string>& resource_list)
{
	// potentially not real-time: malloc/free calls.
	owned_resources.clear(); 
	std::copy(resource_list.begin(), resource_list.end(), std::inserter(owned_resources, owned_resources.begin())); 

	is_operational = true;
	if (resourceChangeHook) is_operational = resourceChangeHook();
	//else if (resourceChangeHook_call.ready()) is_operational = resourceChangeHook_call();

	log(INFO) << "[" << getOwner()->getName()  << "] ResourceDummyService: ResourceAssignment processed, opertional = " << is_operational << endlog();
	return true;
}

bool ResourceClientDummyService::stopOperational()
{
	owned_resources.clear();
	if (is_operational) {
		is_operational = false;
		if (stopOperationalHook) stopOperationalHook();
		//else if (stopOperationalHook_call.ready()) stopOperationalHook_call();
	}

	log(INFO) << "[" << getOwner()->getName()  << "] ResourceDummyService: exits operational state. " << endlog();
	return true;
}

bool ResourceClientDummyService::hasResources(const std::vector<std::string>& resource_list) const {
	for(std::vector<std::string>::const_iterator res = resource_list.begin(); res != resource_list.end(); res++) {
		if ( !owned_resources.count(*res) ) return false;
	}
	return true;
}

std::vector<std::string> ResourceClientDummyService::listResources() const
{
	std::vector<std::string> resources;
	for(auto it = owned_resources.begin(); it != owned_resources.end(); it++) {
		resources.push_back(*it);
	}
	return resources;
}

} // namespace motion 

} // namespace sweetie_bot

ORO_SERVICE_NAMED_PLUGIN(sweetie_bot::motion::ResourceClientDummyService, "resource_client_dummy")
