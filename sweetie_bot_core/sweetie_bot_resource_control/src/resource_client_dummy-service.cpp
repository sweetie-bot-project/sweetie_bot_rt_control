#include <rtt/RTT.hpp>
#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/Logger.hpp>

#include <sweetie_bot_resource_control_msgs/ResourceRequest.h>
#include <sweetie_bot_resource_control_msgs/ResourceRequesterState.h>
#include <sweetie_bot_resource_control_msgs/ResourceAssignment.h>

#include <sweetie_bot_resource_control/sweetie_bot_resource_control_service.hpp>

#include <algorithm>

using namespace RTT;
using RTT::Logger;
using namespace std;
using namespace sweetie_bot_resource_control_msgs;


class ResourceClientDummyService : 
	public ResourceClientInterface2, public RTT::Service 
{
	protected:
		bool is_operational; 
		std::set<std::string> owned_resources;
		OperationCaller<bool()> resourceChangedHook;

	public:
		ResourceClientDummyService(TaskContext* owner);

		bool isOperational() {
			return is_operational;
		}

		void requestResources(const std::vector<std::string>& resource_list);

		void stopOperational();

		bool hasResource(const std::string& resource) {
			return owned_resources.count(resource);
		}

		bool hasResources(const std::vector<std::string>& resource_list);
};
		

ResourceClientDummyService::ResourceClientDummyService(TaskContext* owner) :
	Service("resource_client_dummy", owner),
	is_operational(false),
	resourceChangedHook("resourceChangedHook")
{
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

	if (!owner) throw std::invalid_argument("ResourceClientDummyService: you must provide owner TaskContext to contructor.");
	
	resourceChangedHook = owner->provides()->getLocalOperation("resourceChangedHook");
	resourceChangedHook.setCaller(owner->engine());

	log(Info) << "resource_client_dummy service is loaded into " << getOwner()->getName() << " component." << endlog();
}


void ResourceClientDummyService::requestResources(const std::vector<std::string>& resource_list)
{
	// potentially not real-time: malloc/free calls.
	owned_resources.clear(); 
	std::copy(resource_list.begin(), resource_list.end(), std::inserter(owned_resources, owned_resources.begin())); 

	if (resourceChangedHook.ready()) resourceChangedHook();
	is_operational = true;

	log(Info) << "resource_client_dummy: " << this->getOwner()->getName() << " is switched operational." << endlog();
}

void ResourceClientDummyService::stopOperational()
{
	owned_resources.clear();
	is_operational = false;

	log(Info) << "resource_client_dummy: " << getOwner()->getName() << " is switched nonoperational." << endlog();
}

bool ResourceClientDummyService::hasResources(const std::vector<std::string>& resource_list) {
	for(std::vector<std::string>::const_iterator res = resource_list.begin(); res != resource_list.end(); res++) {
		if ( !owned_resources.count(*res) ) return false;
	}
	return true;
}

ORO_SERVICE_NAMED_PLUGIN(ResourceClientDummyService, "resource_client_dummy")

