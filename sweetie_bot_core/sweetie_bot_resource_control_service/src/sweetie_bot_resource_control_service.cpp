#include <rtt/RTT.hpp>
#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

#include <sweetie_bot_resource_control_msgs/ResourceRequest.h>
#include <sweetie_bot_resource_control_msgs/ResourceRequesterState.h>
#include <sweetie_bot_resource_control_msgs/ResourceAssignment.h>

#include <sweetie_bot_resource_control_service/sweetie_bot_resource_control_service.hpp>

#include <algorithm>

using namespace RTT;
using namespace std;

using namespace sweetie_bot_resource_control_msgs;

/**
 * An example service which can be loaded in a component.
 */
class ResourceClientService : public ResourceClientInterface, 
  public RTT::Service 
{
  protected:
	 RTT::OutputPort<ResourceRequest> resourceRequestPort;
	 RTT::OutputPort<ResourceRequesterState> resourceRequesterStatePort;

	 RTT::InputPort<ResourceAssignment> resourceAssignmentPort;

	 std::string name_;

	 // maps a resource that is absolutely essential to the component
	 // to a priority in range [0:1] that indicates how important this
	 // resource is to this user among all possible other users
	 std::map<std::string, double> resourcesRequired;

	 // a list of resources that this user was given control of
	 std::map<std::string, bool> resourcesControlled;

	 void processResourceAssignment(ResourceAssignment &msg)
	 {
	 	bool isResourcesAcquired = true;

		for (std::map<std::string, double>::iterator it = resourcesRequired.begin();
			 it != resourcesRequired.end(); ++it)
		{
		  std::vector<std::string>::iterator res_it = std::find(msg.resources.begin(), msg.resources.end(), it->first);
		  if ( res_it == msg.resources.end() )
		  {
		  	 // resource not controlled
			 isResourcesAcquired = false;
			 break;
		  }
		}

		ResourceRequesterState resourceRequesterStateMsg;
		resourceRequesterStateMsg.requester_name = name_;

		if (isResourcesAcquired)
		  resourceRequesterStateMsg.is_operational = true;
		else
		  resourceRequesterStateMsg.is_operational = false;

		resourceRequesterStatePort.write(resourceRequesterStateMsg);
	 }

  public:
    ResourceClientService(TaskContext* owner) 
        : Service("resource_client", owner) 
    {
		this->addPort("resource_request", resourceRequestPort);
		this->addPort("resource_requester_status", resourceRequesterStatePort);

		this->addEventPort("resource_assignment", resourceAssignmentPort);

		this->addOperation("getOwnerName", &ResourceClientService::getOwnerName, this)
		  .doc("Returns the name of the owner of this object.");

		this->addOperation("requestResources", &ResourceClientService::requestResources, this);
		this->addOperation("stopOperational", &ResourceClientService::stopOperational, this);
		this->addOperation("isOperational", &ResourceClientService::isOperational, this);
		this->addOperation("hasResource", &ResourceClientService::hasResource, this);

		name_ = getOwner()->getName();
    }

	 std::string getOwnerName() 
    {
		// getOwner() returns the TaskContext pointer we got in
		// the constructor:
		return getOwner()->getName();
    }

  	 bool requestResources(std::map<std::string, double> resourcesPriorities)
  	 {
		ResourceRequest resourceRequestMsg;

		resourceRequestMsg.requester_name = name_;

		resourcesRequired.clear();
		for (std::map<std::string, double>::iterator it = resourcesPriorities.begin();
			 it != resourcesPriorities.end(); ++it)
		{
		  resourcesRequired.insert(*it);

		  resourceRequestMsg.resources.push_back(it->first);
		  // TODO: add priorities
  		}

  		resourceRequestPort.write(resourceRequestMsg);
  	 }

  	 bool stopOperational()
  	 {
  	 	ResourceRequesterState msg;

  	 	msg.requester_name = name_;
  	 	msg.is_operational = false;

  	 	resourceRequesterStatePort.write(msg);

		return true;
  	 }

  	 bool isOperational()
  	 {
  	 	for (std::map<std::string, double>::iterator it = resourcesRequired.begin();
  	 		 it != resourcesRequired.end(); ++it)
  	 	{
		  if ( (resourcesControlled.find(it->first) == resourcesControlled.end())
		  	 || (resourcesControlled[it->first] == false) )
		  {
			 // one of the required resources is not controlled, thus the component is inactive
			 return false;
		  }
  	 	}

		return true;
  	 }

  	 bool hasResource(std::string res)
  	 {
		bool hasResource = false;

		if (resourcesControlled.find(res) == resourcesControlled.end())
		  hasResource = false;
		else 
		  hasResource = resourcesControlled[res];

		return hasResource;
  	 }
  	 
};

/* For consistency reasons, it's better to name the
 * service the same as in the class above.
 */
ORO_SERVICE_NAMED_PLUGIN(ResourceClientService, "resource_client")
