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

	 /* Is true when the controller can function. Usually depends on allocated resources.
	  *
	  */
	 bool isOperational;

	 /* Setter of isOperational.
	  */
  	 void setIsOperational(bool value)
  	 {
		isOperational = value;

		Logger::log(Logger::Info) << name_ << " status changed to: " << value << Logger::endl;
	 }

	 /* Maps a resource that is absolutely essential to the component
	  * to a priority in range [0:1] that indicates how important this
	  * resource is to this user among all possible other users
	  */
	 std::map<std::string, double> resourcesRequired;

	 /* a list of resources that this user was given control of
	  */
	 std::map<std::string, bool> resourcesControlled;

	 /* Hook that is provided by the controller that is using this plugin to determine
	  * whether it can work with the given resources. 
	  *
	  * @return bool True when the controller is active (can function), false otherwise.
	  */
	 bool (*resourceChangedHook)();

	 /* Detects if the requested resources were allocated to this component and replies
	  * with an activation message if they were, with a deactivation message otherwise.
	  *
	  * @param msg ResourceAssignment Message from the resource arbitrator the contains 
	  * all resources and indicates who they were given to.
	  */
	 void processResourceAssignment(ResourceAssignment &msg)
	 {
	 	bool resourceFound = false;

		for (std::map<std::string, double>::iterator it = resourcesRequired.begin();
			 it != resourcesRequired.end(); ++it)
		{
		  resourceFound = false;
		  // try to find the resource among the allocated
		  for (int i = 0; i < msg.resources.size(); i++)
		  {
		  	 if (msg.resources[i] == it->first)
		  	 {
		  	 	resourceFound = true;

		  	 	if (msg.owners[i] == name_)
		  	 	{
				  it->second = true;
				  Logger::log(Logger::Info) << name_ << " acquired resource "
				  	 << it->first << Logger::endl;
				}

				// resource was found and is either ours, or not. proceed to the next one
				break;
		  	 }
		  }

		  if (!resourceFound)
		  	 Logger::log(Logger::Warning) << "Resource " << it->first << " does not exist"
		  	 	<< Logger::endl;
		}

		// ask the controller if it can function with these resources
		if (resourceChangedHook != NULL)
		  isOperational = resourceChangedHook();
	 	else
		  Logger::log(Logger::Error) << "resourceChangedHook() is null" << Logger::endl;

		// announce state
		ResourceRequesterState resourceRequesterStateMsg;
		resourceRequesterStateMsg.requester_name = name_;

		resourceRequesterStateMsg.is_operational = isOperational;

		resourceRequesterStatePort.write(resourceRequesterStateMsg);
	 }


  public:
    ResourceClientService(TaskContext* owner) 
        : Service("resource_client", owner),
        isOperational(false)
    {
		this->addPort("resource_request", resourceRequestPort);
		this->addPort("resource_requester_state", resourceRequesterStatePort);

		this->addEventPort("resource_assignment", resourceAssignmentPort);

		this->addOperation("requestResources", &ResourceClientService::requestResources, this);
		this->addOperation("stopOperational", &ResourceClientService::stopOperational, this);
		this->addOperation("getIsOperational", &ResourceClientService::getIsOperational, this);
		this->addOperation("hasResource", &ResourceClientService::hasResource, this);

		name_ = getOwner()->getName();
    }

    // Properties' getters and setters

	 /* Getter of isOperational.
	  */
  	 bool getIsOperational()
  	 {
  	 	return isOperational;
  	 }

	 //--------------------
	 
	 void setResourceChangeHook(bool (*resourceChangedHook_)())
	 {
	 	this->resourceChangedHook = resourceChangedHook_;
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

  	 	isOperational = false;

  	 	msg.requester_name = name_;
  	 	msg.is_operational = false;

  	 	resourceRequesterStatePort.write(msg);

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
