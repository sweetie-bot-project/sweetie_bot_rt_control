#ifndef OROCOS_SWEETIE_BOT_RESOURCE_CONTROL_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_RESOURCE_CONTROL_COMPONENT_HPP

#include <string>

#include <rtt/RTT.hpp>

#include <sweetie_bot_resource_control_msgs/ResourceRequest.h>
#include <sweetie_bot_resource_control_msgs/ResourceRequesterState.h>
#include <sweetie_bot_resource_control_msgs/ResourceAssignment.h>

using sweetie_bot_resource_control_msgs::ResourceRequest;
using sweetie_bot_resource_control_msgs::ResourceRequesterState;
using sweetie_bot_resource_control_msgs::ResourceAssignment;

typedef std::map<std::string, std::string> ResourceToOwnerMap;

class ResourceArbiter : public RTT::TaskContext
{
  protected:
	 RTT::InputPort<ResourceRequest> resourceRequestPort;
	 RTT::InputPort<ResourceRequesterState> resourceRequesterStatePort;

	 RTT::OutputPort<ResourceAssignment> resourceAssignmentPort;

	 /* Contains info about the current owner of a resource; 
	  * key is a resource, value is the owner; "none" string is a 'free' resource.
	  * If there is no such resource at all, then the key does not exist.
	 */ 
	 ResourceToOwnerMap resourceOwners; 

	 // TODO: priorities, queue of requesters, partial allocation and reallocation, etc.

	 void initResources();
	 void processResourceRequest(ResourceRequest& resourceRequestMsg);
	 void processResourceRequesterState(ResourceRequesterState& resourceRequesterStateMsg);

  public:
    ResourceArbiter(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void assignAllResourcesTo(std::string name);
};
#endif
