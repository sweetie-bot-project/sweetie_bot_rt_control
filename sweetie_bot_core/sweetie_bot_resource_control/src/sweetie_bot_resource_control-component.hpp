#ifndef OROCOS_SWEETIE_BOT_RESOURCE_CONTROL_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_RESOURCE_CONTROL_COMPONENT_HPP

#include <string>

#include <rtt/RTT.hpp>

#include <sweetie_bot_resource_control_msgs/ResourceRequest.h>
#include <sweetie_bot_resource_control_msgs/ResourceReply.h>
#include <sweetie_bot_resource_control_msgs/ResourceReleased.h>
#include <sweetie_bot_resource_control_msgs/ResourceDemandRelease.h>

using sweetie_bot_resource_control_msgs::ResourceRequest;
using sweetie_bot_resource_control_msgs::ResourceReply;
using sweetie_bot_resource_control_msgs::ResourceReleased;
using sweetie_bot_resource_control_msgs::ResourceDemandRelease;

class ResourceControl : public RTT::TaskContext
{
  protected:
	 RTT::InputPort<ResourceRequest> resourceRequestPort;
	 RTT::OutputPort<ResourceReply> resourceReplyPort;

	 RTT::InputPort<ResourceReleased> resourceReleasedPort;
	 RTT::OutputPort<ResourceDemandRelease> resourceDemandReleasePort;

	 /* Contains info about the current owner of a resource; 
	  * string is a resource, int is the owner; 0 is a 'free' resource.
	  * If there is no such resource at all, then the key does not exist.
	 */ 
	 std::map<std::string, int> resourceOwners; 

	 void initResources(); // gets a list of resources and maps them to 0
	 void processResourceRequest(ResourceRequest& resourceRequestMsg);
	 void processResourceReleased(ResourceReleased& resourceReleasedMsg);

  public:
    ResourceControl(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
