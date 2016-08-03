#ifndef OROCOS_SWEETIE_BOT_GAIT_WALKING_3_PLUS_1_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_GAIT_WALKING_3_PLUS_1_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <string>

#include <sweetie_bot_resource_control_msgs/ResourceRequest.h>
#include <sweetie_bot_resource_control_msgs/ResourceReply.h>
#include <sweetie_bot_resource_control_msgs/ResourceReleased.h>
#include <sweetie_bot_resource_control_msgs/ResourceDemandRelease.h>

using sweetie_bot_resource_control_msgs::ResourceRequest;
using sweetie_bot_resource_control_msgs::ResourceReply;
using sweetie_bot_resource_control_msgs::ResourceReleased;
using sweetie_bot_resource_control_msgs::ResourceDemandRelease;

class GaitWalking3Plus1 : public RTT::TaskContext
{
  protected:
	 RTT::OutputPort<ResourceRequest> resourceRequestPort;
	 RTT::InputPort<ResourceReply> resourceReplyPort;

	 RTT::InputPort<ResourceDemandRelease> resourceDemandReleasePort;
	 RTT::OutputPort<ResourceReleased> resourceReleasedPort;

	 // maps a resource that is absolutely essential to the component
	 // to a priority in range [0:1] that indicates how important this
	 // resource is to this user among all possible other users
	 std::map<std::string, double> resourcesRequiredPrimary;

	 // the same as resourcesRequiredPrimary, but the resources here
	 // are secondary to the component and it can function wihthout them
	 // (but it would still like to get ahold of them)
	 std::map<std::string, double> resourcesRequiredAuxiliary;

	 std::map<std::string, bool> resourcesControlled; // a list of resources
		// that this user was given control of
	 
	 void initResources();
	 void calculateMovement();
	 
	 void processResourceReleaseDemand();
	 void processResourceGiven();

	 void appendResourceRequest(std::string name, double priority,
	 	  ResourceRequest& resourceRequestMsg);
	 void processResourceReply(ResourceReply& resourceReplyMsg);
	 void processResourceDemandRelease(ResourceDemandRelease& resourceDemandReleaseMsg);

  	 // TODO: move behaviours' ids somewhere to parameters or smth.
  	 static const int GAIT_WALKING_ID = 1;

  public:
    GaitWalking3Plus1(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
