#ifndef OROCOS_SWEETIE_BOT_RESOURCE_CLIENT_USAGE_EXAMPLE_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_RESOURCE_CLIENT_USAGE_EXAMPLE_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <sweetie_bot_resource_control/sweetie_bot_resource_control_service.hpp>

#include <string>

class ResourceClientExample : public RTT::TaskContext
{
  protected:
	 // maps a resource that is absolutely essential to the component
	 // to a priority in range [0:1] that indicates how important this
	 // resource is to this user among all possible other users
	 std::map<std::string, double> resourcesRequiredPrimary;

	 // the same as resourcesRequiredPrimary, but the resources here
	 // are secondary to the component and it can function without them
	 // (but it would still like to get ahold of them)
	 std::map<std::string, double> resourcesRequiredAuxiliary;

	 boost::shared_ptr<ResourceClient> resource_client;
	 
	 void initResources();
	 void doUsefulStuff();

	 bool isActive;

  public:
    ResourceClientExample(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    bool activate();

    bool resourceChangedHook();
};
#endif
