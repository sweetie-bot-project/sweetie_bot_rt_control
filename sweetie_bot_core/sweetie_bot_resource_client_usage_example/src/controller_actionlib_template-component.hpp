#ifndef OROCOS_SWEETIE_BOT_RESOURCE_CLIENT_USAGE_EXAMPLE_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_RESOURCE_CLIENT_USAGE_EXAMPLE_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <rtt_actionlib/rtt_actionlib.h>
#include <rtt_actionlib/rtt_action_server.h>
#include <actionlib/action_definition.h>

#include <sweetie_bot_resource_control/sweetie_bot_resource_control_service.hpp>

#include <string>

class ControllerActionlibTemplate : public RTT::TaskContext
{
	protected:
		typedef actionlib::ServerGoalHandle<some_msgs::SomeAction> GoalHandle;
		ACTION_DEFINITION(some_msgs::SomeAction);	

	protected:
		// ActionLib variables
		rtt_actionlib::RTTActionServer<some_msgs::SomeAction> action_server;
		GoalHandle goal;

		// ResourceClient interface
		ResourceClientInterface2 * resource_client;

	public:
		ControllerActionlibTemplate(std::string const& name);

		bool resourceChangedHook();

		void goalCallback(GoalHandle gh);
		void cancelCallback(GoalHandle gh);

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};
#endif
