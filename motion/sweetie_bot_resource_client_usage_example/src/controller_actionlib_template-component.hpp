#ifndef OROCOS_SWEETIE_BOT_RESOURCE_CLIENT_USAGE_EXAMPLE_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_RESOURCE_CLIENT_USAGE_EXAMPLE_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <string>

#include <sweetie_bot_resource_control_msgs/MoveManuallyAction.h>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_resource_control/resource_client.hpp>
#include <sweetie_bot_resource_control/simple_action_server.hpp>


namespace sweetie_bot {
namespace motion {
namespace controller {

class ControllerActionlibTemplate : public RTT::TaskContext
{
	protected:
		// Goal, Feedback, Result typedefs
		ACTION_DEFINITION(sweetie_bot_resource_control_msgs::MoveManuallyAction);	

	protected:
		// COMPONENT INTERFACE
		//
		// PORTS: input
		// PORTS: output
		// PROPERTIES
		std::vector<std::string> resources_required;
	protected:
		// OPERATIONS: provides
		// OPERATIONS: requires
		// SERVICES: 
		sweetie_bot::motion::ResourceClientInterface * resource_client;

	protected:
		// ACTIONLIB:
		// Simple action server
		OrocosSimpleActionServer<sweetie_bot_resource_control_msgs::MoveManuallyAction> action_server;
		// enable actionlib event ports in stopped state
        bool dataOnPortHook(RTT::base::PortInterface *portInterface);
		// new pending goal is received
		void newGoalHook(const Goal& goal);
		// active goal is being canceled
		void cancelGoalHook();
		// buffers (also can be created dynamically)
		Goal goal; // maybe shared_ptr is better?
		Result result;
		Feedback feedback;

	protected:
		// COMPONENT STATE
		long long start_time;
		// ports buffers
		
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif

	public:
		ControllerActionlibTemplate(std::string const& name);

		bool resourceChangedHook();
		void stopOperationalHook();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

#endif
