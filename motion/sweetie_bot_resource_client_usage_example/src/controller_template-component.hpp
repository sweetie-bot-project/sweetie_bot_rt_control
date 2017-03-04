#ifndef OROCOS_SWEETIE_BOT_RESOURCE_CLIENT_USAGE_EXAMPLE_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_RESOURCE_CLIENT_USAGE_EXAMPLE_COMPONENT_HPP

#include <vector>
#include <string>

#include <rtt/Component.hpp>

#include <std_srvs/SetBool.h>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_orocos_misc/simple_action_server.hpp>
#include <sweetie_bot_resource_control/resource_client.hpp>

namespace sweetie_bot {
namespace motion {
namespace controller {


class ControllerTemplate : public RTT::TaskContext
{
	protected:
		// COMPONENT INTERFACE
		//
		// PORTS: input
		// PORTS: output
		// PROPERTIES
		std::vector<std::string> resources_required;
	protected:
		// OPERATIONS: provides
		bool rosSetOperational(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
		// OPERATIONS: requires
		// SERVICES: provides
		// SERVICES: required
		// SERVICES: internal interface
		sweetie_bot::motion::ResourceClientInterface * resource_client;

	protected:
		// COMPONENT STATE
		// ports buffers
		
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif

	public:
		ControllerTemplate(std::string const& name);

		bool resourceChangedHook();

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
