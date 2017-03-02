#ifndef OROCOS_SWEETIE_BOT_RESOURCE_CONTROL_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_RESOURCE_CONTROL_COMPONENT_HPP

#include <string>

#include <rtt/RTT.hpp>

#include <orocos/sweetie_bot_resource_control_msgs/typekit/ResourceRequest.h>
#include <orocos/sweetie_bot_resource_control_msgs/typekit/ResourceRequesterState.h>
#include <orocos/sweetie_bot_resource_control_msgs/typekit/ResourceAssignment.h>

#include <sweetie_bot_logger/logger.hpp>

using sweetie_bot_resource_control_msgs::ResourceRequest;
using sweetie_bot_resource_control_msgs::ResourceRequesterState;
using sweetie_bot_resource_control_msgs::ResourceAssignment;

namespace sweetie_bot {

namespace motion {

class ResourceArbiter : public RTT::TaskContext
{
	public:
		typedef std::map<std::string, std::string> ResourceToOwnerMap;
		const unsigned int max_requests_per_cycle = 10;

	protected:
		// COMPONENT INTERFACE
		// Input ports
		RTT::InputPort<ResourceRequest> request_port;
		RTT::InputPort<ResourceRequesterState> requester_status_port;
		// Output ports
		RTT::OutputPort<ResourceAssignment> assigment_port;
		// Properites
		std::vector<std::string> resource_list;
		// Operations: provided
	public:
		void assignAllResourcesTo(std::string name);

	protected:
		// COMPONENT STATE
		/* 
		 * Contains info about the current owner of a resource; 
		 * key is a resource, value is the owner; "none" string is a 'free' resource.
		 * If there is no such resource at all, then the key does not exist.
		 */ 
		ResourceToOwnerMap resource_assigment; 
		// port buffers
		ResourceAssignment assigment_msg;
		ResourceRequest request_msg;
		ResourceRequesterState requester_state_msg;

#ifdef SWEETIEBOT_LOGGER
		logger::SWEETIEBOT_LOGGER log;
#else
		logger::LoggerRTT log;
#endif
		// TODO: priorities, queue of requesters, partial allocation and reallocation, etc.

	protected:
		void processResourceRequest(ResourceRequest& resourceRequestMsg);
		void processResourceRequesterState(ResourceRequesterState& resourceRequesterStateMsg);
		void sendResourceAssigmentMsg();

	public:
		ResourceArbiter(std::string const& name);

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
};

} // namespace motion

} // namespace sweetie_bot

#endif
