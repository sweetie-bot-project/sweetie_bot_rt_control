#ifndef OROCOS_SWEETIE_BOT_RESOURCE_CONTROL_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_RESOURCE_CONTROL_COMPONENT_HPP

#include <string>

#include <rtt/RTT.hpp>

#include <orocos/sweetie_bot_resource_control_msgs/typekit/ResourceRequest.h>
#include <orocos/sweetie_bot_resource_control_msgs/typekit/ResourceRequesterState.h>
#include <orocos/sweetie_bot_resource_control_msgs/typekit/ResourceAssignment.h>
#include <orocos/sweetie_bot_resource_control_msgs/typekit/ControllerState.h>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_resource_control/resource_client.hpp>

namespace sweetie_bot {

namespace motion {

class ResourceArbiter : public RTT::TaskContext
{
	public:
		typedef sweetie_bot_resource_control_msgs::ResourceRequest ResourceRequest;
		typedef sweetie_bot_resource_control_msgs::ResourceRequesterState ResourceRequesterState;
		typedef sweetie_bot_resource_control_msgs::ResourceAssignment ResourceAssignment;
		typedef sweetie_bot_resource_control_msgs::ControllerState ControllerState;

	protected:
		/** 
		 * Packed resource set representation.
		 */
		struct ResourceSet {
			static const unsigned int max_resource_index = 8*sizeof(unsigned long long) - 1;
			unsigned long long bitvector;

			ResourceSet() : bitvector(0) {}
			ResourceSet(const ResourceSet&) = default;
			void insertByIndex(unsigned int index) { bitvector |= 1ull << index; }
			void eraseByIndex(unsigned int index) { bitvector &= ~(1ull << index); }
			bool findByIndex(unsigned int index) { return bitvector & (1ull << index); }
		};

		/**
		 * ResourceClient information.
		 **/
		struct ClientInfo {
			ResourceClient::ResourceClientState state; /**< Client state. */
			ResourceSet last_request; /**< Last resource request. */
			unsigned int request_id; /**< ID of last resource request. */
			unsigned int seq; /**< Global sequence naumber of resource request. */

			ClientInfo() : state(ResourceClient::NONOPERATIONAL), request_id(0), seq(0) {}
			ClientInfo(const ClientInfo& client) = default;
			ClientInfo(ResourceClient::ResourceClientState _state, ResourceSet _last_request, unsigned int _request_id, unsigned int _seq) :
				state(_state), last_request(_last_request), request_id(_request_id), seq(_seq) {};
		};

		/**
		 *  Registered resource clients. 
		 */
		typedef std::map<std::string, ClientInfo> Clients; 

		/**
		 * Resource information.
		 **/
		struct ResourceInfo {
			unsigned int index; /**< Resource index to speedup set manipulations. It is equal to index of membership bit in ResourceSet.  */
			std::string owner; /**< Owner of resource or 'none'. */ 
			//Clients::iterator owner; /**< Owner of iterator in clients. */ 

			ResourceInfo(unsigned int _index, const std::string& _owner) :
				index(_index), owner(_owner) {}
			ResourceInfo(const ResourceInfo& resource) = default;
		};
		/**
		 * Registered resources.
		 */
		typedef std::map<std::string, ResourceInfo> Resources; 

	protected:
		// COMPONENT INTERFACE
		// Input ports
		RTT::InputPort<ResourceRequest> request_port;
		RTT::InputPort<ResourceRequesterState> requester_status_port;
		// Output ports
		RTT::OutputPort<ResourceAssignment> assigment_port;
		RTT::OutputPort<ControllerState> controllers_state_port;
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
		unsigned int seq; /**< Request sequence number. */
		const unsigned int max_requests_per_cycle = 10;

		Resources resources; /**< List of controlled resources. */
		Clients clients; /**< List of registered clients. */
		// port buffers
		ResourceAssignment assigment_msg;
		ResourceRequest request_msg;
		ResourceRequesterState requester_state_msg;
		ControllerState controllers_state_msg;

#ifdef SWEETIEBOT_LOGGER
		logger::SWEETIEBOT_LOGGER log;
#else
		logger::LoggerRTT log;
#endif
		// TODO: priorities, queue of requesters, partial allocation and reallocation, etc.

	protected:
		void processResourceRequest(ResourceRequest& resourceRequestMsg);
		bool processResourceRequesterState(ResourceRequesterState& resourceRequesterStateMsg);
		void sendResourceAssigmentMsg();
		void sendControllersStateMsg();

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
