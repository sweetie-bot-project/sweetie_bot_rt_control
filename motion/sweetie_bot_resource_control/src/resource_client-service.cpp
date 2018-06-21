#include "sweetie_bot_resource_control/resource_client.hpp"

#include <functional>

#include <rtt/RTT.hpp>
#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

#include <sweetie_bot_resource_control_msgs/typekit/ResourceRequest.h>
#include <sweetie_bot_resource_control_msgs/typekit/ResourceRequesterState.h>
#include <sweetie_bot_resource_control_msgs/typekit/ResourceAssignment.h>

#include <sweetie_bot_logger/logger.hpp>

using namespace RTT;
using namespace std;

using sweetie_bot_resource_control_msgs::ResourceRequest;
using sweetie_bot_resource_control_msgs::ResourceRequesterState;
using sweetie_bot_resource_control_msgs::ResourceAssignment;

namespace sweetie_bot {

namespace motion {

/**
 * An example service which can be loaded in a component.
 */
class ResourceClientService : public ResourceClientInterface, public RTT::Service 
{
	protected:
		/**
		 * Resources set. Element presents if it was requsted, value is true if it is owned.
		 */
		typedef std::map<std::string, bool> ResourceSet;
		typedef int ResourceClientState;

	protected:
		// SERVICE INTERFACE
		// PORTS
		RTT::OutputPort<ResourceRequest> request_port;
		RTT::OutputPort<ResourceRequesterState> requester_state_port;
		RTT::InputPort<ResourceAssignment> assignment_port;

	protected:

		/**
		 * Owner component name and unique client_id
		 */
		std::string owner_name;
		unsigned int client_id; 
		unsigned int request_counter;

		// SERVICE STATE
		/* 
		 * Resource client state: NONOPERATIONAL, PENDING, OPERATIONAL.
		 */
		ResourceClientState state;

		/* 
		 * Hook that is provided by the controller that is using this plugin to determine
		 * whether it can work with the given resources. 
		 *
		 * @return bool True when the controller is active (can function), false otherwise.
		 */
		boost::function<bool()> resourceChangeHook;
		//OperationCaller<bool()> resourceChangeHook_call;
	
		/*
		 * Hook is called when client enters NONOPERATIONAL state.
		 */
		boost::function<void()> stopOperationalHook;
		//OperationCaller<void()> stopOperationalHook_call;

		/**
		 * Sets with assigned and requested resources.
		 */
		ResourceSet resources;

		// port buffers
		ResourceAssignment assignment_msg;

#ifdef SWEETIEBOT_LOGGER
		logger::SWEETIEBOT_LOGGER log;
#else
		logger::LoggerRTT log;
#endif

	protected:

		/* Detects if the requested resources were allocated to this component and replies
		 * with an activation message if they were, with a deactivation message otherwise.
		 *
		 * @param msg ResourceAssignment Message from the resource arbitrator the contains 
		 * all resources and indicates who they were given to.
		 */
		void processResourceAssignment(ResourceAssignment &msg)
		{
			switch (state) {	
				case ResourceClient::NONOPERATIONAL:
					// skip message in NONOPERATIONAL state
					return;

				case ResourceClient::PENDING:
				case ResourceClient::OPERATIONAL_PENDING:
					// in PENDING state skip messages until our request is processed
					{
						bool last_request_is_processed = false;
						for(auto it = msg.request_ids.begin(); it != msg.request_ids.end(); it++) {
							if ( (0xffff & *it) == client_id  && (*it >> 16) >= request_counter ) {
								last_request_is_processed = true;
								break;
							}
						}
						if (!last_request_is_processed) {
							log(INFO) << "`" << owner_name << "` ResourceService: ResourceAssignment skipped, state: " << state << endlog();
							return;
						}
					}

				case ResourceClient::OPERATIONAL:
					// process all assigmetns
					break;
			}

			if (msg.resources.size() != msg.owners.size()) {
				log(ERROR) << "[" << owner_name << "] ResourceService: invalid ResourceAssignment message." << endlog();
				return;
			}

			// modify lis of owned resources
			// TODO bug: additional resource is not visible
			for(auto it = resources.begin(); it != resources.end(); it++) it->second = false;
			for(int i = 0; i < msg.resources.size(); i++) {
				if (msg.owners[i] == owner_name) { 
					// create resource or mark it `owned`
					resources[msg.resources[i]] = true;
				}
			}
			if (log(DEBUG)) {
				log() << "[" << owner_name << "] ResourceService: owns [";
				for(ResourceSet::const_iterator i = resources.begin(); i != resources.end(); i++) if (i->second) log() << i->first << ", ";
				log() << "]" << endlog();
			}
			// ask the controller if it is able function with these resources
			bool is_operational;
			if (resourceChangeHook) {
				is_operational = resourceChangeHook();
			}
			// TODO OperationCaller interface
			else {
				// check if all requested resources presents
				is_operational = true;
				for(ResourceSet::const_iterator i = resources.begin(); i != resources.end(); i++) {
					is_operational = is_operational && i->second;
					if (!is_operational) break;
				}
				if (!is_operational) {
					// clear own flags
					for(ResourceSet::iterator i = resources.begin(); i != resources.end(); i++) {
						i->second = false;
					}
				}
			}
			// determine state transition
			if (is_operational) {
				ResourceClientState prev_state = state;
				// new state
				state = ResourceClient::OPERATIONAL;
				// announce state
				if (state != prev_state) {
					ResourceRequesterState requester_state_msg;
					requester_state_msg.request_id = client_id + (request_counter << 16);
					requester_state_msg.requester_name = owner_name;
					requester_state_msg.is_operational = is_operational;
					requester_state_port.write(requester_state_msg);
				}
				log(INFO) << "[" << owner_name << "] ResourceService: ResourceAssignment processed, state: " << prev_state << " -> " << state << endlog();
			}
			else {
				log(INFO) << "[" << owner_name << "] ResourceService: ResourceAssignment processed, state: " << state << " -> " << ResourceClient::NONOPERATIONAL << endlog();
				stopOperational();
			}
		}


	public:
		ResourceClientService(TaskContext* owner) :
			Service("resource_client", owner),
			state(ResourceClient::NONOPERATIONAL),
			log(logger::getDefaultCategory("swetie_bot.motion") + ".resource_control")
			//resourceChangeHook_call("resourceChangeHook"),
			//stopOperationalHook_call("stopOperationalHook")
		{
			doc("Client plugin for basic ResourceArbiter (without priorities).");

			// PORTS
			this->addPort("out_resource_request", request_port).
				doc("Send ResourceRequest to ResourceArbiter component.");
			this->addPort("out_resource_requester_status", requester_state_port).
				doc("Send information about client state cahnge to ResourceArbiter component.");
			this->addEventPort("in_resource_assigment", assignment_port).
				doc("Receive resource assignment changes from ResourceArbiter component.");

			// OPERATIONS
			/*this->addOperation("resourceChangeRequest", &ResourceClientService::resourceChangeRequest, this, OwnThread).
				doc("Request ResourceArbiter for given list of resources.").
				arg("resources", "List of requested resources");
			this->addOperation("stopOperational", &ResourceClientService::stopOperational, this, OwnThread).
				doc("Exit operational state. Inform ResourceArbiter and release all resources.");*/
			this->addOperation("isOperational", &ResourceClientService::isOperational, this).
				doc("Returns true if resource client is in operational state (shortcut for getState() & OPERATIONS).");
			this->addOperation("isPending", &ResourceClientService::isPending, this).
				doc("Returns true if resource client is waiting for arbiter responce (sortcut for getState() & PENDING).");
			this->addOperation("getState", &ResourceClientService::getState, this).
				doc("Returns state of ResourceClient: 0=NONOPERATIONAL, 1=PENDING, 2=OPERATIONAL, 3=OPERATIONAL_PENDING");
			this->addOperation("hasResource", &ResourceClientService::hasResource, this, OwnThread).
				doc("Check is resource client owns a resource.").
				arg("resource", "Resource name.");
			this->addOperation("hasResources", &ResourceClientService::hasResources, this, OwnThread).
				doc("Check is resource client owns resources.").
				arg("resources", "Resource name list.");
			this->addOperation("listResources", &ResourceClientService::listResources, this).
				doc("Return list of owned resources.");
			/*this->addOperation("step", &ResourceClientService::step, this, OwnThread).
				doc("Process incomming resource assignment. Call this operation periodically.");*/

			if (!getOwner()) throw std::invalid_argument("ResourceClient: owner pointer is null.");

			owner_name = getOwner()->getName();
			client_id = 0xffff & hash<string>()(owner_name); // unique client id
			request_counter = 0;

			//resourceChangeHook_call = owner->provides()->getLocalOperation("resourceChangeHook");
			//resourceChangeHook_call.setCaller(owner->engine());
			//stopOperationalHook_call = owner->provides()->getLocalOperation("stopOperationalHook");
			//stopOperationalHook_call.setCaller(owner->engine());

			log(INFO) << "[" << owner_name << "] ResourceService is loaded!" << endlog();
		}

		bool isOperational() const
		{
			return state & ResourceClient::OPERATIONAL;
		}

		bool isPending() const
		{
			return state & ResourceClient::PENDING;
		}

		bool isNonOperational() const
		{
			return state == ResourceClient::NONOPERATIONAL;
		}

		int getState() const
		{
			return state;
		}

		void setResourceChangeHook(boost::function<bool()> resourceChangeHook_)
		{
			this->resourceChangeHook = resourceChangeHook_;
		}

		void setStopOperationalHook(boost::function<void()> stopOperationalHook_)
		{
			this->stopOperationalHook = stopOperationalHook_;
		}

		unsigned long resourceChangeRequest(const std::vector<std::string>& resources_requested)
		{
			ResourceRequest request_msg;

			if (!request_port.connected()) {
				log(ERROR) << "[" << owner_name << "] ResourceService: request_port is not connected." << endlog();
				return false;
			}
			if (!assignment_port.connected()) {
				log(ERROR) << "[" << owner_name << "] ResourceService: assignment_port is not connected." << endlog();
				return false;
			}
			if (!requester_state_port.connected()) {
				log(ERROR) << "[" << owner_name << "] ResourceService: requester_state_port is not connected." << endlog();
				return false;
			}

			// clear incoming assigment_messages
			assignment_port.readNewest(assignment_msg);
			request_counter++;

			// send resource request
			request_msg.requester_name = owner_name;
			request_msg.request_id = client_id + ((unsigned long) request_counter << 16);
			request_msg.resources = resources_requested;

			request_port.write(request_msg);
			state = state | ResourceClient::PENDING; 

			// save resource list
			resources.clear();
			for (std::vector<std::string>::const_iterator it = resources_requested.begin(); 
					it != resources_requested.end(); ++it)
			{
				resources[*it] = false;
			}

			if (log(DEBUG)) {
				log() << "`" << owner_name << "` ResourceService: request " << request_msg.request_id << " [";
				for(ResourceSet::const_iterator i = resources.begin(); i != resources.end(); i++) log() << i->first << ", ";
				log() << "]" << endlog();
			}
			return request_msg.request_id;
		}

		bool stopOperational()
		{
			if (!request_port.connected()) {
				log(ERROR) << "[" << owner_name << "] ResourceService: requester_status_port is not connected." << endlog();
				return false;
			}

			ResourceRequesterState msg;
			msg.requester_name = owner_name;
			msg.request_id = client_id + ((unsigned long) request_counter << 16);
			msg.is_operational = false; 
			requester_state_port.write(msg);

			if (state != ResourceClient::NONOPERATIONAL) {
				log(INFO) << "[" << owner_name << "] ResourceService leaves operational state." << endlog();
				state = ResourceClient::NONOPERATIONAL;
				// call user hook
				if (stopOperationalHook) stopOperationalHook();
				//else if (stopOperationalHook_call.ready()) stopOperationalHook_call();
			}
			return true;
		}

		bool hasResource(const std::string& res) const
		{
			ResourceSet::const_iterator it = resources.find(res);
			return it != resources.end() && it->second;
		}

		bool hasResources(const std::vector<std::string>& res) const
		{
			for (int i = 0; i < res.size(); i++) {
				ResourceSet::const_iterator it = resources.find(res[i]);
				if (it == resources.end() || !it->second) return false;
			}
			return true;
		}

		std::vector<std::string> listResources() const
		{
			std::vector<std::string> owned_resources;
			for(ResourceSet::const_iterator it = resources.begin(); it != resources.end(); it++) {
				if (it->second) owned_resources.push_back(it->first);
			}
			return owned_resources;
		}

		void step() 
		{
			while (assignment_port.read(assignment_msg, false) == RTT::NewData) {
				processResourceAssignment(assignment_msg);
			}
		}
};

} // namespace motion 

} // namespace sweetie_bot

/* For consistency reasons, it's better to name the
 * service the same as in the class above.
 */
ORO_SERVICE_NAMED_PLUGIN(sweetie_bot::motion::ResourceClientService, "resource_client")
