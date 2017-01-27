#include <sweetie_bot_resource_control/resource_client-service.hpp>

#include <algorithm>

#include <rtt/RTT.hpp>
#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

#include <orocos/sweetie_bot_resource_control_msgs/typekit/ResourceRequest.h>
#include <orocos/sweetie_bot_resource_control_msgs/typekit/ResourceRequesterState.h>
#include <orocos/sweetie_bot_resource_control_msgs/typekit/ResourceAssignment.h>

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
		// port buffers
		ResourceAssignment assignment_msg;

	protected:
		// SERVICE INTERFACE
		// PORTS
		RTT::OutputPort<ResourceRequest> request_port;
		RTT::OutputPort<ResourceRequesterState> requester_state_port;
		RTT::InputPort<ResourceAssignment> assignment_port;

	protected:

		/**
		 * Owner component name.
		 */
		std::string owner_name;

		// SERVICE STATE
		/* 
		 * Resource client state: NONOPERATIONAL, PENDING, OPERATIONAL.
		 */
		ResourceClient::ResourceClientState state;

		/* 
		 * Hook that is provided by the controller that is using this plugin to determine
		 * whether it can work with the given resources. 
		 *
		 * @return bool True when the controller is active (can function), false otherwise.
		 */
		boost::function<bool()> resourceChangedHook;

		/**
		 * Sets with assigned and requested resources.
		 */
		ResourceSet resources;

#ifdef SWEETIEBOT_LOGGER
		SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::LoggerRTT log;
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
			if (state == ResourceClient::NONOPERATIONAL) return;

			if (msg.resources.size() != msg.owners.size()) {
				log(ERROR) << "[" << owner_name << "] ResourceService: invalid ResourceAssignment message." << endlog();
				return;
			}

			// modify lis of owned resources
			resources.clear();
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
			if (resourceChangedHook) {
				is_operational = resourceChangedHook();
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
				state = ResourceClient::OPERATIONAL;
			}
			else {
				switch (state) {
					case ResourceClient::OPERATIONAL:
						state = ResourceClient::NONOPERATIONAL;
						break;
					case ResourceClient::PENDING:
						if (std::find(msg.requesters.begin(), msg.requesters.end(), owner_name) != msg.requesters.end()) 
							state = ResourceClient::NONOPERATIONAL; // resource request was processed by arbiter
						//TODO transition PENDING -> NONOPERATIONAL after timeout
						break;
				}
			}
			// report state change
			log(INFO) << "[" << owner_name << "] ResourceService: ResourceAssignment processed, state = " << state << endlog();

			// announce state
			ResourceRequesterState requester_state_msg;
			requester_state_msg.requester_name = owner_name;
			requester_state_msg.state = state;
			requester_state_port.write(requester_state_msg);
		}


	public:
		ResourceClientService(TaskContext* owner) :
			Service("resource_client", owner),
			state(ResourceClient::NONOPERATIONAL),
			log("sweetie.motion.resource_control")
		{
			doc("Client plugin for basic ResourceArbiter (without priorities).");

			// PORTS
			this->addPort("out_resource_request", request_port).
				doc("Send ResourceRequest to ResourceArbiter component.");
			this->addPort("out_resource_requester_state", requester_state_port).
				doc("Send information about client state cahnge to ResourceArbiter component.");
			this->addEventPort("in_resource_assignment", assignment_port).
				doc("Receive resource assignment changes from ResourceArbiter component.");

			// OPERATIONS
			this->addOperation("requestResources", &ResourceClientService::requestResources, this).
				doc("Request ResourceArbiter for given list of resources.").
				arg("resources", "List of requested resources");
			this->addOperation("stopOperational", &ResourceClientService::stopOperational, this).
				doc("Exit operational state. Inform ResourceArbiter and release all resources.");
			this->addOperation("isOperational", &ResourceClientService::isOperational, this).
				doc("Returns true if resource client is in operational state.");
			this->addOperation("getState", &ResourceClientService::getState, this).
				doc("Returns state of ResourceClient: 0=NONOPERATIONAL, 1=PENDING, 2=OPERATIONAL.");
			this->addOperation("hasResource", &ResourceClientService::hasResource, this).
				doc("Check is resource client owns a resource.").
				arg("resource", "Resource name.");
			this->addOperation("hasResources", &ResourceClientService::hasResource, this).
				doc("Check is resource client owns resources.").
				arg("resources", "Resource name list.");
			this->addOperation("step", &ResourceClientService::step, this).
				doc("Process incomming resource assignment. Call this operation periodically.");

			if (getOwner()) {
				owner_name = getOwner()->getName();
			}
			log(INFO) << "[" << owner_name << "] ResourceService is loaded!" << endlog();
		}

		// Properties' getters and setters

		bool isOperational()
		{
			return state == ResourceClient::OPERATIONAL;
		}

		int getState()
		{
			return state;
		}

		void setResourceChangeHook(boost::function<bool()> resourceChangedHook_)
		{
			this->resourceChangedHook = resourceChangedHook_;
		}

		bool requestResources(const std::vector<std::string>& resources_requested)
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

			// clear incoming assigment_messages
			assignment_port.readNewest(assignment_msg);

			// send resource request
			request_msg.requester_name = owner_name;
			request_msg.resources = resources_requested;

			request_port.write(request_msg);

			// save resource list
			resources.clear();
			for (std::vector<std::string>::const_iterator it = resources_requested.begin(); 
					it != resources_requested.end(); ++it)
			{
				resources[*it] = false;
			}

			if (log(DEBUG)) {
				log() << "[" << owner_name << "] ResourceService: requests [";
				for(ResourceSet::const_iterator i = resources.begin(); i != resources.end(); i++) log() << i->first << ", ";
				log() << "]" << endlog();
			}
			return true;
		}

		bool stopOperational()
		{
			state = ResourceClient::NONOPERATIONAL;

			if (!request_port.connected()) {
				log(ERROR) << "[" << owner_name << "] ResourceService: requester_status_port is not connected." << endlog();
				return false;
			}

			ResourceRequesterState msg;
			msg.requester_name = owner_name;
			msg.state = state;
			requester_state_port.write(msg);

			log(INFO) << "[" << owner_name << "] ResourceService: exits opertional state.";

			return true;
		}

		bool hasResource(const std::string& res)
		{
			ResourceSet::const_iterator it = resources.find(res);
			return it != resources.end() && it->second;
		}

		bool hasResources(const std::vector<std::string>& res)
		{
			for (int i = 0; i < res.size(); i++) {
				ResourceSet::const_iterator it = resources.find(res[i]);
				if (it == resources.end() || !it->second) return false;
			}
			return true;
		}

		void step() 
		{
			if (assignment_port.read(assignment_msg, false) == RTT::NewData) {
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
