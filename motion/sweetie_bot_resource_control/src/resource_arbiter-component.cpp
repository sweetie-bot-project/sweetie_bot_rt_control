#include "resource_arbiter-component.hpp"

#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include <vector>
#include <iostream>

namespace sweetie_bot {

namespace motion {

//const int ResourceArbiter::max_requests_per_cycle = 5; /**< Arbiter will process no more then 5 requests in updateHook. */

ResourceArbiter::ResourceArbiter(std::string const& name) :
	TaskContext(name, RTT::base::TaskCore::PreOperational),
	log(logger::categoryFromComponentName(name))
{
	// PORTS
	this->ports()->addPort("out_resource_assigment", assigment_port)
		.doc("Publish a list of all resources and their current owners.");
	this->ports()->addPort("out_controllers_state", controllers_state_port)
		.doc("States of all registered controllers.");
	this->ports()->addEventPort("in_resource_request", request_port)
		.doc("Resource arbiter receives requests for resource allocation via this port.");
	this->ports()->addEventPort("in_resource_requester_status", requester_status_port)
		.doc("Resource arbiter is notified about client state changes  a component is active or has deactivated "
				"and released all its resources.");
	// PROPERTIES
	this->addProperty("resources", resource_list)
		.doc("List of controlled resources.");
	// OPERATIONS
	this->addOperation("assignAllResourcesTo", &ResourceArbiter::assignAllResourcesTo, this, RTT::OwnThread)
		.doc("Assign all resources to the component 'name' or to no one when the name equals \"none\"")
		.arg("name", "Name of component");

	log(INFO) << "ResourceArbiter constructed !" << RTT::endlog();
}

bool ResourceArbiter::configureHook() 
{
	// clear resource assigment
	this->seq = 0;
	resources.clear();
	clients.clear();
	// create a default resource list
	for(std::vector<std::string>::iterator name = resource_list.begin(); name != resource_list.end(); name++)
	{
		resources.insert( Resources::value_type(*name, ResourceInfo(resources.size(), "none")) ); // add a free resource
	}
	// allocates buffers
	assigment_msg.request_ids.reserve(max_requests_per_cycle);
	assigment_msg.resources.reserve(resources.size());
	assigment_msg.owners.reserve(resources.size());
	request_msg.resources.reserve(resources.size());
	// TODO controllers_state_msg
	// set data samples
	assigment_port.setDataSample(assigment_msg);
	// TODO controllers_state_port

	log(INFO) << "ResourceArbiter is configured !" << RTT::endlog();
	return true;
}

bool ResourceArbiter::startHook() 
{
	log(INFO) << "ResourceArbiter is started !" << RTT::endlog();
	return true;
}

/* Reallocates resources when a component asks for them.
 *
 * Current implementation gives the required resources to the last requester and
 * ignores priorities.
 *
 * @param resourceRequestMsg ResourceRequest Message, that contains the requester's
 * name, a list of resources requested and their priorities for the component.
 */
void ResourceArbiter::processResourceRequest(ResourceRequest& resourceRequestMsg)
{
	if (log(INFO)) {
		log() << "ResourceArbiter: `" << resourceRequestMsg.requester_name << "` request " << resourceRequestMsg.request_id << " [";
		for(auto it = resourceRequestMsg.resources.begin(); it != resourceRequestMsg.resources.end(); it++) log() << *it << ", ";
		log() << " ]" << RTT::endlog();
	}

	// Now process request and change resource assigment. 
	ResourceSet requested_resources;
	for (auto resource = resourceRequestMsg.resources.begin(); resource != resourceRequestMsg.resources.end(); resource++) {
		// check if the resource actually exists
		Resources::iterator res_info_pair = resources.find(*resource);
		if (res_info_pair != resources.end()) {
			// there is such resource
			log(DEBUG) << "ResourceArbiter: resource `" << res_info_pair->first << "` is transferred from `" << res_info_pair->second.owner
				<< "` to `" << resourceRequestMsg.requester_name << "`"<< RTT::endlog();
			// transfer resource
			res_info_pair->second.owner = resourceRequestMsg.requester_name;
		}
		else {
			// there is no such resource
			log(WARN) << "ResourceArbiter: resource does not exist: " << *resource  << RTT::endlog();
			log(DEBUG) << "ResourceArbiter: resources `" << *resource  << "` is transferred from [none]" 
				<< " to `" << resourceRequestMsg.requester_name << "`"<< RTT::endlog();
			// allocate resource and set it index and owner
			if (resources.size() > ResourceSet::max_resource_index) {
				log(WARN) << "ResourceArbiter: to many resources, unable to implement reposession correctly." << RTT::endlog();
			}
			res_info_pair = resources.insert( Resources::value_type( *resource, ResourceInfo(resources.size(), resourceRequestMsg.requester_name) ) ).first;
		}
		requested_resources.insertByIndex(res_info_pair->second.index);
	} 
	// Update client iformation.
	// Register new client if necessary.
	ClientInfo& requester = clients[resourceRequestMsg.requester_name];
	requester.state |= ResourceClient::PENDING; // NONOPERATIONAL -> PENDING or OPERATIONAL -> OPERATIONAL_PENDING
	requester.last_request = requested_resources;
	requester.request_id = resourceRequestMsg.request_id;
	requester.seq = seq++; // assign and increase request counter
}

/* 
 * Send ResourceAssignment message with current assigment to clients.
 */
void ResourceArbiter::sendResourceAssigmentMsg() 
{
	// inform components about resource assigments
	if (assigment_msg.resources.size() != resources.size()) {
		// recreate all fields
		assigment_msg.resources.clear();
		assigment_msg.owners.clear();
		for (Resources::const_iterator it = resources.begin(); it != resources.end(); ++it) {
			assigment_msg.resources.push_back(it->first);
			assigment_msg.owners.push_back(it->second.owner);
		}
	}
	else {
		// renew only owners
		assigment_msg.owners.clear();
		for (Resources::const_iterator it = resources.begin(); it != resources.end(); ++it) {
			assigment_msg.owners.push_back(it->second.owner);
		}
	}
	// fill up request_id of pending components
	assigment_msg.request_ids.clear();
	for (Clients::const_iterator it = clients.begin(); it != clients.end(); ++it) {
		// add request_ids of controllers with pending requests
		if (it->second.state & ResourceClient::PENDING) assigment_msg.request_ids.push_back( it->second.request_id );
	}
	// send message
	assigment_port.write(assigment_msg);
}

void ResourceArbiter::sendControllersStateMsg() 
{
	if (controllers_state_msg.name.size() != clients.size()) {
		unsigned int sz = clients.size();
		// change array sizes
		controllers_state_msg.name.clear();
		controllers_state_msg.name.reserve(sz);
		controllers_state_msg.request_id.resize(sz);
		controllers_state_msg.state.resize(sz);
		// assign names
		for(Clients::const_iterator it = clients.begin(); it != clients.end(); it++) {
			controllers_state_msg.name.push_back(it->first);
		}
	}
	// copy states and request_id
	int index = 0;
	for(Clients::const_iterator it = clients.begin(); it != clients.end(); it++) {
		controllers_state_msg.state[index] = it->second.state;
		controllers_state_msg.request_id[index] = it->second.request_id;
		index++;
	}
	// send message
	controllers_state_port.write(controllers_state_msg);
}

/* Process resource requester state report.
 *
 * Reallocate resources if needed (usually, just free the resources of a deactivated component).
 *
 * @param resourceRequesterStateMsg ResourceRequesterState Message that notifies the arbitrator about 
 * a component's change of state.
 */
bool ResourceArbiter::processResourceRequesterState(ResourceRequesterState& resourceRequesterStateMsg)
{
	log(INFO) << "ResourceArbiter: requester state change: `" << resourceRequesterStateMsg.requester_name  << "` request_id = " << resourceRequesterStateMsg.request_id <<
		" is_operational = " << (int) resourceRequesterStateMsg.is_operational << RTT::endlog();

	// Update client iformation.
	// Register new client if necessary.
	bool assigment_changed = false;
	ClientInfo& client = clients[resourceRequesterStateMsg.requester_name];

	// Ignore message if it does not correspond the last ResourceRequest issuied by the client.
	if (client.request_id > resourceRequesterStateMsg.request_id) {
		return false;
	}
	
	if (resourceRequesterStateMsg.is_operational) {
		client.state = ResourceClient::OPERATIONAL;
	}	
	else {
		client.state = ResourceClient::NONOPERATIONAL;
		// reassign resources of a deactivated component
		for (Resources::iterator resource = resources.begin(); resource != resources.end(); resource++) {
			if (resource->second.owner == resourceRequesterStateMsg.requester_name) {
				assigment_changed = true;
				// try to find component which should repossess resource
				Clients::iterator repossessor = clients.end();
				for (Clients::iterator client = clients.begin(); client != clients.end(); client++) {
					// it must be OPERATIONAL and contain resource in its last request
					if ( (client->second.state & ResourceClient::OPERATIONAL) && client->second.last_request.findByIndex(resource->second.index) ) {
						// it must have maximal sequence number
						if ( repossessor == clients.end() || repossessor->second.seq < client->second.seq ) {
							repossessor = client;
						}
					}
				}
				if (repossessor != clients.end()) resource->second.owner = repossessor->first;
				else resource->second.owner = "none";
				log(DEBUG) << "ResourceArbiter: resource `" << resource->first << "` released (component deactivation) and assigned to `" << resource->second.owner <<  "`." << RTT::endlog();
			}
		}
	}
	return assigment_changed;
}

/* Assign all resources to the component 'name' or to no one
 * when the name equals "none".
 */
void ResourceArbiter::assignAllResourcesTo(std::string name)
{
	for(Resources::iterator it = resources.begin(); it != resources.end(); ++it)
		it->second.owner = name;
	// inform clients
	sendResourceAssigmentMsg();
}

void ResourceArbiter::updateHook()
{
	bool assigment_changed = false;

	// Process up to max_requests_per_cycle ResourceRequests, store requesteres name
	for(int req_count = 0; req_count < max_requests_per_cycle; req_count++) {
		if (request_port.read(request_msg, false) == RTT::NewData) {
			processResourceRequest(request_msg);
			assigment_changed = true;
		}
		else break;
	}

	// Process ResourceRequesterState.
	while (requester_status_port.read(requester_state_msg) == RTT::NewData) {
		if (processResourceRequesterState(requester_state_msg)) assigment_changed = true;
	}

	// Inform clients
	if (assigment_changed) {
		sendResourceAssigmentMsg();

		if (log(DEBUG)) {
			log() << "Pending request_ids: [ ";
			for (auto it = assigment_msg.request_ids.begin(); it != assigment_msg.request_ids.end(); it++) log() << *it << ", ";
			log() << "]" << RTT::endlog();
			log() << "ResourceAssignment: [ ";
			for (Resources::const_iterator it = resources.begin(); it != resources.end(); it++) 
				log() << it->first << ": " << it->second.owner << ", ";
			log() << "]" << RTT::endlog();
			log() << "ActiveClients: [ ";
			for (Clients::const_iterator it = clients.begin(); it != clients.end(); it++) {
				if (it->second.state != ResourceClient::NONOPERATIONAL)
					log() << it->first << ": state=" << it->second.state << " req_id=" << it->second.request_id << " seq=" << it->second.seq  <<", ";
			}
			log() << "]" << RTT::endlog();
		}
	}
	sendControllersStateMsg();
}

void ResourceArbiter::stopHook() 
{
	log(INFO) << "ResourceArbiter is stopped !" << RTT::endlog();
}

} // namespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ResourceArbiter)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::ResourceArbiter)
