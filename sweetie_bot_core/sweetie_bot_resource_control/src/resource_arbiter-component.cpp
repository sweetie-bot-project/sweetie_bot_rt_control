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
		.doc("Publishes a list of all resources and their current owners.");
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
	resource_assigment.clear();
	for(std::vector<std::string>::iterator name = resource_list.begin(); name != resource_list.end(); name++)
	{
		resource_assigment.insert( ResourceToOwnerMap::value_type(*name, "none") ); // add a free resource
	}
	// allocates buffers
	assigment_msg.request_ids.reserve(max_requests_per_cycle);
	assigment_msg.resources.reserve(resource_assigment.size());
	assigment_msg.owners.reserve(resource_assigment.size());
	request_msg.resources.reserve(resource_assigment.size());
	// set data samples
	assigment_port.setDataSample(assigment_msg);

	log(INFO) << "ResourceArbiter is configured !" << RTT::endlog();
	return true;
}

bool ResourceArbiter::startHook() 
{
	assigment_msg.request_ids.clear();
	log(INFO) << "ResourceArbiter is started !" << RTT::endlog();
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

	// Use msg buffer to store pending requests ids
	{ 
		auto it = assigment_msg.request_ids.begin();
		for(; it != assigment_msg.request_ids.end(); it++) {
			if ( (*it & 0xffff) == (resourceRequestMsg.request_id & 0xffff) ) break;
		} 
		if (it != assigment_msg.request_ids.end()) *it = resourceRequestMsg.request_id; // replace old pending request by new
		else assigment_msg.request_ids.push_back(resourceRequestMsg.request_id); // add new pending request
	}

	// Now process request and change resource assigment. Direct use of assigment_msg is inconvinent, so ResourceToOwnerMap is used.
	for (int i = 0; i < resourceRequestMsg.resources.size(); i++) {
		// check if the resource actually exists
		ResourceToOwnerMap::iterator res_owner_pair = resource_assigment.find(resourceRequestMsg.resources[i]);
		if (res_owner_pair != resource_assigment.end()) {
			// there is such resource
			log(DEBUG) << "ResourceArbiter: resource `" << res_owner_pair->first << "` is transferred from `" << res_owner_pair->second
				<< "` to `" << resourceRequestMsg.requester_name << "`"<< RTT::endlog();
			// transfer resource
			res_owner_pair->second = resourceRequestMsg.requester_name;
		}
		else {
			// there is no such resource
			log(WARN) << "ResourceArbiter: resource does not exist: " << resourceRequestMsg.resources[i]  << RTT::endlog();
			log(DEBUG) << "ResourceArbiter: resources `" << resourceRequestMsg.resources[i]  << "` is transferred from [none]" 
				<< " to `" << resourceRequestMsg.requester_name << "`"<< RTT::endlog();
			// allocate resource
			resource_assigment[resourceRequestMsg.resources[i]] = resourceRequestMsg.requester_name;
		}
	} 
}

/* 
 * Send ResourceAssignment message with current assigment to clients.
 */
void ResourceArbiter::sendResourceAssigmentMsg() {
	// inform components about resource assigments

	// do not touch resource_ids --- it contains pending request list
	assigment_msg.resources.clear();
	assigment_msg.owners.clear();
	for (ResourceToOwnerMap::iterator it = resource_assigment.begin(); it != resource_assigment.end(); ++it) {
		assigment_msg.resources.push_back(it->first);
		assigment_msg.owners.push_back(it->second);
	}
	assigment_port.write(assigment_msg);
}

/* Process resource requester state report.
 *
 * Reallocate resources if needed (usually, just free the resources of a deactivated component).
 *
 * @param resourceRequesterStateMsg ResourceRequesterState Message that notifies the arbitrator about 
 * a component's change of state.
 */
void ResourceArbiter::processResourceRequesterState(ResourceRequesterState& resourceRequesterStateMsg)
{
	log(INFO) << "ResourceArbiter: requester state change: `" << resourceRequesterStateMsg.requester_name  << "` request_id = " << resourceRequesterStateMsg.request_id <<
		" is_operational = " << (int) resourceRequesterStateMsg.is_operational << RTT::endlog();

	// delete coresponding request from pending requests list
	auto found = std::find(assigment_msg.request_ids.begin(), assigment_msg.request_ids.end(), resourceRequesterStateMsg.request_id);
	if (found != assigment_msg.request_ids.end()) assigment_msg.request_ids.erase(found);

	if (!resourceRequesterStateMsg.is_operational) {
		// free resources of a deactivated component
		for (ResourceToOwnerMap::iterator it = resource_assigment.begin(); it != resource_assigment.end(); ++it) {
			if (it->second == resourceRequesterStateMsg.requester_name) {
				log(DEBUG) << "ResourceArbiter: resource released (component deactivation): " << it->first << RTT::endlog();
				it->second = "none";
			}
		}
	}
}

/* Assign all resources to the component 'name' or to no one
 * when the name equals "none".
 */
void ResourceArbiter::assignAllResourcesTo(std::string name)
{
	for(ResourceToOwnerMap::iterator it = resource_assigment.begin(); it != resource_assigment.end(); ++it)
		it->second = name;
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
		processResourceRequesterState(requester_state_msg);
		assigment_changed = true;
	}

	// Inform clients
	if (assigment_changed) {
		sendResourceAssigmentMsg();

		if (log(DEBUG)) {
			log() << "Pending request_ids: [ ";
			for (auto it = assigment_msg.request_ids.begin(); it != assigment_msg.request_ids.end(); it++) log() << *it << ", ";
			log() << "]" << RTT::endlog();
			log() << "ResourceAssignment: [ ";
			for (ResourceToOwnerMap::const_iterator it = resource_assigment.begin(); it != resource_assigment.end(); it++) 
				log() << it->first << ": " << it->second << ", ";
			log() << "]" << RTT::endlog();
		}
	}
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
