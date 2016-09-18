#include "sweetie_bot_resource_control-component.hpp"

#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include <vector>
#include <iostream>

using RTT::Logger;

ResourceArbiter::ResourceArbiter(std::string const& name) : TaskContext(name)
{
  Logger::log(Logger::Info) << "ResourceArbiter constructed !" << Logger::endl;

  this->ports()->addPort("resource_assignment", resourceAssignmentPort)
	 .doc("Publishes a list of all resources and their current owners.");

  this->ports()->addEventPort("resource_request", resourceRequestPort)
	 .doc("Resource manager receives requests for resource allocation via this port.");

  this->ports()->addEventPort("resource_requester_state", resourceRequesterStatePort)
	 .doc("Resource manager is notified that a component is active or has deactivated "
	 	  "and released all its resources.");

  this->addOperation("assignAllResourcesTo", &ResourceArbiter::assignAllResourcesTo, this, RTT::OwnThread)
  	 .doc("Assign all resources to the component 'name' or to no one when the name equals \"none\"");
 
  initResources();
}

/* Gets a list of resources and maps them to "none" to indicate that they are there but
 * free.
 */
void ResourceArbiter::initResources()
{
  // TODO: make grouping resources possible
  // e.g. requesting "leg1" resource and converting that automatically 
  // to a "servo11", "servo12" request
  
  // TODO: move to the parameter server
  std::string tmp[] = {"leg1", "leg2", "leg3", "leg4", "servo1_1", 
  	 "servo1_2", "servo2_2", "eye1", "head"};
  std::vector<std::string> resources(tmp, tmp+sizeof(tmp) / sizeof(tmp[0]));
  for(std::vector<std::string>::iterator name = resources.begin(); name != resources.end(); name++)
  {
	 this->resourceOwners[*name] = "none"; // add a free resource
	 Logger::log(Logger::Info) << "Resource initialized: " << *name << Logger::endl;
  }
}

bool ResourceArbiter::configureHook()
{
  Logger::log(Logger::Info) << "ResourceArbiter configured !" <<Logger::endl;

  return true;
}

bool ResourceArbiter::startHook()
{
  Logger::log(Logger::Info) << "ResourceArbiter started !" <<Logger::endl;
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
  // a flag that indicates if any changes to resources' owners were made
  bool isOwnersChanged = false;  
  
  for (int i = 0; i < resourceRequestMsg.resources.size(); i++)
  {
 	 // check if the resource actually exists
 	 ResourceToOwnerMap::iterator ownedIt = resourceOwners.find(resourceRequestMsg.resources[i]);
	 if (ownedIt != resourceOwners.end())
	 {
		// there is such resource
		if (resourceOwners[resourceRequestMsg.resources[i]] == resourceRequestMsg.requester_name)
		{
		  // the requester already owns this resource (should be a warning or an error
		  // since this is a strange case)
		  
		  Logger::log(Logger::Warning) << "Resource [" << resourceRequestMsg.resources[i] << "] already owned by [" 
		  	 << resourceRequestMsg.requester_name << "]" << Logger::endl;
		} 
		else if (resourceOwners[resourceRequestMsg.resources[i]] == "none")
		{
		  // the resource is free and will be given to the requester
		  isOwnersChanged = true;

		  Logger::log(Logger::Info) << "Resource [" << resourceRequestMsg.resources[i] << "] given to ["
		  	 << resourceRequestMsg.requester_name << "]" << Logger::endl;
		} 
		else 
		{
		  // the resource is already owned by someone else 
		  // NOTE: it will still be given in this implementation
		  isOwnersChanged = true;

		  Logger::log(Logger::Info) << "Resource [" << resourceRequestMsg.resources[i] << "] is already owned by ["
		  	 << resourceOwners[resourceRequestMsg.resources[i]] << "] but will be reallocated" << Logger::endl;
		}
	 } 
	 else 
	 {
		// there is no such resource
		Logger::log(Logger::Error) << "Resource does not exist: " << resourceRequestMsg.resources[i] << Logger::endl;
	 }
	 
	 // allocate resource
  	 resourceOwners[resourceRequestMsg.resources[i]] = resourceRequestMsg.requester_name;
  }

  ResourceAssignment resourceAssignmentMsg;
  // republish a new list of resources (if any changes to the list were made
  if (isOwnersChanged)
  {
	 for (ResourceToOwnerMap::iterator it = resourceOwners.begin();
  		  it != resourceOwners.end(); ++it)
  	 {
		resourceAssignmentMsg.resources.push_back(it->first);
		resourceAssignmentMsg.owners.push_back(it->second);
  	 }
	 resourceAssignmentPort.write(resourceAssignmentMsg);
  }

  // TODO: decide how and what resources to reallocate

  // TODO: demand release of necessary resources
  
  // TODO: give resources that are not owned yet
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
  // if the component has not been deactivated, no actions needed
  if (resourceRequesterStateMsg.is_operational)
  	 return;

  // free resources of a deactivated component
  for (ResourceToOwnerMap::iterator it = resourceOwners.begin();
  		it != resourceOwners.end(); ++it)
  {
	 if (it->first == resourceRequesterStateMsg.requester_name)
	 {
		Logger::log(Logger::Info) << "Resource released (component deactivation): " 
		  << it->first << Logger::endl;

		it->second = "none";
	 }
  }

  // TODO: check whether any resource requests can be satisfied (resources were released)
}

/* Assign all resources to the component 'name' or to no one
 * when the name equals "none".
 */
void ResourceArbiter::assignAllResourcesTo(std::string name)
{
  for(ResourceToOwnerMap::iterator it = resourceOwners.begin();
  		it != resourceOwners.end(); ++it)
  {
	 it->second = name;
  }
}

void ResourceArbiter::updateHook()
{
  Logger::log(Logger::Info) << "ResourceArbiter executes updateHook !" <<Logger::endl;

  // process ports
  
  ResourceRequest resourceRequestMsg;
  switch (resourceRequestPort.read(resourceRequestMsg))
  {
	 case RTT::NewData:
		Logger::log(Logger::Info) << "New resource request" << Logger::endl;
		processResourceRequest(resourceRequestMsg);
		break;
  }

  ResourceRequesterState resourceRequesterStateMsg;
  switch (resourceRequesterStatePort.read(resourceRequesterStateMsg))
  {
	 case RTT::NewData:
		Logger::log(Logger::Info) << "Component is reporting state" << Logger::endl;
		processResourceRequesterState(resourceRequesterStateMsg);
		break;
  }
}

void ResourceArbiter::stopHook() 
{
  Logger::log(Logger::Info) << "ResourceArbiter executes stopping !" <<Logger::endl;
}

void ResourceArbiter::cleanupHook() 
{
  Logger::log(Logger::Info) << "ResourceArbiter cleaning up !" <<Logger::endl;
}

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
ORO_CREATE_COMPONENT(ResourceArbiter)
