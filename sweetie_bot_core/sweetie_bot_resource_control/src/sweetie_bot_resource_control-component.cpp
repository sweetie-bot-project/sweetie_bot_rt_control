#include "sweetie_bot_resource_control-component.hpp"
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include <vector>
#include <iostream>

using RTT::Logger;

ResourceControl::ResourceControl(std::string const& name) : TaskContext(name)
{
  Logger::log(Logger::Info) << "ResourceControl constructed !" <<Logger::endl;

  this->ports()->addPort("resourceReplyPort", resourceReplyPort)
	 .doc("Resource manager will reply to requesters using this port to notify them, whether"
		  " the requested resource was allocated to them, or they were refused.");

  this->ports()->addPort("resourceDemandReleasePort", resourceDemandReleasePort)
	 .doc("Resource manager demands components to release resources they were previously"
		 " given using this port.");

  this->ports()->addEventPort("resourceRequestPort", resourceRequestPort)
	 .doc("Resource manager receives requests for resource allocation via this port.");

  this->ports()->addEventPort("resourceReleasedPort", resourceReleasedPort)
	 .doc("Resource manager is notified that a resource was released by a component"
		 " via this port.");

  initResources();
}

void ResourceControl::initResources()
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
	 this->resourceOwners[*name] = 0; // add a free resource
	 Logger::log(Logger::Info) << "Resource initialized: " << *name << Logger::endl;
  }
}

bool ResourceControl::configureHook()
{
  Logger::log(Logger::Info) << "ResourceControl configured !" <<Logger::endl;

  return true;
}

bool ResourceControl::startHook()
{
  Logger::log(Logger::Info) << "ResourceControl started !" <<Logger::endl;
  return true;
}

void ResourceControl::processResourceRequest(ResourceRequest& resourceRequestMsg)
{
  // TODO: redo with new messages
  
  // NOTE: temporary solution
  // give resources that were demanded unless they are unavailable
  /*
  ResourceReply resourceReplyMsg;
  resourceReplyMsg.user_id = resourceRequestMsg.user_id; // reply to the requester

  for (std::vector<std::string>::iterator requestedIt = resourceRequestMsg.resources.begin();
  		requestedIt != resourceRequestMsg.resources.end();
  		++requestedIt)
  {
	 std::map<std::string, int>::iterator ownedIt = resourceOwners.find(*requestedIt);
	 if (ownedIt != resourceOwners.end())
	 {
		// there is such resource
		if (resourceOwners[*requestedIt] == resourceRequestMsg.user_id)
		{
		  // the requester already owns this resource (should be a warning or an error
		  // since this is a strange case)
		  resourceReplyMsg.resources.push_back(*requestedIt);
		  resourceOwners[*requestedIt] = resourceReplyMsg.user_id;

		  Logger::log(Logger::Info) << "Resource [" << *requestedIt<< "] already owned by [" 
		  	 << resourceRequestMsg.user_id << "]" << Logger::endl;
		} 
		else if (resourceOwners[*requestedIt] == 0)
		{
		  // the resource is free and will be given to the requester
		  resourceReplyMsg.resources.push_back(*requestedIt);
		  resourceOwners[*requestedIt] = resourceReplyMsg.user_id;

		  Logger::log(Logger::Info) << "Resource [" << *requestedIt<< "] given to ["
		  	 << resourceRequestMsg.user_id << "]" << Logger::endl;
		} 
		else 
		{
		  // the resource is not available
		  Logger::log(Logger::Info) << "Resource [" << *requestedIt<< "] is not available and owned by ["
		  	 << resourceOwners[*requestedIt] << "]" << Logger::endl;
		}
	 } 
	 else 
	 {
		// there is no such resource
		Logger::log(Logger::Info) << "Resource does not exist: " << *requestedIt<< Logger::endl;
	 }
  }

  resourceReplyPort.write(resourceReplyMsg);
	 */

  // TODO: decide how and what resources to reallocate

  // TODO: demand release of necessary resources
  
  // TODO: give resources that are not owned yet
}

/* Process resource released message.
 *
 * @param resourceReleasedMsg ResourceReleased The message with a list of released resources.
 */
void ResourceControl::processResourceReleased(ResourceReleased& resourceReleasedMsg)
{
  // NOTE: the message does not contain the id of the releaser, so that information can not
  // be checked against the arbitrator's existing table of resources allocated

  for (std::vector<std::string>::iterator releasedIt = resourceReleasedMsg.resources.begin();
  		releasedIt != resourceReleasedMsg.resources.end();
  		++releasedIt)
  {
  	 // check if such resource really exists
	 std::map<std::string, int>::iterator resourceIt = resourceOwners.find(*releasedIt);
	 if (resourceIt == resourceOwners.end())
	 {
		// no such resource
		Logger::log(Logger::Info) << "Resource released does not exist" << Logger::endl;
	 }
	 else 
	 {
		resourceIt->second = 0; // mark resource as released
	 }
  }

  // TODO: check whether any resource requests can be satisfied (resources were released)
}

void ResourceControl::updateHook()
{
  Logger::log(Logger::Info) << "ResourceControl executes updateHook !" <<Logger::endl;

  // process ports
  
  ResourceRequest resourceRequestMsg;
  switch (resourceRequestPort.read(resourceRequestMsg))
  {
	 case RTT::NewData:
		Logger::log(Logger::Info) << "New resource request" << Logger::endl;
		processResourceRequest(resourceRequestMsg);
		break;
  }

  ResourceReleased resourceReleasedMsg;
  switch (resourceReleasedPort.read(resourceReleasedMsg))
  {
	 case RTT::NewData:
		Logger::log(Logger::Info) << "Resource was released" << Logger::endl;
		processResourceReleased(resourceReleasedMsg);
		break;
  }
}

void ResourceControl::stopHook() 
{
  Logger::log(Logger::Info) << "ResourceControl executes stopping !" <<Logger::endl;
}

void ResourceControl::cleanupHook() 
{
  Logger::log(Logger::Info) << "ResourceControl cleaning up !" <<Logger::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ResourceControl)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(ResourceControl)
