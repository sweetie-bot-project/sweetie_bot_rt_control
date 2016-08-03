#include "sweetie_bot_gait_walking_3_plus_1-component.hpp"
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include <iostream>

using RTT::Logger;

GaitWalking3Plus1::GaitWalking3Plus1(std::string const& name) : TaskContext(name)
{
  Logger::log(Logger::Info) << "GaitWalking3Plus1 constructed !" <<Logger::endl;

  this->ports()->addPort("resourceRequestPort", resourceRequestPort)
  	 .doc("The components sends requests to allocate resources to it"
  	 	  " through this port.");

  this->ports()->addPort("resourceReleasedPort", resourceReleasedPort)
  	 .doc("The component notifies that is has released a resource"
  	 	  " through this port.");

  this->ports()->addEventPort("resourceReplyPort", resourceReplyPort)
  	 .doc("The component listens to this port to get notified what"
  	 	  " resources it has been given.");

  this->ports()->addEventPort("resourceDemandReleasePort", resourceDemandReleasePort)
  	 .doc("The component listens to this port to know if a release"
  	 	  " of one of its resources has been demanded.");

  initResources();
}

void GaitWalking3Plus1::initResources()
{
  // TODO: move to the parameter server
  std::string tmp[] = {"leg1", "leg2", "leg3", "leg4"};
  std::vector<std::string> resources(tmp, tmp+sizeof(tmp) / sizeof(tmp[0]));
  for (std::vector<std::string>::iterator name = resources.begin(); name != resources.end(); name++)
  {
	 this->resourcesRequiredPrimary[*name] = 1.0;
	 Logger::log(Logger::Info) << "Primary resource initialized: " << *name << " with priority "
	 	<< this->resourcesRequiredPrimary[*name] << Logger::endl;
  }

  resources.clear();
  resources.push_back(std::string("head"));
  for (std::vector<std::string>::iterator name = resources.begin(); name != resources.end(); name++)
  {
	 this->resourcesRequiredAuxiliary[*name] = 1.0;
	 Logger::log(Logger::Info) << "Primary resource initialized: " << *name << " with priority "
	 	<< this->resourcesRequiredAuxiliary[*name] << Logger::endl;
  }
}

bool GaitWalking3Plus1::configureHook()
{
  Logger::log(Logger::Info) << "GaitWalking3Plus1 configured !" <<Logger::endl;

  return true;
}

bool GaitWalking3Plus1::startHook()
{
  Logger::log(Logger::Info) << "GaitWalking3Plus1 started !" <<Logger::endl;
  return true;
}

void GaitWalking3Plus1::appendResourceRequest(std::string name, double priority,
	 ResourceRequest& resourceRequestMsg
	 )
{
  resourceRequestMsg.resources.push_back(name);

  resourceRequestMsg.priorities.push_back(priority);
}

void GaitWalking3Plus1::processResourceReply(ResourceReply& resourceReplyMsg)
{
  if (GAIT_WALKING_ID != resourceReplyMsg.user_id)
  {
	 Logger::log(Logger::Info) << "Resource reply not for this component" << Logger::endl;
	 return;
  }

  for (std::vector<std::string>::iterator it = resourceReplyMsg.resources.begin();
		it != resourceReplyMsg.resources.end();
		++it)
  {
  	 resourcesControlled[*it] = true;
	 Logger::log(Logger::Info) << "Gained control of resource: " << *it << Logger::endl;
  }
}

void GaitWalking3Plus1::processResourceDemandRelease(
  ResourceDemandRelease& resourceDemandReleaseMsg
  )
{
  ResourceReleased resourceReleasedMsg;

  for (std::vector<std::string>::iterator it = resourceDemandReleaseMsg.resources.begin();
  	 it != resourceDemandReleaseMsg.resources.end();
  	 ++it)
  {
	 if (resourcesControlled.find(*it) != resourcesControlled.end())
	 {
	 	resourcesControlled.erase(*it);
	 	Logger::log(Logger::Info) << "Releasing resource: " << *it<< Logger::endl;

	 	resourceReleasedMsg.resources.push_back(*it);
	 } else
	 {
		Logger::log(Logger::Info) << "Can't release resource not owned: " << *it<< Logger::endl;
	 }
  }

  resourceReleasedPort.write(resourceReleasedMsg);
}

void GaitWalking3Plus1::updateHook()
{
  Logger::log(Logger::Info) << "GaitWalking3Plus1 executes updateHook !" <<Logger::endl;

  // process ports
  ResourceReply resourceReplyMsg;
  switch (resourceReplyPort.read(resourceReplyMsg))
  {
	 case RTT::NewData:
		Logger::log(Logger::Info) << "Resource reply received" << Logger::endl;
		processResourceReply(resourceReplyMsg);
		break;
  }

  ResourceDemandRelease resourceDemandReleaseMsg;
  switch (resourceDemandReleasePort.read(resourceDemandReleaseMsg))
  {
	 case RTT::NewData:
		Logger::log(Logger::Info) << "Resource demand release requested" << Logger::endl;
		processResourceDemandRelease(resourceDemandReleaseMsg);
		break;
  }

  // main logic
  
  ResourceRequest resourceRequestMsg;
  resourceRequestMsg.user_id = GAIT_WALKING_ID; // set the id of this behaviour
  
  bool primaryResourcesAcquired = true;

  // if not all primary resources are in this one's posession, request them
  for (std::map<std::string, double>::iterator resource = resourcesRequiredPrimary.begin();
  		resource != resourcesRequiredPrimary.end(); ++resource)
  {
	 if (resourcesControlled.find(resource->first) == resourcesControlled.end())
	 {
	 	// resource not controlled
	 	primaryResourcesAcquired = false;
		appendResourceRequest(resource->first, resource->second, resourceRequestMsg);
	 }
  }
  
  // if not all auxiliary resources are in this one's posession, try to get
  // them, too
  bool auxiliaryResourcesAcquired = true;

  for (std::map<std::string, double>::iterator resource = resourcesRequiredAuxiliary.begin();
  		resource != resourcesRequiredAuxiliary.end(); ++resource)
  {
	 if (resourcesControlled.find(resource->first) == resourcesControlled.end())
	 {
	 	// resource not controlled
	 	auxiliaryResourcesAcquired = false;
		appendResourceRequest(resource->first, resource->second, resourceRequestMsg);
	 }
  } 

  // if not all resources acquired, then request them
  if (!(primaryResourcesAcquired && auxiliaryResourcesAcquired))
	 resourceRequestPort.write(resourceRequestMsg);

  // if all necessary resources have been allocated to this component, it
  // can execute its main cycle
  if (primaryResourcesAcquired)
  {
	 calculateMovement();
  }

  // TODO: if a release of a resource has been demanded, perform necessary cleanup
  // and safety actions before releasing it
}

void GaitWalking3Plus1::calculateMovement()
{
  // TODO: calculate and send speeds of the end of legs
  Logger::log(Logger::Info) << "Calculating movement" << Logger::endl;

}

void GaitWalking3Plus1::stopHook() 
{
  Logger::log(Logger::Info) << "GaitWalking3Plus1 executes stopping !" <<Logger::endl;
}

void GaitWalking3Plus1::cleanupHook() 
{
  Logger::log(Logger::Info) << "GaitWalking3Plus1 cleaning up !" <<Logger::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(GaitWalking3Plus1)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(GaitWalking3Plus1)
