#include "sweetie_bot_resource_client_usage_example-component.hpp"

#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include <sweetie_bot_resource_control/sweetie_bot_resource_control_service.hpp>

#include <iostream>

using RTT::Logger;

ResourceClientExample::ResourceClientExample(std::string const& name) : TaskContext(name),
  isActive(false)
{
  initResources();

  resource_client = getProvider<ResourceClient>("resource_client");

  Logger::log(Logger::Info) << "ResourceClientExample constructed !" << Logger::endl;
}

void ResourceClientExample::initResources()
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
	 Logger::log(Logger::Info) << "Auxiliary resource initialized: " << *name << " with priority "
	 	<< this->resourcesRequiredAuxiliary[*name] << Logger::endl;
  }
}

/* Tries to start the controller.
 *
 * Sends a resource request, a reply to which will be processed by the plugin's
 * callback and the controller will be activated and start doing its task
 * in the updateHook if all resources were allocated or not, if some are lacking.
 * This is checked using the controller's resourceChangedHook.
 */
bool ResourceClientExample::activate()
{
  resource_client->requestResources(resourcesRequiredPrimary);
}

/*	Determines if the controller can function normally in the given situation or not. 
 *	Usually it just checks if all the required resources have been allocated to it.
 *	The isOperational flag will be set by the plugin.
 */
bool ResourceClientExample::resourceChangedHook()
{
  // TODO
  for (std::map<std::string, double>::iterator it = resourcesRequiredPrimary.begin();
  		it != resourcesRequiredPrimary.end(); ++it)
  {
	 // if a resource is not owned then activation is not possible
	 if (!resource_client->hasResource(it->first))
	 {
		return false;
	 }
  }

  return true;
}

bool ResourceClientExample::configureHook()
{
  Logger::log(Logger::Info) << "ResourceClientExample configured !" << Logger::endl;

  return true;
}

bool ResourceClientExample::startHook()
{
  Logger::log(Logger::Info) << "ResourceClientExample started !" << Logger::endl;

  return true;
}

void ResourceClientExample::updateHook()
{
  Logger::log(Logger::Info) << "ResourceClientExample executes updateHook !" << Logger::endl;

  // main logic
  
  if (resource_client->isOperational())
  {
	 doUsefulStuff();
  } 
}

void ResourceClientExample::doUsefulStuff()
{
  // TODO: calculate and send speeds of the end of legs
  Logger::log(Logger::Info) << "Calculating movement" << Logger::endl;

}

/* Preempts the controllers and releases its resources.
 *
 */
void ResourceClientExample::stopHook() 
{
  Logger::log(Logger::Info) << "ResourceClientExample executes stopping !" <<Logger::endl;

  resource_client->stopOperational();
}

void ResourceClientExample::cleanupHook() 
{
  Logger::log(Logger::Info) << "ResourceClientExample cleaning up !" <<Logger::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ResourceClientExample)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(ResourceClientExample)
