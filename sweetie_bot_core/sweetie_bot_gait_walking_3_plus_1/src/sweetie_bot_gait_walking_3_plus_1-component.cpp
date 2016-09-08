
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include <sweetie_bot_gait_walking_3_plus_1/sweetie_bot_gait_walking_3_plus_1-component.hpp>
#include <sweetie_bot_resource_control_service/sweetie_bot_resource_control_service.hpp>

#include <iostream>

using RTT::Logger;

GaitWalking3Plus1::GaitWalking3Plus1(std::string const& name) : TaskContext(name)
{
  initResources();

  resource_client = getProvider<ResourceClient>("resource_client");

  Logger::log(Logger::Info) << "GaitWalking3Plus1 constructed !" << Logger::endl;
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
	 Logger::log(Logger::Info) << "Auxiliary resource initialized: " << *name << " with priority "
	 	<< this->resourcesRequiredAuxiliary[*name] << Logger::endl;
  }
}

bool GaitWalking3Plus1::configureHook()
{
  Logger::log(Logger::Info) << "GaitWalking3Plus1 configured !" << Logger::endl;

  return true;
}

bool GaitWalking3Plus1::startHook()
{
  Logger::log(Logger::Info) << "GaitWalking3Plus1 started !" << Logger::endl;

  resource_client->requestResources(resourcesRequiredPrimary);

  return true;
}

void GaitWalking3Plus1::updateHook()
{
  Logger::log(Logger::Info) << "GaitWalking3Plus1 executes updateHook !" << Logger::endl;

  // process ports
  /*
  ResourceReply resourceReplyMsg;
  switch (resourceReplyPort.read(resourceReplyMsg))
  {
	 case RTT::NewData:
		Logger::log(Logger::Info) << "Resource reply received" << Logger::endl;
		processResourceReply(resourceReplyMsg);
		break;
  }
*/

  // main logic
  
  if (resource_client->isOperational())
  {
	 calculateMovement();
  } 
  else
  {
  	 resource_client->requestResources(resourcesRequiredPrimary);
  }

}

void GaitWalking3Plus1::calculateMovement()
{
  // TODO: calculate and send speeds of the end of legs
  Logger::log(Logger::Info) << "Calculating movement" << Logger::endl;

}

void GaitWalking3Plus1::stopHook() 
{
  Logger::log(Logger::Info) << "GaitWalking3Plus1 executes stopping !" <<Logger::endl;

  resource_client->stopOperational();
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
