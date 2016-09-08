#ifndef OROCOS_RESOURCE_CONTROL_SERVICE_HPP
#define OROCOS_RESOURCE_CONTROL_SERVICE_HPP

#include <rtt/RTT.hpp>

#include <string>

using RTT::ServiceRequester;
using RTT::OperationCaller;
using RTT::TaskContext;

class ResourceClientInterface
{
  public:
  	 virtual bool requestResources(std::map<std::string, double>) = 0;
  	 virtual bool stopOperational() = 0;
  	 virtual bool isOperational() = 0;
  	 virtual bool hasResource(std::string res) = 0;
};


class ResourceClient: public RTT::ServiceRequester
{
  public:
  	 OperationCaller< bool(std::map<std::string, double>) > requestResources;
  	 OperationCaller< bool() > stopOperational;
  	 OperationCaller< bool() > isOperational;
  	 OperationCaller< bool(std::string) > hasResource;

  	 ResourceClient(TaskContext *owner) :
  	 	ServiceRequester("resource_client_requester", owner),
  	 	requestResources("requestResources"),
  	 	stopOperational("stopOperational"),
  	 	isOperational("isOperational"),
  	 	hasResource("hasResource")
  	 {
		addOperationCaller(requestResources);
		addOperationCaller(stopOperational);
		addOperationCaller(isOperational);
		addOperationCaller(hasResource);
  	 }
};

#endif
