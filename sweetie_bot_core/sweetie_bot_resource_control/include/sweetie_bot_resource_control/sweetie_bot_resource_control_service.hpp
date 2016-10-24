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
  	 virtual bool getIsOperational() = 0;
  	 virtual bool hasResource(std::string res) = 0;
};

class ResourceClientInterface2
{
  public:
  	 virtual void requestResources(const std::vector<std::string>& resource_list) = 0;
  	 virtual void stopOperational() = 0;
  	 virtual bool isOperational() = 0;
  	 virtual bool hasResource(const std::string& resource) = 0;
  	 virtual bool hasResources(const std::vector<std::string>& resource_list) = 0;
};


class ResourceClient: public RTT::ServiceRequester
{
  public:
  	 OperationCaller< void(const std::vector<std::string>&) > requestResources;
  	 OperationCaller< void() > stopOperational;
  	 OperationCaller< bool() > isOperational;
  	 OperationCaller< bool(const std::string&) > hasResource;
  	 OperationCaller< bool(const std::vector<std::string>&) > hasResources;

  	 ResourceClient(TaskContext *owner) :
  	 	ServiceRequester("resource_client_requester", owner),
  	 	requestResources("requestResources"),
  	 	stopOperational("stopOperational"),
  	 	isOperational("isOperational"),
  	 	hasResource("hasResource"),
  	 	hasResources("hasResources")
  	 {
		addOperationCaller(requestResources);
		addOperationCaller(stopOperational);
		addOperationCaller(isOperational);
		addOperationCaller(hasResource);
        addOperationCaller(hasResources);
  	 }
};

#endif
