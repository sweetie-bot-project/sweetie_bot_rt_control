#ifndef OROCOS_RESOURCE_CONTROL_SERVICE_HPP
#define OROCOS_RESOURCE_CONTROL_SERVICE_HPP

#include <rtt/RTT.hpp>
#include <string>

using RTT::ServiceRequester;
using RTT::OperationCaller;
using RTT::TaskContext;

namespace sweetie_bot {

namespace motion {

class ResourceClientInterface
{
  public:
  	 virtual bool requestResources(const std::vector<std::string>& resource_list) = 0;
  	 virtual bool stopOperational() = 0;
  	 virtual bool isOperational() = 0;
  	 virtual bool hasResource(const std::string& resource) = 0;
  	 virtual bool hasResources(const std::vector<std::string>& resource_list) = 0;
  	 virtual void step() = 0;
	 virtual void setResourceChangeHook(bool (*resourceChangedHook_)()) = 0;
};


class ResourceClient: public RTT::ServiceRequester
{
  public:
  	 OperationCaller< bool(const std::vector<std::string>&) > requestResources;
  	 OperationCaller< bool() > stopOperational;
  	 OperationCaller< bool() > isOperational;
  	 OperationCaller< bool(const std::string&) > hasResource;
  	 OperationCaller< bool(const std::vector<std::string>&) > hasResources;
  	 OperationCaller< void() > step;

  	 ResourceClient(TaskContext *owner) :
  	 	ServiceRequester("resource_client_requester", owner),
  	 	requestResources("requestResources"),
  	 	stopOperational("stopOperational"),
  	 	isOperational("isOperational"),
  	 	hasResource("hasResource"),
  	 	hasResources("hasResources"),
		step("step")
  	 {
		addOperationCaller(requestResources);
		addOperationCaller(stopOperational);
		addOperationCaller(isOperational);
		addOperationCaller(hasResource);
		addOperationCaller(hasResources);
		addOperationCaller(step);
  	 }
};

} // namespace motion 

} // namespace sweetie_bot


#endif
