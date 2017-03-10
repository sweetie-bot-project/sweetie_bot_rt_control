#ifndef OROCOS_RESOURCE_CONTROL_SERVICE_HPP
#define OROCOS_RESOURCE_CONTROL_SERVICE_HPP

#include <string>
#include <vector>
#include <boost/function.hpp>

#include <rtt/OperationCaller.hpp>
#include <rtt/ServiceRequester.hpp>
#include <rtt/ServiceRequester.hpp>

namespace sweetie_bot {

namespace motion {

class ResourceClientInterface
{
  public:
  	 virtual bool resourceChangeRequest(const std::vector<std::string>& resource_list) = 0;
  	 virtual bool stopOperational() = 0;
  	 virtual void step() = 0;

  	 virtual bool isOperational() const = 0;
  	 virtual bool isPending() const = 0;
  	 virtual int getState() const = 0;
  	 virtual bool hasResource(const std::string& resource) const = 0;
  	 virtual bool hasResources(const std::vector<std::string>& resource_list) const = 0;

	 virtual void setResourceChangeHook(boost::function<bool()> resourceChangeHook_) = 0;
	 virtual void setStopOperationalHook(boost::function<void()> stopOperationalHook_) = 0;
};


class ResourceClient: public RTT::ServiceRequester
{
	public:
		enum ResourceClientStateEnum {
			NONOPERATIONAL = 0, 
			PENDING = 1, 
			OPERATIONAL = 2,
			OPERATIONAL_PENDING = 3,
		};
		typedef int ResourceClientState;

	public:
		RTT::OperationCaller< bool(const std::vector<std::string>&) > resourceChangeRequest;
		RTT::OperationCaller< void() > step;
		RTT::OperationCaller< bool() > stopOperational;

		RTT::OperationCaller< bool() > isOperational;
		RTT::OperationCaller< bool() > isPending;
		RTT::OperationCaller< int() > getState;
		RTT::OperationCaller< bool(const std::string&) > hasResource;
		RTT::OperationCaller< bool(const std::vector<std::string>&) > hasResources;

		ResourceClient(RTT::TaskContext *owner) :
			ServiceRequester("resource_client", owner),
			step("step"),
			resourceChangeRequest("resourceChangeRequest"),
			stopOperational("stopOperational"),
			isPending("isPending"),
			isOperational("isOperational"),
			getState("getState"),
			hasResource("hasResource"),
			hasResources("hasResources")
	{
		addOperationCaller(resourceChangeRequest);
		addOperationCaller(stopOperational);
		addOperationCaller(step);

		addOperationCaller(isOperational);
		addOperationCaller(getState);
		addOperationCaller(hasResource);
		addOperationCaller(hasResources);
	}
};

} // namespace motion 

} // namespace sweetie_bot


#endif
