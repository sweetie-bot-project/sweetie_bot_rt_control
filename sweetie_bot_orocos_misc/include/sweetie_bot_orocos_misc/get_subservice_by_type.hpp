#ifndef  GET_SUBSERVICE_BY_TYPE_HPP
#define  GET_SUBSERVICE_BY_TYPE_HPP

#include <rtt/Service.hpp>

namespace sweetie_bot {

/**
 * @brief Find subservice by type.
 * Find subservice which can be casted to the given type, return pointer to it or NULL otherwise.
 * @param service Pointer to service. 
 * @return Pointer to found subservice or NULL if subservice not found or @a service is NULL.
 **/
template<class ServiceInterface> ServiceInterface * getSubServiceByType(RTT::Service * service) 
{
	if (!service) return nullptr;

	ServiceInterface * found_service;
	RTT::Service::ProviderNames subservices;

	subservices = service->getProviderNames();
	for(RTT::Service::ProviderNames::const_iterator name = subservices.begin(); name != subservices.end(); name++) {
        found_service = dynamic_cast<ServiceInterface*>(service->getService(*name).get());
		if (found_service) return found_service;
	}
	return nullptr;
}

/**
 * @brief Find subservice by type. @c shared_ptr version.
 * Find subservice which can be casted to the given type, return pointer to it or NULL otherwise.
 * @param service Pointer to service. 
 * @return Pointer to found subservice or NULL if subservice not found or @a service is NULL.
 **/
template<class ServiceInterface> boost::shared_ptr<ServiceInterface>  getSubServiceByType(boost::shared_ptr<RTT::Service> service) 
{
	if (!service) return nullptr;

	boost::shared_ptr<ServiceInterface> found_service;
	RTT::Service::ProviderNames subservices;

	subservices = service->getProviderNames();
	for(RTT::Service::ProviderNames::const_iterator name = subservices.begin(); name != subservices.end(); name++) {
		found_service = boost::dynamic_pointer_cast<ServiceInterface>(service->getService(*name));
		if (found_service) return found_service;
	}
	return nullptr;
}

} // namespace sweetie_bot

#endif  /*GET_SUBSERVICE_BY_TYPE_HPP*/
