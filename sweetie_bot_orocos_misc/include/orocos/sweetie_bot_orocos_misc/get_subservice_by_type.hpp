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
	// get subservices names 
	RTT::Service::ProviderNames subservice_names = service->getProviderNames();
	// find subservice of given type
	for(const std::string& name : subservice_names) {
        ServiceInterface * found_service = dynamic_cast<ServiceInterface*>(service->getService(name).get());
		if (found_service) return found_service;
	}
	return nullptr;
}

/**
 * @brief Get all subservices of given type.
 * Find all subservice which can be casted to the given type.
 * @param service Pointer to parent service. 
 * @return @c std::vector of pointers to found subservices.
 **/
template<class ServiceInterface> std::vector<ServiceInterface *> getAllSubServicesByType(boost::shared_ptr<RTT::Service> service) 
{
	if (!service) return std::vector<ServiceInterface *>();
	// get subservices names 
	RTT::Service::ProviderNames subservice_names = service->getProviderNames();
	// find subservice of given type
	std::vector<ServiceInterface *> service_list;
	for(const std::string& name : subservice_names) {
        ServiceInterface * found_service = dynamic_cast<ServiceInterface*>(service->getService(name).get());
		if (found_service) service_list.push_back(found_service);
	}
	return service_list;
}

/**
 * @brief Find subservice by type. @c boost::shared_ptr version.
 * Find subservice which can be casted to the given type, return pointer to it or NULL otherwise.
 * @param service Pointer to parent service. 
 * @return @c boost::shared_ptr pointer to found subservice or NULL if subservice not found or @a service is NULL.
 **/
template<class ServiceInterface> boost::shared_ptr<ServiceInterface>  getSubServiceByType(boost::shared_ptr<RTT::Service> service) 
{
	if (!service) return nullptr;
	// get subservices names 
	RTT::Service::ProviderNames subservice_names = service->getProviderNames();
	// find subservice of given type
	boost::shared_ptr<ServiceInterface> found_service;
	for(const std::string& name : subservice_names) {
		found_service = boost::dynamic_pointer_cast<ServiceInterface>(service->getService(name));
		if (found_service) return found_service;
	}
	return nullptr;
}

/**
 * @brief Get all subservices of given type. @c boost::shared_ptr version.
 * Find all subservice which can be casted to the given type.
 * @param service Pointer to parent service. 
 * @return @c std::vector of @c boost::shared_ptr pointers to found subservices.
 **/
/*template<class ServiceInterface> std::vector< boost::shared_ptr<ServiceInterface> > getAllSubServicesByType(boost::shared_ptr<RTT::Service> service) 
{
	if (!service) return std::vector< boost::shared_ptr<ServiceInterface> >();
	// get subservices names 
	RTT::Service::ProviderNames subservice_names = service->getProviderNames();
	// find subservice of given type
	boost::shared_ptr<ServiceInterface> found_service;
	std::vector< boost::shared_ptr<ServiceInterface> > service_list;
	for(const std::string& name : subservice_names) {
		found_service = boost::dynamic_pointer_cast<ServiceInterface>(service->getService(name));
		if (found_service) service_list.push_back(found_service);
	}
	return service_list;
}*/


} // namespace sweetie_bot

#endif  /*GET_SUBSERVICE_BY_TYPE_HPP*/
