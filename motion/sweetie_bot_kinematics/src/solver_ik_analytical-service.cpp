#include "solver_ik_analytical.hpp"

#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>


namespace sweetie_bot {
namespace motion {


/**
 * @brief Analytical IK solver factory service.
 *
 **/
class SolverIKFactoryAnalyticalService : 
	public SolverIKFactoryAnalytical, public RTT::Service 
{
	protected:
		// SERVICE INTERFACE
		// properties
		std::vector<std::string> chain_names_;

	public:
		SolverIKFactoryAnalyticalService(RTT::TaskContext * owner);

		std::unique_ptr<SolverIKInterface> getSolver(const std::string& name, const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance, sweetie_bot::logger::Logger& log) const override;
};

SolverIKFactoryAnalyticalService::SolverIKFactoryAnalyticalService(RTT::TaskContext * owner) :
	Service("solver_ik_factory_analytical", owner)
{
	// Service description.
	this->doc("Provides analytical inverse kinematic solvers.");
	// Properties
	this->addProperty( "kinematic_chains", chain_names_ )
		.doc( "List of kinematic chains for which poses are calculated. Other chains are ignored.");
}

std::unique_ptr<SolverIKInterface> 
SolverIKFactoryAnalyticalService::getSolver(const std::string& name, const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance, sweetie_bot::logger::Logger& log) const
{
	// ignore kinematic chain if it is not inside list
	if (std::find(chain_names_.begin(), chain_names_.end(), name) == chain_names_.end()) {
		return nullptr;
	}
	// get solver from underlying class
	return SolverIKFactoryAnalytical::getSolver(name, chain, lower, upper, tolerance, log);
}


} // namespace sweetie_bot 
} // namespace motion 

/* For consistency reasons, it's better to name the
 * service the same as in the class above.
 */
ORO_SERVICE_NAMED_PLUGIN(sweetie_bot::motion::SolverIKFactoryAnalyticalService, "solver_ik_factory_analytical")
