#ifndef  ACTIONLIB_CONTROLLER_BASE_HPP
#define  ACTIONLIB_CONTROLLER_BASE_HPP 

#include <vector>
#include <string>

#include <rtt/Component.hpp>
#include <rtt/os/Timer.hpp>

#include <std_srvs/SetBool.h>
#include <sweetie_bot_resource_control_msgs/typekit/SetOperationalResult.h>
#include <sweetie_bot_resource_control_msgs/typekit/SetOperationalAction.h>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_resource_control/resource_client.hpp>
#include <sweetie_bot_orocos_misc/simple_action_server.hpp>

namespace sweetie_bot {
namespace motion {
namespace controller {


/**
 * @brief Superclass for controller components which use actionlib SetOperational interface for activation.
 *
 * This class provides common functionality (actionlib and resource arbiter interations) for
 * controllers which are able to control arbitrary or specific set of resources. 
 * Such controllers can are activated or deactivated by SetOperationalGoal message which contains 
 * `resource` field with desired resource set (this value can be ignored by controller).
 *
 * This class implements ActionServer interface and ResourceClientInterface, so a component subclass should
 * only implements following methods.
 * @code
		bool configureHook_impl();
		bool processResourceSet_impl(const std::vector<std::string>& resource_set, std::vector<std::string>& desired_resource_set);
		bool startHook_impl();
		bool resourceChangedHook_impl(const std::vector<std::string>& desired_resource_set, const std::vector<std::string>& acquired_resource_set);
		void updateHook_impl();
		void stopHook_impl();
		void cleanupHook_impl();
 * @endcode
 * Note that minimal implementation are already provided.
 *
 * Also class provides Logger @c log object and @a controlled_chains and @a period properties.
 **/
class ActionlibControllerBase : public RTT::TaskContext
{
	protected:
		// Goal, Feedback, Result typedefs
		ACTION_DEFINITION(sweetie_bot_resource_control_msgs::SetOperationalAction);	
	protected:
		// COMPONENT INTERFACE
		//
		// PORTS: input
		RTT::InputPort<RTT::os::Timer::TimerId> sync_port; /**< Syncronization port. Recives message on start each control cycle. */
		// PORTS: output
		// PROPERTIES
		std::vector<std::string> default_resource_set; /**< Property to hold default list of resources. This list is used if component activated via @c stop() or @c rosSetOperational() call. */
		double period; /**< Property to hold the control cycle duration. */
	protected:
		// OPERATIONS: provides
		bool rosSetOperational(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp); /**< ROS-compatible activation operation (ROS service). */
		// OPERATIONS: requires
		// SERVICES: provides
		// SERVICES: required
		// SERVICES: internal interface
		sweetie_bot::motion::ResourceClientInterface * resource_client; /**< Pointer to recources_client. It acquired durin configuration from loaded plugins list. */
	private:
		// process resource change messages
		bool resourceChangeHook();

	protected:
		// ACTIONLIB:
		// Simple action server
		OrocosSimpleActionServer<sweetie_bot_resource_control_msgs::SetOperationalAction> action_server; /**< OrocosSimpleActionServer implementation. */
	private:
		// enable port callbacks in configured state
		bool dataOnPortHook(RTT::base::PortInterface* portInterface);
		// new pending goal is received
		void newGoalHook(const Goal& goal);
		// active goal is being canceled
		void cancelGoalHook();
		
		// COMPONET STATE
		std::vector<std::string> desired_resource_set; 
		// buffer
		Result goal_result;
		Feedback goal_feedback;

	protected:
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log; 
#else
		sweetie_bot::logger::LoggerRTT log;
#endif
	protected:
		// subclass interface

		/**
		 * @brief @c configureHook() inplementation. 
		 * Called after resource_client and ActionServer are configured. Should contain component initialization.
		 **/	
		virtual bool configureHook_impl();
		/**
		 * @brief Implement this function to restrict acceptible resources sets or enforce specific resource set.
		 *
		 * Always called after configureHook_impl() and before actual resource request. This function is used  
		 * to check if action goal is sane and to determine resources which should be requested.
		 *
		 * Minimal implementation sets @a desired_resource_set equal to @a goal_resource_set and return true.
		 * In more complex cases @a goal_resource_set should be checked if it comforms controller-specific 
		 * conditions and the actual resource set to request should be assigned to @a desired_resource_set.
		 *
		 * @param goal_resource_set A resource from SetOperationalGoal messages. start() operation uses default value from `kinematic_chains` property.
		 * @param desired_resource_set Resource set which should be requested. Usually 
		 * @return true if goal_resource_set set is acceptible for component.
		 **/
		virtual bool processResourceSet_impl(const std::vector<std::string>& goal_resource_set, std::vector<std::string>& desired_resource_set);
		/**
		 * @brief @c startHook() implementation.
		 * Requested and acquired resource sets are not known yet.
		 * @return true to start component.
		 **/
		virtual bool startHook_impl();
		/**
		 * @brief Implement reaction on resource set change.
		 * Always called after startHook_impl() and processResourceSet_impl(). Acquired resource set can be accessed via @c resource_client field.
		 * This function must reinitialize controller to run with different resource set or return failure.
		 * @param desired_resource_set Resource set requested from arbiter. It is the same set which was returned in second parameter of @a processResourceSet_impl().
		 * @return true to activate controller.
		 **/
		virtual bool resourceChangedHook_impl(const std::vector<std::string>& desired_resource_set);
		/**
		 * @brief @c updateHook() implementation. 
		 * It is periodically called if component active.
		 **/
		virtual void updateHook_impl();
		virtual void stopHook_impl(); /**< @c stopHook() implementation. */
		virtual void cleanupHook_impl(); /**< cleanupHook() implementation. */

	public:
		ActionlibControllerBase(std::string const& name);

		//TODO access specificator? These methoads are final and private.
		bool configureHook(); 
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

#endif  /*ACTIONLIB_CONTROLLER_BASE_HPP*/
