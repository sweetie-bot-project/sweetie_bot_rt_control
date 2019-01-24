#ifndef OROCOS_SWEETIE_BOT_CONTROLLER_FOLLOW_SATNCE_HPP
#define OROCOS_SWEETIE_BOT_CONTROLLER_FOLLOW_SATNCE_HPP

#include <vector>
#include <string>

#include <rtt/Component.hpp>
#include <rtt/os/Timer.hpp>

#include <sweetie_bot_kinematics_msgs/typekit/RigidBodyState.h>
#include <sweetie_bot_kinematics_msgs/typekit/SupportState.h>
#include <sweetie_bot_kinematics_msgs/typekit/BalanceState.h>
#include <sweetie_bot_control_msgs/typekit/FollowStepSequenceAction.h>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_resource_control/resource_client.hpp>
#include <sweetie_bot_robot_model/robot_model.hpp>
#include <sweetie_bot_orocos_misc/simple_action_server.hpp>

#include "cartesian_trajectory_cache.hpp"

namespace sweetie_bot {
namespace motion {
namespace controller {

class ExecuteStepSequence : public RTT::TaskContext
{
	protected:
		// Goal, Feedback, Result typedefs
		ACTION_DEFINITION(sweetie_bot_control_msgs::FollowStepSequenceAction);	
	
	protected:
		// COMPONENT INTERFACE
		//
		// PORTS: input
		RTT::InputPort<RTT::os::Timer::TimerId> sync_port;
		RTT::InputPort<sweetie_bot_kinematics_msgs::RigidBodyState> in_limbs_port;
		RTT::InputPort<sweetie_bot_kinematics_msgs::RigidBodyState> in_base_port;
		//RTT::InputPort<sweetie_bot_kinematics_msgs::BalanceState> in_balance_port;
		// PORTS: output
		RTT::OutputPort<sweetie_bot_kinematics_msgs::RigidBodyState> out_limbs_ref_port;
		//RTT::OutputPort<sweetie_bot_kinematics_msgs::RigidBodyState> out_base_ref_port;
		RTT::OutputPort<sweetie_bot_kinematics_msgs::SupportState> out_supports_port;
		// PROPERTIES
		double period;

	protected:
		// OPERATIONS: provides
		// OPERATIONS: requires
		RTT::OperationCaller<bool(const sweetie_bot_kinematics_msgs::RigidBodyState&)> poseToJointStatePublish;
		// SERVICES: provides
		// SERVICES: required
		sweetie_bot::motion::RobotModel * robot_model;
		// SERVICES: internal interface
		sweetie_bot::motion::ResourceClientInterface * resource_client; //< resource client
		bool resourceChangedHook();
		void stopOperationalHook();
		// ACTIONLIB:
		// Simple action server
		OrocosSimpleActionServer<sweetie_bot_control_msgs::FollowStepSequenceAction> action_server;
		// new pending goal is received
		void newGoalHook(const Goal& goal);
		// active goal is being canceled
		void cancelGoalHook();
		// buffers (also can be created dynamically)
		Result goal_result;
		std::shared_ptr<Goal> goal;
		std::shared_ptr<Goal> goal_pending;


	protected:
		// COMPONENT STATE
		std::shared_ptr<CartesianTrajectoryCache> trajectory;
		// ports buffers
		sweetie_bot_kinematics_msgs::RigidBodyState base; // current pose received from in_base
		//sweetie_bot_kinematics_msgs::RigidBodyState base_next; // next pose calucalated by component
		//sweetie_bot_kinematics_msgs::RigidBodyState base_ref; // reference pose received from in_base_ref
		//sweetie_bot_kinematics_msgs::BalanceState balance; // balance 
		sweetie_bot_kinematics_msgs::RigidBodyState limbs; // calculated reference pose to publish on out_port
		sweetie_bot_kinematics_msgs::RigidBodyState limbs_full; // received limbs state on in_limbs_fixed
		sweetie_bot_kinematics_msgs::SupportState supports; // support state to publish

#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif

		
	public:
		ExecuteStepSequence(std::string const& name);

	protected:
		bool dataOnPortHook( RTT::base::PortInterface* portInterface );
		
		// convinece functions
		bool readPorts();
		void rejectPending(const std::string& where, const std::string& msg, int code);
		void abortActive(const std::string& where, const std::string& msg, int code);


		bool configureHook(); 
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

#endif /*OROCOS_SWEETIE_BOT_CONTROLLER_FOLLOW_SATNCE_HPP*/
