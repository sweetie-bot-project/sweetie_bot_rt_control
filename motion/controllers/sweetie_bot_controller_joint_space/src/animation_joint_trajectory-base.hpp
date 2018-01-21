#ifndef  ANIMATION_JOINT_TRAJECTORY_BASE_HPP
#define  ANIMATION_JOINT_TRAJECTORY_BASE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/os/Timer.hpp>

#include <sensor_msgs/typekit/JointState.h>
#include <control_msgs/typekit/FollowJointTrajectoryGoal.h>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_resource_control/resource_client.hpp>
#include <sweetie_bot_robot_model/robot_model.hpp>
#include <sweetie_bot_controller_joint_space/filter_joint_state.hpp>

#include <sweetie_bot_kinematics_msgs/typekit/SupportState.h>

#include "joint_trajectory_cache.hpp"

namespace sweetie_bot {
namespace motion {
namespace controller {

/**
 * Incasulate allmost base logic of FollowJointTrajectoryGoal controller except action server interaction.
 **/
class AnimJointTrajectoryBase : public RTT::TaskContext
{
	public:
		typedef sensor_msgs::JointState JointState;
		typedef control_msgs::FollowJointTrajectoryGoal FollowJointTrajectoryGoal;
		typedef sweetie_bot_kinematics_msgs::SupportState SupportState;

	protected:
		// COMPONENT INTERFACE
		//
		// PORTS: input
		RTT::InputPort<RTT::os::Timer::TimerId> sync_port;
		RTT::InputPort<sensor_msgs::JointState> in_joints_port;
		// PORTS: output
		RTT::OutputPort<sensor_msgs::JointState> out_joints_port;
		RTT::OutputPort<sweetie_bot_kinematics_msgs::SupportState> out_supports_port;
		// PROPERTIES
		double period;
	protected:
		// SERVICES: required
		sweetie_bot::motion::RobotModel * robot_model; //< joints list, kinematics chains access
		int n_joints_fullpose; /**< number of joints in full robot pose. */
		// SERVICES: internal interface
		sweetie_bot::motion::ResourceClientInterface * resource_client; //< resource client
		sweetie_bot::motion::filter::FilterJointStateInterface * filter; // trajectory smoother

	protected:
		// COMPONENT STATE
		//
		double time_from_start; /**< time elasped from movement start */
		std::shared_ptr<JointTrajectoryCache> goal_active; //< current active goal cache
		std::shared_ptr<JointTrajectoryCache> goal_pending; //< goal expecting activation
		// ports buffers
		JointState actual_fullpose; //< buffer for input port in_joints_port
		JointState actual_pose; //< controlled joints actual positions
		JointState ref_pose; //< controlled joints reference positions
		SupportState support_state; //< list of contact
		
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif

	protected:
		/**
		* enable actionlib event ports in stopped state: subclasses uses this ability
		*/
        bool dataOnPortHook(RTT::base::PortInterface *portInterface);

	public:
		AnimJointTrajectoryBase(std::string const& name);

		/**
		 * @brief Implementation specific actions.
		 * Called in operation state. Ports buffer (@a actual_pose, @a actual_fullpose, @a ref_pose) and @a time_from_start variable 
		 * contains actual for current time step values. 
		 * @param on_target true if movement time elasped.
		 **/
		virtual void operationalHook(bool on_target) = 0;

		/**
		 * Check resource_client and robot_model presence. Set @a n_joints_fullpose.
		 */
		bool configureHook();
		/**
		 * Prepare component to Running state: port->getDataSample, clear timer messages. 
		 * Does not touch component state variables (port buffers, time_from_start).
		 */
		bool startHook();
		/**
		 * Main control loop: resource_client messages processing, timer syncronization, get actual_pose and calculate next reference pose.
		 * If resource_client->isOperational() operationalHook() is called.
		 */
		void updateHook();
		/**
		 * Destroy goal_active, detach resource_client.
		 */
		void cleanupHook();
};

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

#endif  /*ANIMATION_JOINT_TRAJECTORY_BASE_HPP*/
