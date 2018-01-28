#ifndef OROCOS_SWEETIE_BOT_CONTROLLER_FOLLOW_SATNCE_HPP
#define OROCOS_SWEETIE_BOT_CONTROLLER_FOLLOW_SATNCE_HPP

#include <vector>
#include <string>

#include <rtt/Component.hpp>

#include <std_srvs/SetBool.h>
#include <sensor_msgs/typekit/JointState.h>
#include <geometry_msgs/typekit/PoseStamped.h>
#include <kdl_msgs/typekit/Twist.h>
#include <sweetie_bot_kinematics_msgs/typekit/JointStateAccel.h>
#include <sweetie_bot_kinematics_msgs/typekit/RigidBodyState.h>
#include <sweetie_bot_kinematics_msgs/typekit/SupportState.h>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_orocos_misc/simple_action_server.hpp>
#include <sweetie_bot_resource_control/resource_client.hpp>
#include <sweetie_bot_robot_model/robot_model.hpp>

namespace sweetie_bot {
namespace motion {
namespace controller {


class FollowStance : public RTT::TaskContext
{
	protected:
		// COMPONENT INTERFACE
		//
		// PORTS: input
		RTT::InputPort<sweetie_bot_kinematics_msgs::JointStateAccel> in_joints_accel_port;
		RTT::InputPort<sweetie_bot_kinematics_msgs::RigidBodyState> in_base_port;
		RTT::InputPort<geometry_msgs::PoseStamped> in_base_ref_port;
		RTT::InputPort<RTT::os::Timer::TimerId> sync_port;
		// PORTS: output
		RTT::OutputPort<kdl_msgs::Twist> out_accel_port;
		RTT::OutputPort<sensor_msgs::JointState> out_joints_port;
		RTT::OutputPort<sweetie_bot_kinematics_msgs::SupportState> out_supports_port;
		// PROPERTIES
		std::vector<std::string> support_legs;
		double Kp;
		double Kv;
		double period;
	protected:
		// OPERATIONS: provides
		bool rosSetOperational(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
		// OPERATIONS: requires
		// SERVICES: provides
		// SERVICES: required
		sweetie_bot::motion::RobotModel * robot_model;
		// SERVICES: internal interface
		sweetie_bot::motion::ResourceClientInterface * resource_client;

	protected:
		// COMPONENT STATE
		int n_joints_fullpose; // number of joints in full robot model
		std::vector<int> joint_index;
		// ports buffers
		sweetie_bot_kinematics_msgs::JointStateAccel joints_accel;
		sensor_msgs::JointState joints_ref;
		sweetie_bot_kinematics_msgs::RigidBodyState base;
		sweetie_bot_kinematics_msgs::SupportState supports;
		
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif
	protected:
		bool setupSupports(const vector<string>& support_legs);

	public:
		FollowStance(std::string const& name);

		bool resourceChangedHook();

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
