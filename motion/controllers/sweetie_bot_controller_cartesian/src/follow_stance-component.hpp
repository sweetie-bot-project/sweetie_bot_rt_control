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
		RTT::InputPort<sweetie_bot_kinematics_msgs::RigidBodyState> in_limbs_port;
		RTT::InputPort<sweetie_bot_kinematics_msgs::RigidBodyState> in_base_port;
		RTT::InputPort<geometry_msgs::PoseStamped> in_base_ref_port;
		RTT::InputPort<RTT::os::Timer::TimerId> sync_port;
		// PORTS: output
		RTT::OutputPort<sweetie_bot_kinematics_msgs::RigidBodyState> out_limbs_ref_port;
		RTT::OutputPort<sweetie_bot_kinematics_msgs::RigidBodyState> out_base_ref_port;
		RTT::OutputPort<sweetie_bot_kinematics_msgs::SupportState> out_supports_port;
		// PROPERTIES
		std::vector<std::string> support_legs;
		double Kp;
		double Kv;
		double period;
		bool base_pose_feedback;

	protected:
		// OPERATIONS: provides
		bool rosSetOperational(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
		// OPERATIONS: requires
		RTT::OperationCaller<bool(const sweetie_bot_kinematics_msgs::RigidBodyState&)> poseToJointStatePublish;
		// SERVICES: provides
		// SERVICES: required
		sweetie_bot::motion::RobotModel * robot_model;
		// SERVICES: internal interface
		sweetie_bot::motion::ResourceClientInterface * resource_client;

	protected:
		// COMPONENT STATE
		std::vector<KDL::Frame> support_leg_anchors; 
		bool ik_success;
		// ports buffers
		sweetie_bot_kinematics_msgs::RigidBodyState base; // current pose
		sweetie_bot_kinematics_msgs::RigidBodyState base_next; // next pose calucalated by component
		KDL::Frame base_ref; // reference pose
		sweetie_bot_kinematics_msgs::RigidBodyState limbs;
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
