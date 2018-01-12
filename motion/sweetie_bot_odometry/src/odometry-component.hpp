#ifndef OROCOS_SWEETIE_BOT_ODOMETRY_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_ODOMETRY_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <kdl/frames.hpp>
#include <tf2_msgs/typekit/TFMessage.h>
#include <orocos/kdl_typekit/typekit/Types.hpp>

#include <sweetie_bot_logger/logger.hpp>

#include <sweetie_bot_kinematics_msgs/typekit/RigidBodyState.h>
#include <sweetie_bot_kinematics_msgs/typekit/SupportState.h>

namespace sweetie_bot {
namespace motion {

class Odometry : public RTT::TaskContext
{
	protected:
		// COMPONENT INTERFACE
		//
		// PORTS: input
		RTT::InputPort<sweetie_bot_kinematics_msgs::RigidBodyState> limbs_port;
		RTT::InputPort<sweetie_bot_kinematics_msgs::SupportState> support_port;
		RTT::InputPort<sweetie_bot_kinematics_msgs::RigidBodyState> body_reset_port;
		// PORTS: output
		RTT::OutputPort<sweetie_bot_kinematics_msgs::RigidBodyState> body_port;
		RTT::OutputPort<tf2_msgs::TFMessage> tf_port;
		// PROPERTIES
		std::vector<std::string> legs;
		std::vector<double> contact_points_prop; // KDL::Vector defined as vector<double> for ROS parameter compatibility
		std::string odometry_frame;
		std::string base_link_tf_prefix;
	protected:
		// OPERATIONS: provides
		// OPERATIONS: requires
		// SERVICES: provides
		// SERVICES: required
		// SERVICES: internal interface

		// logger
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif
	protected:
		struct LimbState {
			std::string name;
			bool was_in_contact;
			KDL::Frame previous_pose;

			LimbState(const string& _name) : 
				name(_name),
				was_in_contact(false),	
				previous_pose(KDL::Frame::Identity())
			{} 
		};

	protected:
		// COMPONENT STATE
		std::vector<LimbState> limbs;
		std::vector<KDL::Vector> default_contact_points;
		std::vector<KDL::Vector> contact_points;
		std::vector<KDL::Vector> contact_points_prev;
		KDL::Frame body_anchor;
		unsigned int too_few_contact_warn_counter;
		// ports buffers
		sweetie_bot_kinematics_msgs::RigidBodyState limb_poses; 
		sweetie_bot_kinematics_msgs::RigidBodyState body_reset_pose; 
		sweetie_bot_kinematics_msgs::SupportState support_state;
		sweetie_bot_kinematics_msgs::RigidBodyState body_pose; 
		tf2_msgs::TFMessage body_tf;

	public:
		Odometry(std::string const& name);

		bool integrateBodyPose();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
};

} // namespace motion
} // namespace sweetie_bot
#endif
