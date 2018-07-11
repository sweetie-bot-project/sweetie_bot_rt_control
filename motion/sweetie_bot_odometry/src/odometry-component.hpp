#ifndef OROCOS_SWEETIE_BOT_ODOMETRY_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_ODOMETRY_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <kdl/frames.hpp>
#include <tf2_msgs/typekit/TFMessage.h>
#include <orocos/kdl_typekit/typekit/Types.hpp>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_robot_model/robot_model.hpp>

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
		//RTT::OutputPort<geometry_msgs::TwistStamped> twist_port;
		// PROPERTIES
		std::vector<std::string> legs;
		std::string default_contact; 
		bool force_contact_z_to_zero;
		bool zero_twist_if_no_contacts;
		std::string odometry_frame;
		std::string base_link_tf_prefix;
	protected:
		// OPERATIONS: provides
		// OPERATIONS: requires
		// SERVICES: provides
		// SERVICES: required
		RobotModel * robot_model;
		// SERVICES: internal interface

		// logger
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif
	protected:
		struct LimbState {
			std::string name; // kinematic chain name
			int index;        // index of kinematic chain

			std::string contact_name;  // contact name
			bool is_in_contact;
			bool was_in_contact;
			bool contact_name_changed;

			std::vector<KDL::Vector> contact_points_limb; // coordinates in limb end segment frame

			LimbState(const string& _name) : 
				name(_name),
				is_in_contact(false),
				was_in_contact(false)
			{} 
		};

	protected:
		// COMPONENT STATE
		std::vector<LimbState> limbs;
		std::vector<KDL::Vector> contact_points_anchor; // coordinates in world frame
		std::vector<KDL::Vector> contact_points_body; // coordinates in body frame
		unsigned int too_few_contact_warn_counter;
		// ports buffers
		sweetie_bot_kinematics_msgs::RigidBodyState limb_poses; 
		sweetie_bot_kinematics_msgs::RigidBodyState body_reset_pose; 
		sweetie_bot_kinematics_msgs::SupportState support_state;
		sweetie_bot_kinematics_msgs::RigidBodyState body_pose; 
		tf2_msgs::TFMessage body_tf;
		//geometry_msgs::TwistStamped body_twist;

	protected:
		void updateAnchor(bool check_support_state);
		void estimateVelocity();
		void estimateRigidBodyPose(const std::vector<KDL::Vector>& contact_points_body, const std::vector<KDL::Vector>& contact_points_anchor, KDL::Frame& T);
		bool integrateBodyPose();

	public:
		Odometry(std::string const& name);


		// OPERATIONS
		bool setIdentity();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
};

} // namespace motion
} // namespace sweetie_bot
#endif
