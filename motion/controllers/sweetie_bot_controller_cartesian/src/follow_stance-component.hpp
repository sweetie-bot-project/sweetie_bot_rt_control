#ifndef OROCOS_SWEETIE_BOT_CONTROLLER_FOLLOW_SATNCE_HPP
#define OROCOS_SWEETIE_BOT_CONTROLLER_FOLLOW_SATNCE_HPP

#include <vector>
#include <string>

#include <rtt/Component.hpp>
#include <rtt/os/Timer.hpp>

#include <std_srvs/SetBool.h>
#include <sensor_msgs/typekit/JointState.h>
#include <geometry_msgs/typekit/PoseStamped.h>
#include <sweetie_bot_kinematics_msgs/typekit/RigidBodyState.h>
#include <sweetie_bot_kinematics_msgs/typekit/SupportState.h>
#include <sweetie_bot_kinematics_msgs/typekit/BalanceState.h>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_resource_control/resource_client.hpp>
#include <sweetie_bot_robot_model/robot_model.hpp>
#include <sweetie_bot_controller_cartesian/filter_rigid_body_state.hpp>

#include <sweetie_bot_resource_control/simple_controller_base.hpp>

namespace sweetie_bot {
namespace motion {
namespace controller {


class FollowStance : public SimpleControllerBase
{
	protected:
		// COMPONENT INTERFACE
		//
		// PORTS: input
		RTT::InputPort<sweetie_bot_kinematics_msgs::RigidBodyState> in_limbs_port;
		RTT::InputPort<sweetie_bot_kinematics_msgs::RigidBodyState> in_base_port;
		RTT::InputPort<sweetie_bot_kinematics_msgs::BalanceState> in_balance_port;
		RTT::InputPort<geometry_msgs::PoseStamped> in_base_ref_port;
		// PORTS: output
		RTT::OutputPort<sweetie_bot_kinematics_msgs::RigidBodyState> out_limbs_ref_port;
		RTT::OutputPort<sweetie_bot_kinematics_msgs::RigidBodyState> out_base_ref_port;
		RTT::OutputPort<sweetie_bot_kinematics_msgs::SupportState> out_supports_port;
		// PROPERTIES
		// std::vector<std::string> support_legs; use controlled_chains instead
		bool pose_feedback;
		bool balance_check;
		bool balance_keep;
		double safe_pose_z_max;
		double safe_pose_z_min;
		unsigned int activation_delay;

	protected:
		// OPERATIONS: provides
		// OPERATIONS: requires
		RTT::OperationCaller<bool(const sweetie_bot_kinematics_msgs::RigidBodyState&)> poseToJointStatePublish;
		// SERVICES: provides
		// SERVICES: required
		sweetie_bot::motion::RobotModel * robot_model;
		// SERVICES: internal interface
		sweetie_bot::motion::filter::FilterRigidBodyStateInterface * filter; // trajectory smoother

	protected:
		// COMPONENT STATE
		std::vector<KDL::Frame> support_leg_anchors; // positions of legs in world frame (WARNING at start component!)
		std::vector<unsigned int> support_leg_index; // position index of support legs in in_limbs_fixed message
		bool ik_success;
		unsigned int activation_delay_counter;
		// ports buffers
		sweetie_bot_kinematics_msgs::RigidBodyState base; // current pose received from in_base
		sweetie_bot_kinematics_msgs::RigidBodyState base_next; // next pose calucalated by component
		sweetie_bot_kinematics_msgs::RigidBodyState base_ref; // reference pose received from in_base_ref
		sweetie_bot_kinematics_msgs::BalanceState balance; // balance 
		sweetie_bot_kinematics_msgs::RigidBodyState limbs; // calculated reference pose to publish on out_port
		sweetie_bot_kinematics_msgs::RigidBodyState limbs_full; // received limbs state on in_limbs_fixed
		sweetie_bot_kinematics_msgs::SupportState supports; // support state to publish
		
	protected:
		bool setupSupports(const std::vector<std::string>& support_legs);
		void checkBalance() ;
		bool isInsideSupportPolygone(const KDL::Vector point) ;

	public:
		FollowStance(std::string const& name);

	protected:
		bool processResourceSet_impl(const std::vector<std::string>& set_operational_goal_resources, std::vector<std::string>& resources_to_request);
		bool resourceChangedHook_impl(const std::vector<std::string>& set_operational_goal_resources, const std::vector<std::string>& requested_resources);

		bool configureHook_impl(); 
		bool startHook_impl(StateChangeReason reason);
		void updateHook_impl();
		void stopHook_impl(StateChangeReason reason);
		void cleanupHook_impl();
};

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

#endif /*OROCOS_SWEETIE_BOT_CONTROLLER_FOLLOW_SATNCE_HPP*/
