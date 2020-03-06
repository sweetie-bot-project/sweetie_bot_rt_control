#ifndef OROCOS_SWEETIE_BOT_CONTROLLER_LOOK_AT_HPP
#define OROCOS_SWEETIE_BOT_CONTROLLER_LOOK_AT_HPP

#include <vector>
#include <string>

#include <rtt/Component.hpp>
#include <rtt/os/Timer.hpp>

#include <std_srvs/SetBool.h>
#include <sensor_msgs/typekit/JointState.h>
#include <geometry_msgs/typekit/PoseStamped.h>
#include <sensor_msgs/typekit/JointState.h>
#include <sweetie_bot_kinematics_msgs/typekit/RigidBodyState.h>
#include <sweetie_bot_kinematics_msgs/typekit/SupportState.h>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_resource_control/resource_client.hpp>
#include <sweetie_bot_robot_model/robot_model.hpp>
#include <sweetie_bot_controller_cartesian/filter_rigid_body_state.hpp>
#include <sweetie_bot_controller_joint_space/filter_joint_state.hpp>

#include <sweetie_bot_resource_control/simple_controller_base.hpp>

namespace sweetie_bot {
namespace motion {
namespace controller {


class LookAt : public SimpleControllerBase
{
	protected:
		// COMPONENT INTERFACE
		//
		// PORTS: input
		RTT::InputPort<sweetie_bot_kinematics_msgs::RigidBodyState> in_limbs_port;
		RTT::InputPort<sweetie_bot_kinematics_msgs::RigidBodyState> in_base_port;
		RTT::InputPort<geometry_msgs::PoseStamped> in_pose_ref_port;
		// PORTS: output
		RTT::OutputPort<sweetie_bot_kinematics_msgs::RigidBodyState> out_limbs_ref_port;
		RTT::OutputPort<sweetie_bot_kinematics_msgs::SupportState> out_supports_port;
		RTT::OutputPort<sensor_msgs::JointState> out_joints_ref_port;
		// PROPERTIES
		std::string chain_name;
		std::vector< std::string > pitch_yaw_joints;

	protected:
		// OPERATIONS: provides
		// OPERATIONS: requires
		RTT::OperationCaller<bool(const sweetie_bot_kinematics_msgs::RigidBodyState&)> poseToJointStatePublish;
		// SERVICES: provides
		// SERVICES: required
		sweetie_bot::motion::RobotModel * robot_model;
		// SERVICES: internal interface
		sweetie_bot::motion::filter::FilterRigidBodyStateInterface * filter_pose; // trajectory smoother
		sweetie_bot::motion::filter::FilterJointStateInterface * filter_joints; // trajectory smoother

	protected:
		// COMPONENT STATE
		int chain_index;
		KDL::Vector target_point; // point where x-axis should points
		KDL::Vector target_position; // preferable position of origin of last_link frame in base_link frame (set on component activation, prevent last link positon change)
		// ports buffers
		sweetie_bot_kinematics_msgs::RigidBodyState limbs; // limb current pose
		sweetie_bot_kinematics_msgs::RigidBodyState limb_ref; // desired limb pose
		sweetie_bot_kinematics_msgs::RigidBodyState limb_next; // next pose calucalated by component
		sweetie_bot_kinematics_msgs::RigidBodyState base; // base_link pose
		sweetie_bot_kinematics_msgs::SupportState supports;
		sensor_msgs::JointState joints_ref; // desired joints pose
		sensor_msgs::JointState joints_next; // new joints pose, calculated by component
		
	public:
		LookAt(std::string const& name);

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

#endif /*OROCOS_SWEETIE_BOT_CONTROLLER_LOOK_AT_HPP*/
