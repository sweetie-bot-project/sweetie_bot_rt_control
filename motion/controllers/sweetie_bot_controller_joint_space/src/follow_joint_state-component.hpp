#ifndef  FOLLOW_JOINT_STATE_COMPONENT_HPP
#define  FOLLOW_JOINT_STATE_COMPONENT_HPP 

#include <vector>
#include <string>
#include <unordered_map>

#include <rtt/Component.hpp>
#include <rtt/os/Timer.hpp>

#include <std_srvs/SetBool.h>
#include <sensor_msgs/typekit/JointState.h>
#include <sweetie_bot_kinematics_msgs/typekit/SupportState.h>
#include <sweetie_bot_orocos_misc/simple_action_server.hpp>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_resource_control/resource_client.hpp>
#include <sweetie_bot_robot_model/robot_model.hpp>
#include <sweetie_bot_controller_joint_space/filter_joint_state.hpp>
#include <sweetie_bot_orocos_misc/simple_action_server.hpp>

#include <sweetie_bot_resource_control/simple_controller_base.hpp>

namespace sweetie_bot {
namespace motion {
namespace controller {


class FollowJointState : public SimpleControllerBase
{
	protected:
		// COMPONENT INTERFACE
		//
		// PORTS: input
		RTT::InputPort<sensor_msgs::JointState> in_joints_port;
		RTT::InputPort<sensor_msgs::JointState> in_joints_ref_port;
		// PORTS: output
		RTT::OutputPort<sensor_msgs::JointState> out_joints_port;
		RTT::OutputPort<sensor_msgs::JointState> out_joints_src_reset_port;
		RTT::OutputPort<sweetie_bot_kinematics_msgs::SupportState> out_supports_port;
		// PROPERTIES
		double activation_delay;
		bool publish_supports;
	protected:
		// OPERATIONS: provides
		// OPERATIONS: requires
		// SERVICES: provides
		// SERVICES: required
		sweetie_bot::motion::RobotModel * robot_model; // joints list, kinematics chains access
		int n_joints_fullpose;
		// SERVICES: internal interface
		sweetie_bot::motion::filter::FilterJointStateInterface * filter; // trajectory smoother

	protected:
		struct JointIndex {
			unsigned int index_fullpose; // joint index in full pose vector
			unsigned int index; // joint index in controlled JointState vector
			JointIndex() {}
			JointIndex(unsigned int _index_fullpose, unsigned int _index) : index_fullpose(_index_fullpose), index(_index) {}
		};
		typedef std::unordered_map<std::string, JointIndex> JointIndexes;

	protected:
		// COMPONENT STATE
		JointIndexes controlled_joints; // joint indexes cache
		RTT::os::TimeService::ticks activation_timestamp;
		// ports buffers
		sensor_msgs::JointState ref_pose_unsorted; // buffer for input port in_joints_ref_port
		sensor_msgs::JointState actual_fullpose; // buffer for input port in_joints_port
		sensor_msgs::JointState actual_pose; // controlled joints actual position
		sensor_msgs::JointState ref_pose; // controlled joints ref position
		sweetie_bot_kinematics_msgs::SupportState supports; // contact list buffer

	public:
		FollowJointState(std::string const& name);

	protected:
		bool resourceChangedHook_impl(const std::vector<std::string>& requested_resource_set);
		bool formJointIndex(const std::vector<std::string>& controlled_chains);

		bool configureHook_impl();
		bool startHook_impl();
		void updateHook_impl();
		void stopHook_impl();
		void cleanupHook_impl();
};

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

#endif  /*FOLLOW_JOINT_STATE_COMPONENT_HPP*/
