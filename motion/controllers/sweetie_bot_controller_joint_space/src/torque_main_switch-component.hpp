#ifndef  TORQUE_MAIN_SWITCH_COMPONENT_HPP
#define  TORQUE_MAIN_SWITCH_COMPONENT_HPP

#include <rtt/Component.hpp>
#include <rtt/os/Timer.hpp>

#include <std_srvs/SetBool.h>
#include <orocos/sensor_msgs/typekit/JointState.h>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_resource_control/resource_client.hpp>
#include <sweetie_bot_robot_model/robot_model.hpp>

#include <sweetie_bot_resource_control/simple_controller_base.hpp>

namespace sweetie_bot {
namespace motion {
namespace controller {

class TorqueMainSwitch : public SimpleControllerBase
{
	protected:
		typedef sensor_msgs::JointState JointState;
		struct JointInfo {
			std::string name;
			int index_fullpose;
			RTT::OperationCaller<bool(std::string, bool)> * setTorqueFree_caller;
		};
		struct HerkulexGroupInfo {
			std::string array;
			std::string sched;
			RTT::OperationCaller<bool(std::string, bool)> setTorqueFree_caller;
		};
		struct JointGroupInfo {
			std::vector<JointInfo> joints;
			std::vector<int> herkulex_groups_induces;
		};

	protected:
		// COMPONENT INTERFACE
		//
		// PORTS: input
		RTT::InputPort<RTT::os::Timer::TimerId> sync_port;
		RTT::InputPort<JointState> in_joints_port;
		// PORTS: output
		RTT::OutputPort<JointState> out_joints_port;
		// PROPERTIES
		std::vector<std::string> herkulex_arrays;
		std::vector<std::string> herkulex_scheds;
		bool velocity_zeroing;
	protected:
		// OPERATIONS: provides
		// OPERATIONS: requires
		// SERVICES: provides
		// SERVICES: required
		sweetie_bot::motion::RobotModel * robot_model; // joints list, kinematics chains access
		// SERVICES: internal interface

	protected:
		// COMPONENT STATE
		std::map< std::string, JointGroupInfo > joint_groups_index; // groups of joints (accodring to RobotModel)
		std::vector< HerkulexGroupInfo > herkulex_groups; // herkulex control groups
		std::vector< std::string > controlled_groups; // list of groups
		// ports buffers
		JointState actual_fullpose; // buffer for input port in_joints_port
		JointState pose_published; // buffer for output port
		
	public:
		TorqueMainSwitch(std::string const& name);

		bool setSchedulersActive(const JointGroupInfo& group, bool is_active);
		bool setAllServosTorqueFree(bool torque_is_off);

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

#endif  /*TORQUE_MAIN_SWITCH_COMPONENT_HPP*/
