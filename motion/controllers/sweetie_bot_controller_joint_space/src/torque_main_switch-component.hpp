#ifndef  TORQUE_MAIN_SWITCH_COMPONENT_HPP
#define  TORQUE_MAIN_SWITCH_COMPONENT_HPP

#include <rtt/Component.hpp>
#include <rtt/os/Timer.hpp>

#include <std_srvs/SetBool.h>
#include <orocos/sensor_msgs/typekit/JointState.h>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_resource_control/resource_client.hpp>
#include <sweetie_bot_robot_model/robot_model.hpp>

#include <sweetie_bot_resource_control/actionlib_controller_base.hpp>

namespace sweetie_bot {
namespace motion {
namespace controller {


class TorqueMainSwitch : public ActionlibControllerBase
{
	protected:
		typedef sensor_msgs::JointState JointState;

	protected:
		// COMPONENT INTERFACE
		//
		// PORTS: input
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
		std::vector< RTT::OperationCaller<bool(bool)> > torque_off_callers; // operation callers to control servos torque
		// std::vector< std::string > controlled_chains; // list of controlled chains (inherited from ActionlibControllerBase)
		// ports buffers
		JointState actual_fullpose; // buffer for input port in_joints_port
		
	public:
		TorqueMainSwitch(std::string const& name);

	protected:
		bool setSchedulersActive(bool is_active);
		bool setAllServosTorqueFree(bool torque_is_off);

		bool checkResourceSet_impl(const std::vector<std::string>& desired_resource_set);

		bool configureHook_impl(); 
		bool startHook_impl();
		void updateHook_impl();
		void stopHook_impl();
		void cleanupHook_impl();
};

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

#endif  /*TORQUE_MAIN_SWITCH_COMPONENT_HPP*/
