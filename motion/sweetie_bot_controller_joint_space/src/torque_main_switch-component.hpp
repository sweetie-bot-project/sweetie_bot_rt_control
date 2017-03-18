#ifndef  TORQUE_MAIN_SWITCH_COMPONENT_HPP
#define  TORQUE_MAIN_SWITCH_COMPONENT_HPP

#include <rtt/Component.hpp>
#include <rtt/os/Timer.hpp>

#include <std_srvs/SetBool.h>
#include <orocos/sensor_msgs/typekit/JointState.h>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_resource_control/resource_client.hpp>
#include <sweetie_bot_robot_model/robot_model-simple.hpp>

namespace sweetie_bot {
namespace motion {
namespace controller {


class TorqueMainSwitch : public RTT::TaskContext
{
	protected:
		typedef sensor_msgs::JointState JointState;

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
	protected:
		// OPERATIONS: provides
		bool rosSetOperational(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
		// OPERATIONS: requires
		// SERVICES: provides
		// SERVICES: required
		sweetie_bot::motion::RobotModel * robot_model; // joints list, kinematics chains access
		// SERVICES: internal interface
		sweetie_bot::motion::ResourceClientInterface * resource_client; // resource client

	protected:
		// COMPONENT STATE
		std::vector< RTT::OperationCaller<bool(bool)> > torque_off_callers; // operation callers to control servos torque
		std::vector< std::string > controlled_chains; // list of controlled chains
		// ports buffers
		JointState actual_fullpose; // buffer for input port in_joints_port
		
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif

	public:
		TorqueMainSwitch(std::string const& name);

		bool setSchedulersActive(bool is_active);
		bool setAllServosTorqueFree(bool torque_is_off);

		bool configureHook(); 
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

#endif  /*TORQUE_MAIN_SWITCH_COMPONENT_HPP*/
