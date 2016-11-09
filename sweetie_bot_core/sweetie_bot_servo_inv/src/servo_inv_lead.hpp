#ifndef OROCOS_SWEETIE_BOT_SERVO_INV_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_SERVO_INV_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>

#include <sweetie_bot_logger/logger.hpp>

#include <orocos/sensor_msgs/typekit/JointState.h>
#include <sensor_msgs/JointState.h>
#include <orocos/sweetie_bot_hardware_herkulex_msgs/typekit/ServoGoal.h>

namespace sweetie_bot
{

class ServoInvLead : public RTT::TaskContext
{
	protected:
		// Logger
#ifdef SWEETIEBOT_LOGGER
		SWEETIEBOT_LOGGER log;
#else
		LoggerRTT log;
#endif

		// buffers
		sensor_msgs::JointState joints;
		sensor_msgs::JointState::_position_type position_perv;
		sweetie_bot_hardware_herkulex_msgs::ServoGoal goals;

	// COMPONENT INTERFACE
	protected:
		// PORTS
		RTT::InputPort<sensor_msgs::JointState> joints_port;
		RTT::InputPort<RTT::os::Timer::TimerId> sync_port;
		RTT::OutputPort<sweetie_bot_hardware_herkulex_msgs::ServoGoal> goals_port;

		// PROPERTIES
		double lead;
		double period;
		
	public:
		ServoInvLead(std::string const& name);
		bool startHook();
		void updateHook();
		void stopHook();
};

}
#endif
