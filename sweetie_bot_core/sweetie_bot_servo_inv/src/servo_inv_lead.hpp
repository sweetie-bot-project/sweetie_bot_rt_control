#ifndef OROCOS_SWEETIE_BOT_SERVO_INV_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_SERVO_INV_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <orocos/sensor_msgs/typekit/JointState.h>
#include <orocos/sweetie_bot_hardware_herkulex_msgs/typekit/ServoGoal.h>

class ServoInvLead : public RTT::TaskContext
{
	protected:
		// buffers
		sensor_msgs::JointState joints;
		sweetie_bot_hardware_herkulex_msgs::ServoGoal goals;

	// COMPONENT INTERFACE
	protected:
		// PORTS
		RTT::InputPort<sensor_msgs::JointState> joints_port;
		RTT::OutputPort<sweetie_bot_hardware_herkulex_msgs::ServoGoal> goals_port;

		// PROPERTIES
		double lead;
		
	public:
		ServoInvLead(std::string const& name);
		bool startHook();
		void updateHook();
		void stopHook();
};
#endif
