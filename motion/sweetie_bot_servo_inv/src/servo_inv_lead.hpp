#ifndef OROCOS_SWEETIE_BOT_SERVO_INV_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_SERVO_INV_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>

#include <sweetie_bot_logger/logger.hpp>

#include <sensor_msgs/typekit/JointState.h>
#include <sweetie_bot_herkulex_msgs/typekit/ServoGoal.h>

namespace sweetie_bot {
namespace motion {

class ServoInvLead : public RTT::TaskContext
{
	protected:
		// Logger
#ifdef SWEETIEBOT_LOGGER
		logger::SWEETIEBOT_LOGGER log;
#else
		logger::LoggerRTT log;
#endif

		// buffers
		sensor_msgs::JointState joints;
		sensor_msgs::JointState::_position_type position_perv;
		std::vector<double> gear_ratio_array;
		sweetie_bot_herkulex_msgs::ServoGoal goals;

	// COMPONENT INTERFACE
	protected:
		// PORTS
		RTT::InputPort<sensor_msgs::JointState> joints_port;
		RTT::InputPort<RTT::os::Timer::TimerId> sync_port;
		RTT::OutputPort<sweetie_bot_herkulex_msgs::ServoGoal> goals_port;

		// PROPERTIES
		double lead;
		double period;
		std::vector<std::string> gear_joints;
		std::vector<double> gear_ratios;
	
	protected:
		bool processJointStateSample(const sensor_msgs::JointState& joints);
		
	public:
		ServoInvLead(std::string const& name);
		bool startHook();
		void updateHook();
		void stopHook();
};

}
}
#endif
