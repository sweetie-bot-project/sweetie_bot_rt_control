#ifndef OROCOS_SWEETIE_BOT_SERVO_EXTRAPOLATE_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_SERVO_EXTRAPOLATE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>

#include <sweetie_bot_logger/logger.hpp>

#include <sensor_msgs/typekit/JointState.h>
#include <sweetie_bot_herkulex_msgs/typekit/ServoGoal.h>

namespace sweetie_bot {
namespace motion {

class ServoInvExtrapolate : public RTT::TaskContext
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
		sweetie_bot_herkulex_msgs::ServoGoal goals;

	// COMPONENT INTERFACE
	protected:
		// PORTS
		RTT::InputPort<sensor_msgs::JointState> joints_port;
		RTT::OutputPort<sweetie_bot_herkulex_msgs::ServoGoal> goals_port;

		// PROPERTIES
		double lead;
		double period;
		bool extrapolate_position;
	
	protected:
		bool processJointStateSample(const sensor_msgs::JointState& joints);
		
	public:
		ServoInvExtrapolate(std::string const& name);
		bool startHook();
		void updateHook();
		void stopHook();
};

}
}
#endif
