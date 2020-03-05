#ifndef OROCOS_SWEETIE_BOT_SERVO_INV_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_SERVO_INV_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>

#include <sweetie_bot_logger/logger.hpp>
#include <sweetie_bot_common/ring_buffer.hpp>

#include <sensor_msgs/typekit/JointState.h>
#include <sweetie_bot_kinematics_msgs/typekit/JointStateAccel.h>

#include <sweetie_bot_common/ring_buffer.hpp>

namespace sweetie_bot {
namespace motion {

class JointsDiff : public RTT::TaskContext
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
		sweetie_bot_kinematics_msgs::JointStateAccel joints_accel;

		ring_buffer<std::vector<double>> joints_buf;

		std::vector<double> vel_prev;
		std::vector<double> filter;
		std::vector<double> d_filter;
		std::vector<double> d2_filter;

		unsigned int njoints;
		bool new_cycle;

	// COMPONENT INTERFACE
	protected:
		// PORTS
		RTT::InputPort<sensor_msgs::JointState> joints_port;
		RTT::InputPort<RTT::os::Timer::TimerId> sync_port;
		RTT::OutputPort<sweetie_bot_kinematics_msgs::JointStateAccel> joints_accel_port;

		// PROPERTIES
		double period;
		bool filter_position;
		bool calc_velocity;
		bool calc_acceleration;
		std::vector<double> filter_param;
		std::vector<double> d_filter_param;
		std::vector<double> d2_filter_param;

	public:
		JointsDiff(std::string const& name);
		bool startHook();
		void updateHook();
		void stopHook();
};

}
}
#endif
