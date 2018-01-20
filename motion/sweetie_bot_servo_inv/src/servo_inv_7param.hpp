#ifndef OROCOS_SWEETIE_BOT_SERVO_INV_COMPONENT_HPP
#define OROCOS_SWEETIE_BOT_SERVO_INV_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <sweetie_bot_logger/logger.hpp>

#include <sensor_msgs/typekit/JointState.h>
#include <sensor_msgs/typekit/BatteryState.h>
#include <sweetie_bot_herkulex_msgs/typekit/ServoGoal.h>
#include <sweetie_bot_servo_model_msg/typekit/ServoModel.h>
#include <sweetie_bot_kinematics_msgs/typekit/JointStateAccel.h>

namespace sweetie_bot {
namespace motion {

class ServoInv7Param : public RTT::TaskContext {

	protected:
		// Logger
#ifdef SWEETIEBOT_LOGGER
		logger::SWEETIEBOT_LOGGER log;
#else
		logger::LoggerRTT log;
#endif

		// buffers
		sweetie_bot_kinematics_msgs::JointStateAccel joints;
		sweetie_bot_herkulex_msgs::ServoGoal goals;

		std::vector<sweetie_bot_servo_model_msg::ServoModel> models;
		sensor_msgs::BatteryState battery_voltage_buf;

		// variabels
		bool models_vector_was_sorted;

		//helper functions
		bool sort_servo_models();
		void prepare_buffers_for_new_joints_size();

	// COMPONENT INTERFACE
	protected:
		// PORTS
		RTT::InputPort<sweetie_bot_kinematics_msgs::JointStateAccel> in_joints_fixed;
		RTT::InputPort<std::vector<sweetie_bot_servo_model_msg::ServoModel>> in_servo_models;
		RTT::InputPort<sensor_msgs::BatteryState> in_battery_state;

		RTT::OutputPort<sweetie_bot_herkulex_msgs::ServoGoal> out_goals;

		// PROPERTIES
		double period;

		std::vector<sweetie_bot_servo_model_msg::ServoModel> servo_models;
		double battery_voltage;

	public:
		ServoInv7Param(std::string const& name);
		bool startHook();
		void updateHook();
		void stopHook();
};

}
}
#endif
