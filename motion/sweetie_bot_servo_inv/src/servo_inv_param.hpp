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

class ServoInvParam : public RTT::TaskContext 
{
	public:
		typedef sweetie_bot_kinematics_msgs::JointStateAccel JointStateAccel;
		typedef sweetie_bot_servo_model_msg::ServoModel ServoModel;
		typedef sweetie_bot_herkulex_msgs::ServoGoal ServoGoal;

	// COMPONENT INTERFACE
	protected:
		// PORTS
		RTT::InputPort<sweetie_bot_kinematics_msgs::JointStateAccel> in_joints_fixed;
		RTT::InputPort<std::vector<sweetie_bot_servo_model_msg::ServoModel> > in_servo_models;
		RTT::InputPort<sensor_msgs::BatteryState> in_battery_state;

		RTT::OutputPort<sweetie_bot_herkulex_msgs::ServoGoal> out_goals;

		// PROPERTIES
		double period; 
		double lead;
		bool extrapolate_position;
		sweetie_bot_servo_model_msg::ServoModel default_servo_model; 
		std::vector<sweetie_bot_servo_model_msg::ServoModel> servo_models; // known models of servos
		double sign_dead_zone; // speed smaller then sign_dead_zoneis assumed zero
		double battery_voltage; // default battery voltage

	protected:
		// COMPONENT STATE
		std::vector<int> servo_model_index; // map position in JointStataAccel msg to servo_models list.
		// port buffers
		sweetie_bot_kinematics_msgs::JointStateAccel joints;
		std::vector<sweetie_bot_servo_model_msg::ServoModel> models;
		sweetie_bot_herkulex_msgs::ServoGoal goals;
		sensor_msgs::BatteryState battery_state;

		// Logger
#ifdef SWEETIEBOT_LOGGER
		logger::SWEETIEBOT_LOGGER log;
#else
		logger::LoggerRTT log;
#endif
	protected:
		// helper functions
		double sign(double vel) {
			if (vel < -sign_dead_zone) return -1.0;
			else if (vel > sign_dead_zone) return 1.0;
			else return 0.0;
		}
		// Create index according to provided joint_names. Resize port buffers.
		void setupServoModelIndex(const std::vector<std::string>& joint_names); 

	public:
		ServoInvParam(std::string const& name);

		bool startHook();
		void updateHook();
		void stopHook();
};

}
}
#endif
