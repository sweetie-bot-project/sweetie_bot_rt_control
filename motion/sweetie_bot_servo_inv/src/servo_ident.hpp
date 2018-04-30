#ifndef OROCOS_SERVO_IDENT_COMPONENT_HPP
#define OROCOS_SERVO_IDENT_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>

#include <sweetie_bot_logger/logger.hpp>

#include <sensor_msgs/typekit/JointState.h>
#include <sensor_msgs/typekit/BatteryState.h>
#include <sweetie_bot_herkulex_msgs/typekit/ServoGoal.h>
#include <sweetie_bot_servo_model_msg/typekit/ServoModel.h>
#include <sweetie_bot_kinematics_msgs/typekit/JointStateAccel.h>

#include <unordered_map>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>

<<<<<<< HEAD:motion/sweetie_bot_servo_inv/src/servo_ident.hpp
=======
#include <orocos/sweetie_bot_joint_state_accel_msg/typekit/JointStateAccel.h>
#include <orocos/sweetie_bot_herkulex_msgs/typekit/ServoGoal.h>
>>>>>>> faf9920... sweetie_bot_servo_ident: Added servo_goals input port and sign_t function.:motion/sweetie_bot_servo_ident/src/servo_ident.hpp

#include <sweetie_bot_common/ring_buffer.hpp>

namespace sweetie_bot {
namespace motion {

struct servo_model_data {
	unsigned int index;
	bool ident_started;
	boost::numeric::ublas::c_vector<double, 4> alpha;
	boost::numeric::ublas::c_matrix<double, 4, 4> P;
	ring_buffer<double> prec_err_buf;

	servo_model_data();
};

class ServoIdent : public RTT::TaskContext {

	protected:
		// Logger
#ifdef SWEETIEBOT_LOGGER
		logger::SWEETIEBOT_LOGGER log;
#else
		logger::LoggerRTT log;
#endif

		// buffers

		RTT::os::Timer::TimerId timer_id;

		//we need store some last joints, because control_delay may be non-zero
		ring_buffer<sweetie_bot_kinematics_msgs::JointStateAccel> joints_buf;
		ring_buffer<sweetie_bot_herkulex_msgs::ServoGoal> goals_buf;

		ring_buffer<std::vector<double>> measured_pos_buf;
		std::vector<double> pos_measured;
		std::vector<double> position;
		std::vector<double> velocity;

		//map servos names and some helper data
		std::unordered_map<std::string, servo_model_data> servo_models_data;

		//for identification calculations
		boost::numeric::ublas::c_vector<double, 4> phi;
		boost::numeric::ublas::c_vector<double, 4> L;

		const sweetie_bot_kinematics_msgs::JointStateAccel *joints;
		const sweetie_bot_herkulex_msgs::ServoGoal *goals;
		sensor_msgs::JointState joints_measured;
		sweetie_bot_joint_state_accel_msg::JointStateAccel effort_joints;
		sensor_msgs::BatteryState battery_voltage_buf;

		bool models_vector_was_sorted;
		bool sort_servo_models();
		void prepare_buffers_for_new_joints_size(sweetie_bot_kinematics_msgs::JointStateAccel const& jnt);
		double sign_t(double vel);
		double n_sign_t(double vel);
		double calc_median(double a, double b, double c);


	// COMPONENT INTERFACE
	protected:
		// PORTS
		RTT::InputPort<RTT::os::Timer::TimerId> in_sync_step;
		RTT::InputPort<sweetie_bot_kinematics_msgs::JointStateAccel> in_joints_fixed;
		RTT::InputPort<sweetie_bot_herkulex_msgs::ServoGoal> in_goals;
		RTT::InputPort<sensor_msgs::JointState> in_joints_measured;
		RTT::InputPort<sensor_msgs::BatteryState> in_battery_state;

		RTT::OutputPort<std::vector<sweetie_bot_servo_model_msg::ServoModel>> out_servo_models;
		RTT::OutputPort<sweetie_bot_joint_state_accel_msg::JointStateAccel> out_torque_error_sorted;

		// PROPERTIES
		double period;
		unsigned int control_delay;
		double relaxation_factor;
		double error_averaging_time;
		double treshhold;
		double sign_dead_zone;
		double battery_voltage;
		sweetie_bot_servo_model_msg::ServoModel default_servo_model;
		std::vector<sweetie_bot_servo_model_msg::ServoModel> servo_models;
		bool ignore_accel;

	public:
		ServoIdent(std::string const& name);
		bool startHook();
		void updateHook();
		void stopHook();

		bool startIdentification(std::vector<std::string> servos);
		std::vector<std::string> endIdentification();
		bool abortIdentification();
};

}
}

#endif
