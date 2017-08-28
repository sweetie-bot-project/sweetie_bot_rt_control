#ifndef OROCOS_SERVO_IDENT_COMPONENT_HPP
#define OROCOS_SERVO_IDENT_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <sweetie_bot_logger/logger.hpp>

#include <orocos/sensor_msgs/typekit/JointState.h>
#include <orocos/sensor_msgs/typekit/BatteryState.h>
#include <orocos/sweetie_bot_herkulex_msgs/typekit/ServoGoal.h>
#include <orocos/sweetie_bot_servo_model_msg/typekit/ServoModel.h>

#include <unordered_map>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>

#include "ring_buffer.hpp"

namespace sweetie_bot {
namespace motion {

struct servo_model_data {
	unsigned int index;
	bool ident_started;
	boost::numeric::ublas::c_vector<double, 5> alpha;
	boost::numeric::ublas::c_matrix<double, 5, 5> P;
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

		//we need store some last joints, because control_delay may be non-zero
		ring_buffer<sensor_msgs::JointState> joints_buf;

		//map servos names and some helper data
		std::unordered_map<std::string, servo_model_data> servo_models_data;

		//for identification calculations
		boost::numeric::ublas::c_vector<double, 5> phi;
		boost::numeric::ublas::c_vector<double, 5> L;

                const sensor_msgs::JointState *joints;
                sensor_msgs::JointState joints_measured;
                sensor_msgs::JointState effort_joints;
		sensor_msgs::BatteryState battery_voltage_buf;

		//count of cycles after getting last measured joint for servo with same number
		std::vector<unsigned int> num_periods;

		std::vector<double> velocity_prev;
		std::vector<double> velocity_measured_prev;

		bool models_vector_was_sorted;
		bool sort_servo_models();
		void prepare_buffers_for_new_joints_size(sensor_msgs::JointState const& jnt);


	// COMPONENT INTERFACE
        protected:
                // PORTS
                RTT::InputPort<sensor_msgs::JointState> in_joints_fixed;
                RTT::InputPort<sensor_msgs::JointState> in_joints_measured;
		RTT::InputPort<sensor_msgs::BatteryState> in_battery_state;

		RTT::OutputPort<std::vector<sweetie_bot_servo_model_msg::ServoModel>> out_servo_models;
                RTT::OutputPort<sensor_msgs::JointState> out_torque_error_sorted;

		// PROPERTIES
		double period;
                unsigned int control_delay;
                double relaxation_factor;
                double error_averaging_time;
		double treshhold;
		double battery_voltage;
		std::vector<sweetie_bot_servo_model_msg::ServoModel> servo_models;

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
