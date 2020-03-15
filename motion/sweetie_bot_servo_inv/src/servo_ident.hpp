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
#include <Eigen/Core>

namespace sweetie_bot {
namespace motion {


template<size_t max_delay> class SampleHistory;

template<size_t max_delay> class TimeDiscrete 
{
	friend class SampleHistory<max_delay>;

	protected:
		size_t index;
	
	public:
		TimeDiscrete() : index(0) {}
		TimeDiscrete(const TimeDiscrete<max_delay>& _t) = default;

		TimeDiscrete<max_delay>& operator=(TimeDiscrete<max_delay>& _t) = default;
		void operator-=(int shift) {
			index -= shift;
			index %= max_delay;
		}
		void operator+=(int shift) {
			index += shift;
			index %= max_delay;
		}
		friend TimeDiscrete<max_delay> operator+(TimeDiscrete<max_delay> lhs, int rhs) {
			lhs += rhs; 
			return lhs;
		}
		friend TimeDiscrete<max_delay> operator+(int lhs, TimeDiscrete<max_delay> rhs) {
			return rhs + lhs;
		}
		friend TimeDiscrete<max_delay> operator-(TimeDiscrete<max_delay> lhs, int rhs) {
			lhs -= rhs; 
			return lhs;
		}

		operator unsigned int() { return index; }
};

template<size_t max_delay> class TimeContinous
{
	friend class SampleHistory<max_delay>;

	protected:
		size_t index_left, index_right;
		double alpha;

	public:
		TimeContinous() : index_left(0), index_right(1), alpha(1.0) {}
		TimeContinous(const TimeContinous<max_delay>& _t) = default;
		TimeContinous(TimeDiscrete<max_delay> td) {
			index_left = td;
			index_right = td + 1; 
		}

		TimeContinous<max_delay>& operator=(const TimeContinous<max_delay>& _t) = default;

		void operator+=(double shift) {
			double time = index_left + shift;
			double time_floor = std::floor(time);
			index_left = (size_t) (int) std::floor(time);
			index_left %= max_delay; 
			index_right = (index_left + 1u) % max_delay;
			if (time == time_floor) alpha = 1.0;
			else alpha = 1.0 - (time - time_floor);
		}
		void operator-=(double shift) {
			*this += -shift;
		}
		void operator+=(int shift) {
			index_left += shift; index_left %= max_delay;
			index_right += shift; index_right %= max_delay;
		}
		void operator-=(int shift) {
			*this += -shift;
		}
		friend TimeContinous<max_delay> operator+(TimeContinous<max_delay> lhs, double rhs) {
			lhs += rhs; 
			return lhs;
		}
		friend TimeContinous<max_delay> operator-(TimeContinous<max_delay> lhs, double rhs) {
			lhs -= rhs; 
			return lhs;
		}
};


template<size_t max_delay> class SampleHistory {
	protected:
		std::array<double, max_delay> buffer;

	public:
		size_t size() { return buffer.size(); }

		double& get(const TimeDiscrete<max_delay> t) { return buffer[t.index]; }
		double get(const TimeDiscrete<max_delay> t) const { return buffer[t.index]; }

		double get(const TimeContinous<max_delay>& t) const {
			return t.alpha*buffer[t.index_left] + (1.0 - t.alpha)*buffer[t.index_right];
		}
};


class ServoIdent : public RTT::TaskContext {
	public:
		typedef sweetie_bot_kinematics_msgs::JointStateAccel JointStateAccel;
		typedef sensor_msgs::JointState JointState;
		typedef sweetie_bot_servo_model_msg::ServoModel ServoModel;
		typedef sweetie_bot_herkulex_msgs::ServoGoal ServoGoal;
	
	public:	
		static const size_t max_cycle_delay = 4;

	protected:

		struct ServoData {
			// data samles
			SampleHistory<max_cycle_delay> pos_measured;
			SampleHistory<max_cycle_delay> vel_measured;
			SampleHistory<max_cycle_delay> accel_ref;
			SampleHistory<max_cycle_delay> effort_ref;
			SampleHistory<max_cycle_delay> goal;
			double playtime;

			// model parameters
			ServoModel model;

			// identification
			bool ident_started;
			Eigen::Matrix4d P; 
			double pred_error_sq_avg;

			ServoData() : playtime(0.0), ident_started(false), pred_error_sq_avg(0.0) {}
		};

	// COMPONENT INTERFACE
	protected:
		// PORTS
		RTT::InputPort<RTT::os::Timer::TimerId> in_sync_step;
		RTT::InputPort<JointStateAccel> in_joints_ref_fixed;
		RTT::InputPort<ServoGoal> in_goals_fixed;
		RTT::InputPort<JointState> in_joints_measured;
		RTT::InputPort<sensor_msgs::BatteryState> in_battery_state;

		RTT::OutputPort<std::vector<ServoModel>> out_servo_models;
		RTT::OutputPort<JointState> out_torque_error_fixed;

		// PROPERTIES
		double period;
		double control_delay;
		int diff_cycles;
		double min_velocity;
		double relaxation_factor;
		double error_averaging_time;
		double threshold;
		double sign_dead_zone;
		double battery_voltage;
		ServoModel default_servo_model;
		std::vector<ServoModel> servo_models;
		//bool ignore_accel;

		// OPERATIONS
		bool startIdentification(std::vector<std::string> servos);
		std::vector<std::string> endIdentification();
		bool abortIdentification();

	protected:
		// Logger
#ifdef SWEETIEBOT_LOGGER
		logger::SWEETIEBOT_LOGGER log;
#else
		logger::LoggerRTT log;
#endif

	protected:
		//COMPONENT STATE
		// store information about servos. The order is the same as in joints_ref message
		std::vector<ServoData> servos;  
		// map servos names to index in servos array and coresponding position in joints and goals messages.
		std::unordered_map<std::string, int> servos_index;
		// error averaging
		double pred_error_avg_alpha;
		// time and counters
		unsigned int history_update_counter;	
		TimeDiscrete<max_cycle_delay> history_cycle_now;

		// port buffers
		JointState joints_measured;
		JointStateAccel joints_ref;
		ServoGoal goals;
		JointState joints_effort;
		sensor_msgs::BatteryState battery_state;
	
	protected:

		void setupServoData(const std::vector<std::string>& joint_names);

		double sign(double vel) {
			if (vel < -sign_dead_zone) return -1;
			else if (vel > sign_dead_zone) return 1;
			else return 0;
		}

		double n_sign(double vel) {
			if (vel < -sign_dead_zone) return 0;
			else if (vel > sign_dead_zone) return 0;
			else return 1;
		}

	public:
		ServoIdent(std::string const& name);
		bool startHook();
		void updateHook();
		void stopHook();

};

}
}
	


#endif
