#include "servo_ident.hpp"

#include <rtt/Component.hpp>
#include <iostream>

#include <sweetie_bot_orocos_misc/message_checks.hpp>
#include <sweetie_bot_orocos_misc/joint_state_check.hpp>

using namespace RTT;
using sweetie_bot::logger::Logger;

namespace sweetie_bot {

namespace motion {

const size_t ServoIdent::max_cycle_delay;

ServoIdent::ServoIdent(std::string const& name) :
	TaskContext(name),
	log(logger::categoryFromComponentName(name)) 
{

	if (!log.ready()) {
		RTT::Logger::In in("ServoIdent");
		RTT::log(RTT::Error) << "Logger is not ready!" << endlog();
		this->fatal();
		return;
	}

	// PORTS
	this->addEventPort("sync_step", in_sync_step)
		.doc("Timer event indicating beginig of next cotrol cycle.");
	this->addPort("in_joints_accel_ref_fixed", in_joints_ref_fixed)
		.doc("Desired joints state. Order of joints should not change.");
	this->addPort("in_goals_fixed", in_goals_fixed)
		.doc("Servo goals. Order of joints should be same as in in_joints_ref_fixed port.");
	this->addEventPort("in_joints_measured", in_joints_measured)
		.doc("Joints state measurements from servos.");
	this->addPort("in_battery_state", in_battery_state)
		.doc("Port for updating current voltage of the battery.");

	this->addPort("out_servo_models", out_servo_models)
		.doc("Result of identification.");
	this->addPort("out_torque_error_fixed", out_torque_error_fixed)
		.doc("Efforts are calculated by servo model. Position, velocity, acceleration are current ones");

	// OPERATIONS
	this->addOperation("startIdentification", &ServoIdent::startIdentification, this, OwnThread)
		.doc("Start servos identification procedure.")
		.arg("servos", "Names of servos to be identified.");
	this->addOperation("endIdentification", &ServoIdent::endIdentification, this, OwnThread)
		.doc("End identification procedure. If identification error is less then accuracy treshold, corresponding models will be written to servo_models port. Returns names of identified servos.");
	this->addOperation("abortIdentification", &ServoIdent::abortIdentification, this, OwnThread)
		.doc("Aborts identification procedure.");

	// PROPERTIES
	this->addProperty("period", period)
		.doc("Control cycle duration (seconds).")
		.set(0.056);
	this->addProperty("control_delay", control_delay)
		.doc("Servo transport delay (seconds): time beetween sync event (set goal command for servos) and moment when the servo start execute command.")
		.set(0.0224);
	this->addProperty("relaxation_factor", relaxation_factor)
		.doc("Relaxation factor for OLQ.")
		.set(0.995);
	this->addProperty("error_averaging_time", error_averaging_time)
		.doc("Error averaging time (seconds). You should set it before start identification.")
		.set(5);
	this->addProperty("treshhold", treshhold)
		.doc("Minimum permissible accuraccy, rad.")
		.set(0.01);
	this->addProperty("sign_dead_zone", sign_dead_zone)
		.doc("Dead zone for detecting sign of velocity, rad/s, should be positive.")
		.set(0.001);
	this->addProperty("battery_voltage", battery_voltage)
		.doc("Current voltage of the battery.")
		.set(7);
	this->addProperty("default_servo_model", default_servo_model)
		.doc("Default servo model. Is used for error calculation if specifc model is not provided in servo_model list.");
	this->addProperty("servo_models", servo_models)
		.doc("Vector of servo models.");
	/*this->addProperty("ignore_accel", ignore_accel)
		.doc("Ignore acceleration during model identification. a[1] coefficient will always be equal to zero.")
		.set(false);*/

	//this coefficient provides zero effort error  for model
	default_servo_model.name = "default";
	default_servo_model.kp = 0;
	default_servo_model.kgear = 1;
	default_servo_model.alpha[0] = 0;
	default_servo_model.alpha[1] = 0;
	default_servo_model.alpha[2] = 0;
	default_servo_model.alpha[3] = 1;
}

struct ModelFinder {
	const std::string s;

	ModelFinder(const std::string &str) : s(str) {};
	bool operator()(const sweetie_bot_servo_model_msg::ServoModel &mdl) const {
		return mdl.name == s;
	}
};


void ServoIdent::setupServoData(const std::vector<std::string>& joint_names) 
{
	// prepare new model index
	servos.clear();
	servos.reserve(joint_names.size());
	servos_index.clear();
	// add joints to index
	for (const std::string name : joint_names) {
		// add ServoData object and store it index
		servos_index[name] = servos.size(); 
		servos.emplace_back();
		ServoData& servo = servos.back();
		// try to find coresponding ServoModel
		auto iter = std::find_if(servo_models.begin(), servo_models.end(), [name](const ServoModel& model) -> bool { return model.name == name;} );
		if (iter != servo_models.end()) {
			// check if model name is unique
			if (std::find_if(iter + 1, servo_models.end(), [name](const ServoModel& model) -> bool { return model.name == name;}) != servo_models.end()) {
				log(WARN) << "Incorrect servo_models property: name '" << name << "' is not unique." << endlog();
			}
			// assign parameters from property
			servo.model = *iter;
		}
		else {
			// use default model
			servo.model = default_servo_model;
		}
	}
	// resize port buffers	
	int n_joints = joint_names.size();

	joints_effort.name = joint_names;
	joints_effort.position.assign(n_joints, 0.0);
	joints_effort.velocity.assign(n_joints, 0.0);
	joints_effort.effort.assign(n_joints, 0.0);

	log(DEBUG) << "ServoIdent: successfully setupServoData for " << n_joints << " servos." << endlog();
}

bool ServoIdent::startHook() {
	// get data samples
	in_joints_measured.getDataSample(joints_measured);
	// reconfigure component
	if (isValidJointStateAccelNamePosVelAccelEffort(joints_ref)) {
		setupServoData(joints_ref.name);
	}
	// get data samples
	in_joints_ref_fixed.getDataSample(joints_ref);
	in_goals_fixed.getDataSample(goals); 

	// set data samples
	out_torque_error_fixed.setDataSample(joints_effort);

	// reset component state
	history_update_counter = 0;

	log(INFO) << "ServoIdent started!" << endlog();
	return true;
}

void ServoIdent::updateHook() {
	unsigned int njoints = servos.size();
	RTT::os::Timer::TimerId timer_id_unused;

	// check if new position measurements are available
	if (in_joints_measured.read(joints_measured, false) == NewData ) {

		// check message
		if (!isValidJointStateNamePosVel(joints_measured)) {
			// incorrect message. Do nothing.
			log(WARN) << "Message on in_joints_measured port has incorrect structure." << endlog();
			return;
		}

		// check if history is good and have enought measurements
		bool history_good = history_update_counter >= max_cycle_delay;

		// calculate time shifts
		// TODO adjustments via properties
		const int diff_cycles = 2;
		TimeDiscrete<max_cycle_delay> history_cycle_diff = history_cycle_now - diff_cycles; // is used for positon derivative calculation
		// reference signals must be delayed: control_delay (passing mesages to servo) + period (playtime is set to period) 
		// (assume that read happens immediately after servo applies goal)
		// TODO Use HerkulexJointState with explicit values of read delays
		TimeContinous<max_cycle_delay> history_cycle_ref = TimeContinous<max_cycle_delay>(history_cycle_now) - (control_delay + period)/period;

		// here we process servos
		for(int i = 0; i < joints_measured.name.size(); i++) {
			auto iter = servos_index.find(joints_measured.name[i]);
			if (iter == servos_index.end()) {
				log(DEBUG) << "joints_measured message contains extra joint " << joints_measured.name[i] << ". Skipped." << endlog();
				continue;
			}
			// get servo index
			int j = iter->second;
			ServoData& servo = servos[j];

			// store measurements
			servos[j].pos_measured.get(history_cycle_now) = joints_measured.position[i];
			servos[j].vel_measured.get(history_cycle_now) = joints_measured.velocity[i];
			joints_effort.position[j] = joints_measured.position[i];

			if (!history_good) {
				joints_effort.velocity[j] = joints_measured.velocity[i];
				joints_effort.effort[j] = 0.0;
			}
			else {
				// apply servo model
				double velocity = (servo.pos_measured.get(history_cycle_now) - servo.pos_measured.get(history_cycle_diff)) / (diff_cycles*period);
				double velocity_sign = sign(velocity);
				double y = (servo.goal.get(history_cycle_ref - (servo.playtime - period)) - servo.pos_measured.get(history_cycle_now)) * servo.model.kp * battery_voltage;

				double effort = (
					 y
					- servo.model.alpha[0] * servo.accel_ref.get(history_cycle_ref)
					- servo.model.alpha[1] * velocity
					- servo.model.alpha[2] * velocity_sign
					) / servo.model.alpha[3];

				// fill output message
				joints_effort.velocity[j] = velocity;
				if (!std::isnan(effort)) joints_effort.effort[j] = effort - servo.effort_ref.get(history_cycle_ref);
				else joints_effort.effort[j] = 0.0;

				//now do identification, if it is necessary
				if (servo.ident_started && (std::abs(velocity) > 0.1)){
					//FIXME: velocity threshold and accelerations
					Eigen::Vector4d phi;
					Eigen::Vector4d L;
					Eigen::Map<Eigen::Vector4d> alpha(&servo.model.alpha[0]);

					phi(0) = servo.accel_ref.get(history_cycle_ref);
					phi(1) = velocity;
					phi(2) = velocity_sign;
					phi(3) = servo.effort_ref.get(history_cycle_ref);

					double den = relaxation_factor + phi.dot(servo.P * phi);
					L = (servo.P * phi) / den;
					double pred_error = y - phi.dot(alpha);
					alpha += L * pred_error;
					servo.P = (servo.P - L * (phi.transpose() * servo.P)) / relaxation_factor;

					// error statistics
					pred_error /= battery_voltage * servo.model.kp;
					servo.pred_error_sq_avg = pred_error_avg_alpha * servo.pred_error_sq_avg + (1.0 - pred_error_avg_alpha) * pred_error*pred_error;
				}

			}

		}
		// publish message
		out_torque_error_fixed.write(joints_effort);
	}


	if (in_sync_step.read(timer_id_unused) == NewData) {
		// new control cycle is triggered: we must shift time forward 

		// new update
		history_update_counter++;	
		// shift data history discrete time, which is used to address measurement history
		history_cycle_now += 1;

		// read joint_ref and goals ports
		in_joints_ref_fixed.read(joints_ref, false);
		in_goals_fixed.read(goals, false);

		// check joints_ref and store its content
		if (isValidJointStateAccelNamePosVelAccelEffort(joints_ref), njoints) {
			// we need only accel and efforts
			for(int i = 0; i < servos.size(); i++) {
				servos[i].accel_ref.get(history_cycle_now) = joints_ref.acceleration[i];
				servos[i].effort_ref.get(history_cycle_now) = joints_ref.effort[i];
			}
		}
		else {
			if (isValidJointStateAccelNamePosVelAccelEffort(joints_ref)) {
				// number of joints has changed. Force reconfigure.
				log(WARN) << "Number of servos was changed. Will skip " << max_cycle_delay << " cycles until message history is good again." << endlog();
				setupServoData(joints_ref.name);
			}
			else {
				// corrupted message
				log(WARN) << "Message on in_joints_ref port has incorrect structure. Will skip " << max_cycle_delay << " cycles until message history is good again." << endlog();
			}
			// history is corrupted
			// force component to wait until history is good again
			history_update_counter = 0;
			return;
		}

		// check goals and store its content
		if (njoints == goals.name.size() && njoints == goals.target_pos.size() && njoints == goals.playtime.size()) {
			// we need only traget_pos and playtime
			for(int i = 0; i < servos.size(); i++) {
				servos[i].goal.get(history_cycle_now) = goals.target_pos[i];
				servos[i].playtime = goals.playtime[i];
			}
		}
		else {
			// corrupted message o
			log(WARN) << "Message on in_goals port has incorrect structure or size. Will skip " << max_cycle_delay << " cycles until message history is good again." << endlog();
			// history is corrupted
			// force component to wait until history is good again
			history_update_counter = 0;
			return;
		}

		// fill the rest ServoData history fields with nan
		// TODO maybe previous values is better?
		for(int i = 0; i < servos.size(); i++) {
			servos[i].vel_measured.get(history_cycle_now) = NAN;
			servos[i].pos_measured.get(history_cycle_now) = NAN;
		}
	}

	// update battery voltage 
	if (in_battery_state.read(battery_state, false) == NewData)
		battery_voltage = battery_state.voltage;
}

void ServoIdent::stopHook() {

	log(INFO) << "Executes stopping !" <<endlog();
}

bool ServoIdent::startIdentification(std::vector<std::string> names) {
	// calcualte error averaging coefficient
	if (error_averaging_time < 50*period) {
		log(WARN) << "error_averaging_time property is too small. Set to default value." << endlog();
		error_averaging_time = 200*period;
	}
	pred_error_avg_alpha = 1.0 - 3*period/error_averaging_time;

	// now start identification
	for (const std::string& name : names) {
		auto iter = servos_index.find(name);

		if (iter == servos_index.end()) {
			log(WARN) << "Unknown servo " << name << " will be skipped." << endlog();
		} 
		else {
			ServoData& servo = servos[iter->second];
			// init identification constants
			servo.ident_started = true;
			servo.pred_error_sq_avg = 0.0;
			servo.P.setZero();
		}
	}

	log(INFO) << "Identification started!" << endlog();

	return true;
}

std::vector<std::string> ServoIdent::endIdentification() {
	std::vector<std::string> names;
	std::vector<sweetie_bot_servo_model_msg::ServoModel> servo_models_msg;


	log(INFO) << "Identification results: ";
	for (ServoData& servo : servos) {
		if (servo.ident_started) {
			// get averaged prediction error
			double pred_error = std::sqrt(servo.pred_error_sq_avg);
			if (pred_error <= treshhold) {
				// identification succesed
				
				// add model to message
				servo_models_msg.push_back(servo.model);
				names.push_back(servo.model.name);

				// update model property
				auto iter = std::find_if(servo_models.begin(), servo_models.end(), ModelFinder(servo.model.name));
				if (iter != servo_models.end()) {
					*iter = servo.model;
				}
				else {
					// add model to list
					servo_models.push_back(servo.model);
				}

				log() << servo.model.name << " OK (" << pred_error << "), ";
			}
			else {
				log() << servo.model.name << " FAILED (" << pred_error << "), ";
			}
		}
	}
	log() << endlog();

	// publish results
	out_servo_models.write(servo_models_msg);
	return names;
}

bool ServoIdent::abortIdentification() {
	for (ServoData& servo : servos) {
		if (servo.ident_started) {
			// stop identification
			servo.ident_started = false;
			// restore default models
			auto iter = std::find_if( servo_models.begin(), servo_models.end(), ModelFinder(servo.model.name) );
			if (iter != servo_models.end()) {
				servo.model = *iter;
			}
			else {
				servo.model = default_servo_model;
			}
		}
	}
	log(INFO) << "Identification aborted!" << endlog();
	return true;
}

} // nmaespace motion
} // namespace sweetie_bot


/*
* Using this macro, only one component may live
* in one library *and* you may *not* link this library
* with another component library. Use
* ORO_CREATE_COMPONENT_TYPE()
* ORO_LIST_COMPONENT_TYPE(ServoIdent)
* In case you want to link with another library that
* already contains components.
*
* If you have put your component class
* in a namespace, don't forget to add it here too:
*/
ORO_CREATE_COMPONENT(sweetie_bot::motion::ServoIdent)
