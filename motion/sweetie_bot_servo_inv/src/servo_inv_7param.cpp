#include <rtt/Component.hpp>
#include <iostream>
#include <algorithm>

#include "servo_inv_7param.hpp"

using namespace RTT;
using sweetie_bot::logger::Logger;

namespace sweetie_bot {

namespace motion {


ServoInv7Param::ServoInv7Param(std::string const& name) :
	TaskContext(name),
	log(logger::categoryFromComponentName(name)) {

	if (!log.ready()) {
		RTT::Logger::In in("ServoInv7Param");
		RTT::log(RTT::Error) << "Logger is not ready!" << endlog();
		this->fatal();
		return;
	}

	this->addEventPort("in_joints_accel_fixed", in_joints_fixed)
		.doc("Desired joints state with acceleration. Order of joints should not change.");
	this->addEventPort("in_servo_models", in_servo_models)
		.doc("Port for updating list of servo's models.");
	this->addEventPort("in_battery_state", in_battery_state)
		.doc("Port for updating current voltage of the battery.");
	this->addPort("out_goals", out_goals)
		.doc("Position controlled servos goals.");

	this->addProperty("period", period)
		.doc("Control cycle duration (seconds).")
		.set(0.0224);
	this->addProperty("default_servo_model", default_servo_model)
		.doc("If servo does not present in servo_model list default model is used.");
	this->addProperty("servo_models", servo_models)
		.doc("Vector of the models of the servos. It is automatically sorted in the order corresponding to the first message on joint_fixed port. You should not change it after this moment!");
	this->addProperty("battery_voltage", battery_voltage)
		.doc("Current voltage of the battery. Updating manually or from battery_state port.")
		.set(7);

	// set default model to equal
	default_servo_model.name = "default";
	default_servo_model.kp = 1;
	default_servo_model.kgear = 1;
	default_servo_model.alpha[0] = 0;
	default_servo_model.alpha[1] = 0;
	default_servo_model.alpha[2] = 0;
	default_servo_model.alpha[3] = 0;
	default_servo_model.alpha[4] = 0;
	default_servo_model.qs = 1;
	default_servo_model.delta = 1;
}

struct ModelFinder {
	const std::string s;

	ModelFinder(const std::string &str) : s(str) {};
	bool operator()(const sweetie_bot_servo_model_msg::ServoModel &mdl) const {
		return mdl.name == s;
	}
};

bool ServoInv7Param::sort_servo_models() {
	std::vector<sweetie_bot_servo_model_msg::ServoModel> mdls;
	std::vector<sweetie_bot_servo_model_msg::ServoModel>::iterator s_iter;
	int i;

	for (i = 0; i < joints.name.size(); i++) {
		s_iter = std::find_if(servo_models.begin(), servo_models.end(), ModelFinder(joints.name[i]));
		if (s_iter != servo_models.end()) mdls.push_back(*s_iter);
		else {
			mdls.push_back(default_servo_model);
			mdls.back().name = joints.name[i];
		}
	}

	servo_models = mdls;

	return true;
}

void ServoInv7Param::prepare_buffers_for_new_joints_size() {
	unsigned int n;

	n = joints.name.size();

	goals.name = joints.name;
	goals.target_pos.assign(n, 0);
	goals.playtime.assign(n, 0);
}

bool ServoInv7Param::startHook() {
	in_joints_fixed.getDataSample(joints);

	prepare_buffers_for_new_joints_size();

	out_goals.setDataSample(goals);

	models_vector_was_sorted = false;

	log(INFO) << "Started!" << endlog();

	return true;
}

void ServoInv7Param::updateHook() {
	unsigned int i;
	unsigned int njoints;
	std::vector<sweetie_bot_servo_model_msg::ServoModel>::iterator s_iter;

	if (in_joints_fixed.read(joints, false) == NewData) {

		njoints = joints.name.size();

		if (!models_vector_was_sorted) {

			models_vector_was_sorted = sort_servo_models();
			log(INFO) << "Servo models was sorted. Now you should not change it's order!" << endlog();
			prepare_buffers_for_new_joints_size();
		}

		if (njoints != joints.position.size() || njoints != joints.velocity.size()
							|| njoints != joints.acceleration.size()
							|| njoints != joints.effort.size()) {

			log(WARN) << "Joints message has incorrect structure." << endlog();
			return;
		}

		if (goals.name.size() != njoints) {

			log(WARN) << "Number of servos was changed. Skipping and resorting servo_models." << endlog();
			models_vector_was_sorted = false;
			return;
		}

		for(i = 0; i < njoints; i++) {

			//calculate target position using inverse model of the servo
			goals.target_pos[i] = (
			    servo_models[i].alpha[0] * joints.velocity[i] +
			    servo_models[i].alpha[1] * joints.acceleration[i] +
			    servo_models[i].alpha[2] * copysign(1, joints.velocity[i]) +
			    servo_models[i].alpha[3] * copysign(exp(-pow(fabs(joints.velocity[i]/servo_models[i].qs),
												servo_models[i].delta)), joints.velocity[i]) +
			    servo_models[i].alpha[4] * joints.effort[i]
			  ) / (servo_models[i].kp*battery_voltage) +
			  joints.position[i];

			goals.target_pos[i] *= servo_models[i].kgear;
		}

		out_goals.write(goals);
	}

	if (in_servo_models.read(models, false) == NewData) {

		//update servo models with new data
		for (i = 0; i < models.size(); i++) {
			s_iter = std::find_if(servo_models.begin(), servo_models.end(), ModelFinder(models[i].name));
			if (s_iter != servo_models.end())
				*s_iter = models[i];
		}
	}

	//update battery voltage rate
	if (in_battery_state.read(battery_voltage_buf, false) == NewData)
		battery_voltage = battery_voltage_buf.voltage;
}

void ServoInv7Param::stopHook() {

	log(INFO) << "Stopped!" << endlog();
}

} // nmaespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ServoInv7Param)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::ServoInv7Param)
