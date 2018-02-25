#include "servo_ident.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace RTT;
using sweetie_bot::logger::Logger;
using namespace boost::numeric::ublas;

namespace sweetie_bot {

namespace motion {

servo_model_data::servo_model_data(): index(0), ident_started(false), alpha(4), P(4, 4) {
};

ServoIdent::ServoIdent(std::string const& name) :
	TaskContext(name),
	log(logger::categoryFromComponentName(name)), phi(4), L(4) {

	if (!log.ready()) {
		RTT::Logger::In in("ServoIdent");
		RTT::log(RTT::Error) << "Logger is not ready!" << endlog();
		this->fatal();
		return;
	}

	this->addEventPort("sync_step", in_sync_step)
		.doc("Timer event indicating beginig of next herck_sched cycle.");
	this->addPort("in_joints_accel_fixed", in_joints_fixed)
		.doc("Desired joints state. Order of joints should not change.");
	this->addPort("in_goals", in_goals)
		.doc("Servo goals.");
	this->addPort("in_joints_measured", in_joints_measured)
		.doc("Some measured joints state.");
	this->addEventPort("in_battery_state", in_battery_state)
		.doc("Port for updating current voltage of the battery.");

	this->addPort("out_servo_models", out_servo_models)
		.doc("Result of identification.");
	this->addPort("out_torques", out_torque_error_sorted)
		.doc("Prediction error. Desired torque - counted by model torque. Position and velocity are current ones");

	this->addOperation("startIdentification", &ServoIdent::startIdentification, this, OwnThread)
		.doc("It starts identification of servos.")
		.arg("servos", "Names of servos to be identified.");
	this->addOperation("endIdentification", &ServoIdent::endIdentification, this, OwnThread)
		.doc("It ends identification of servos. If accuracy treshhold is achieved, models will be written to servo_models port. Returns names of identified servos.");
	this->addOperation("abortIdentification", &ServoIdent::abortIdentification, this, OwnThread)
		.doc("It aborts identification of servos.");

	this->addProperty("period", period)
		.doc("Control cycle duration (seconds).")
		.set(0.0224);
	this->addProperty("control_delay", control_delay)
		.doc("Control cycle delay (number of cycles). You should set it before start.")
		.set(0);
	this->addProperty("relaxation_factor", relaxation_factor)
		.doc("Relaxation factor for OLQ.")
		.set(0.995);
	this->addProperty("error_averaging_time", error_averaging_time)
		.doc("Error averaging time(seconds). You should set it before start identification.")
		.set(1);
	this->addProperty("treshhold", treshhold)
		.doc("Minimum permissible accuraccy, rad.")
		.set(0.01);
	this->addProperty("sign_dead_zone", sign_dead_zone)
		.doc("Dead zone for detecting sign of velocity, rad/s, should be positive.")
		.set(0.01);
	this->addProperty("battery_voltage", battery_voltage)
		.doc("Current voltage of the battery.")
		.set(7);
	this->addProperty("default_servo_model", default_servo_model)
		.doc("Default servo model. Is used for error calculation if specifc model is not provided in servo_model list.");
	this->addProperty("servo_models", servo_models)
		.doc("Vector of models.");
	this->addProperty("ignore_accel", ignore_accel)
		.doc("Ignore acceleration during model identification. a[1] coefficient will always be equal to zero.")
		.set(false);

	//this coefficient provides zero torque_error_sorted for model
	default_servo_model.name = "default";
	default_servo_model.kp = 0;
	default_servo_model.kgear = 1;
	default_servo_model.alpha[0] = 0;
	default_servo_model.alpha[1] = 0;
	default_servo_model.alpha[2] = 0;
	default_servo_model.alpha[3] = 1;
	default_servo_model.qs = 10;
	default_servo_model.delta = 1;

}

struct ModelFinder {
	const std::string s;

	ModelFinder(const std::string &str) : s(str) {};
	bool operator()(const sweetie_bot_servo_model_msg::ServoModel &mdl) const {
		return mdl.name == s;
	}
};

double ServoIdent::sign_t(double vel) {

	if (vel < -sign_dead_zone)
		return -1;
	else if (vel > sign_dead_zone)
		return 1;
	else
		return 0;
}

bool ServoIdent::sort_servo_models() {
	std::vector<sweetie_bot_servo_model_msg::ServoModel> mdls;
	std::vector<sweetie_bot_servo_model_msg::ServoModel>::iterator s_iter;
	int i;

/mdls.assign(joints->name.size(), default_servo_model);

	for (i = 0; i < joints->name.size(); i++) {

		mdls[i].name = joints->name[i];
		s_iter = std::find_if(servo_models.begin(), servo_models.end(), ModelFinder(mdls[i].name));

		if (s_iter != servo_models.end())
			mdls[i] = *s_iter;
		servo_models_data[joints->name[i]].index = i;
	}

	servo_models = mdls;

	return true;
}

void ServoIdent::prepare_buffers_for_new_joints_size(sweetie_bot_kinematics_msgs::JointStateAccel const& jnt) {
	sweetie_bot_kinematics_msgs::JointStateAccel joint_accel;
	sweetie_bot_herkulex_msgs::ServoGoal goal;
	unsigned int i;
	unsigned int n;

	n = jnt.name.size();

	effort_joints.name = jnt.name;
	effort_joints.position.assign(n, 0);
	effort_joints.velocity.assign(n, 0);
	effort_joints.effort.assign(n, 0);

	joints_measured = effort_joints;

	joint_accel.name = effort_joints.name;
	joint_accel.position = effort_joints.position;
	joint_accel.velocity = effort_joints.velocity;
	joint_accel.acceleration.assign(n, 0);
	joint_accel.effort = effort_joints.effort;

	goal.name = jnt.name;
	goal.target_pos.assign(n, 0);
	goal.playtime.assign(n, 0);

	n = control_delay;
	joints_buf.reset(n, joint_accel);
	goals_buf.reset(n, goal);
	joints = &joints_buf.pos_to_get();
	goals = &goals_buf.pos_to_get();
}

bool ServoIdent::startHook() {
	sweetie_bot_kinematics_msgs::JointStateAccel joints_samp;
	unsigned int i;

	in_joints_fixed.getDataSample(joints_samp);

	prepare_buffers_for_new_joints_size(joints_samp);

	out_torque_error_sorted.setDataSample(effort_joints);

	models_vector_was_sorted = false;

	log(INFO) << "ServoIdent started!" << endlog();

	return true;
}

void ServoIdent::updateHook() {
	std::unordered_map<std::string, servo_model_data>::iterator iter;
	unsigned int njoints, njoints_prev;
	unsigned int j;
	unsigned int i;
	double den;
	double y;
	double target_pos;
	double prec_err;

	if (in_sync_step.read(timer_id, false) == NewData) {

		if (in_joints_fixed.read(joints_buf.pos_to_put(), true) == NewData) {

			njoints_prev = joints->name.size();
			joints = &joints_buf.pos_to_get();
			njoints = joints->name.size();
			if (njoints != joints->position.size() || njoints != joints->velocity.size() || njoints != joints->acceleration.size() || njoints != joints->effort.size()) {

				log(WARN) << "joints message has incorrect structure." << endlog();
				return;
			}

			if (!models_vector_was_sorted) {

				for (int n = 0; n < joints_buf.delay_value(); n++)
					joints_buf.pos_to_put();
				joints = &joints_buf.pos_to_get();
				models_vector_was_sorted = sort_servo_models();
				prepare_buffers_for_new_joints_size(*joints);
				log(INFO) << "Servo models was sorted. Now you should not change it's order!" << endlog();

			}

			if (njoints_prev != njoints) {

				log(WARN) << "Number of servos was changed. Skipping and resorting servo_models." << endlog();
				models_vector_was_sorted = false;
				return;
			}

		}

		if (in_goals.read(goals_buf.pos_to_put(), true) == NewData) {

			njoints = joints->name.size();
			goals = &goals_buf.pos_to_get();
			if (njoints != goals->name.size() || njoints != goals->target_pos.size() || njoints != goals->playtime.size()) {

				log(WARN) << "goals message has incorrect structure." << endlog();
				return;
			}
		}

		if (in_joints_measured.read(joints_measured, false) == NewData) {

			njoints = joints_measured.name.size();

			if (njoints != joints_measured.position.size() || njoints != joints_measured.velocity.size()) {

				log(WARN) << "joints_measured message has incorrect structure." << endlog();
				return;
			}
		} else {

			njoints = 0;
		}

		//here we process servos for which measurements was getting
		for(i = 0; i < njoints; i++) {

			iter = servo_models_data.find(joints_measured.name[i]);
			if (iter == servo_models_data.end()) {

				log(INFO) << "joints_measured message contains extra joint " << joints_measured.name[i] << ". Skipped." << endlog();
				continue;
			}

			j = iter->second.index;

			effort_joints.position[j] = joints_measured.position[i];
			effort_joints.velocity[j] = joints_measured.velocity[i];

			effort_joints.effort[j] = (
				(goals->target_pos[j] - joints->position[j]) * servo_models[j].kp*battery_voltage 
				- servo_models[j].alpha[0] * joints_measured.velocity[i]
				- servo_models[j].alpha[1] * sign_t(joints_measured.velocity[i])
				- servo_models[j].alpha[2] * sign_t(joints_measured.velocity[i]) * 
					exp(-pow(fabs(joints_measured.velocity[i]/servo_models[j].qs), servo_models[j].delta))
				) / servo_models[j].alpha[3];
			effort_joints.effort[j] = -(effort_joints.effort[j] - joints->effort[j]);

			//now do identification, if it is necessary
			if (iter->second.ident_started) {
				phi(0) = joints_measured.velocity[i];
				phi(1) = sign_t(joints_measured.velocity[i]);
				phi(2) = sign_t(joints_measured.velocity[i]) * exp(-pow(fabs(joints_measured.velocity[i]/servo_models[j].qs), servo_models[j].delta));
				phi(3) = joints->effort[j];

				y = (goals->target_pos[j] - joints_measured.position[i]) * battery_voltage * servo_models[j].kp;

				den = relaxation_factor + inner_prod(prod(phi, iter->second.P), phi);
				prec_err = y - inner_prod(phi, iter->second.alpha);
				L = prod(iter->second.P, phi) / den;
				iter->second.alpha += L * prec_err;
				iter->second.P = (iter->second.P - prod(outer_prod(L, phi), iter->second.P)) / relaxation_factor;
				iter->second.prec_err_buf.pos_to_put() = prec_err / battery_voltage / servo_models[j].kp;
			}

		}

		out_torque_error_sorted.write(effort_joints);
	}

	//update battery voltage rate
	if (in_battery_state.read(battery_voltage_buf, false) == NewData)
		battery_voltage = battery_voltage_buf.voltage;
}

void ServoIdent::stopHook() {

	log(INFO) << "Executes stopping !" <<endlog();
}

bool ServoIdent::startIdentification(std::vector<std::string> names) {
	std::unordered_map<std::string, servo_model_data>::iterator iter;
	unsigned int i;
	unsigned int j;

	for (i = 0; i < names.size(); i++) {

		iter = servo_models_data.find(names[i]);

		if (iter == servo_models_data.end()) {

			log(WARN) << "Attempt to identify unknown servo " << names[i] << ". Skipped" << endlog();
		} else {

			j = iter->second.index;
			iter->second.ident_started = true;
			iter->second.alpha(0) = servo_models[j].alpha[0];
			iter->second.alpha(1) = servo_models[j].alpha[1];
			iter->second.alpha(2) = servo_models[j].alpha[2];
			iter->second.alpha(3) = servo_models[j].alpha[3];
			iter->second.P = identity_matrix<double>(4,4);
			iter->second.prec_err_buf.reset(error_averaging_time / period);
		}
	}

	log(INFO) << "Identification started!" << endlog();

	return true;
}

std::vector<std::string> ServoIdent::endIdentification() {
	std::unordered_map<std::string, servo_model_data>::iterator iter;
	std::vector<std::string> names;
	std::vector<sweetie_bot_servo_model_msg::ServoModel> mdls;
	unsigned int j, n;
	double prec;

	for (iter = servo_models_data.begin(); iter != servo_models_data.end(); iter++) {

		if (iter->second.ident_started) {

			for (prec = 0, n = 0; n <= iter->second.prec_err_buf.delay_value(); n++)
				prec += abs(iter->second.prec_err_buf[n]);
			prec = prec / (iter->second.prec_err_buf.delay_value() + 1);
			j = iter->second.index;
			if (prec <= treshhold) {

				names.push_back(servo_models[j].name);
				iter->second.ident_started = false;
				servo_models[j].alpha[0] = iter->second.alpha(0);
				servo_models[j].alpha[1] = iter->second.alpha(1);
				servo_models[j].alpha[2] = iter->second.alpha(2);
				servo_models[j].alpha[3] = iter->second.alpha(3);
				mdls.push_back(servo_models[j]);
				log(WARN) << "Identification success for " << servo_models[j].name << "!" << "Prec: " << prec << endlog();
			} else {
				log(WARN) << "Identification faild for " << servo_models[j].name << "!" << "Prec: " << prec << endlog();
			}
		}
	}

	out_servo_models.write(mdls);

	log(INFO) << "Identification ended!" << endlog();

	return names;
}

bool ServoIdent::abortIdentification() {
	std::unordered_map<std::string, servo_model_data>::iterator iter;

	for (iter = servo_models_data.begin(); iter != servo_models_data.end(); iter++)
		if (iter->second.ident_started)
			iter->second.ident_started = false;

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
