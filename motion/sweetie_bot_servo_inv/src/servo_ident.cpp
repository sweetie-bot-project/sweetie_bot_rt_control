#include "servo_ident.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace RTT;
using sweetie_bot::logger::Logger;
using namespace boost::numeric::ublas;

namespace sweetie_bot {

namespace motion {

servo_model_data::servo_model_data(): index(0), ident_started(false), alpha(5), P(5, 5) {
};

ServoIdent::ServoIdent(std::string const& name) :
	TaskContext(name),
	log(logger::categoryFromComponentName(name)), phi(5), L(5) {

	if (!log.ready()) {
		RTT::Logger::In in("ServoIdent");
		RTT::log(RTT::Error) << "Logger is not ready!" << endlog();
		this->fatal();
		return;
	}

	this->addPort("joints_fixed", in_joints_fixed)
                .doc("Desired joints state. Order of joints should not change.");
        this->addEventPort("joints_measured", in_joints_measured)
                .doc("Some measured joints state.");
        this->addEventPort("battery_state", in_battery_state)
		.doc("Port for updating current voltage of the battery.");

        this->addPort("servo_models", out_servo_models)
                .doc("Result of identification.");
        this->addPort("torque_error_sorted", out_torque_error_sorted)
                .doc("Prediction error. Desired tourque - counted by model tourque. Position and velocity are current ones");

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
                .doc("Minimum permissible accuraccy, rad")
                .set(0.01);
	this->addProperty("battery_voltage", battery_voltage)
                .doc("Current voltage of the battery.")
                .set(7);
        this->addProperty("servo_models", servo_models)
                .doc("Vector of models.");
}

struct ModelFinder {
        const std::string s;

        ModelFinder(const std::string &str) : s(str) {};
        bool operator()(const sweetie_bot_servo_model_msg::ServoModel &mdl) const {
                return mdl.name == s;
        }
};

bool ServoIdent::sort_servo_models() {
        sweetie_bot_servo_model_msg::ServoModel eq_mdl;
        std::vector<sweetie_bot_servo_model_msg::ServoModel> mdls;
        std::vector<sweetie_bot_servo_model_msg::ServoModel>::iterator s_iter;
        int i;

	//this coefficient provides zero torque_error_sorted for model
        eq_mdl.name = "";
        eq_mdl.kp = 0;
        eq_mdl.kgear = 1;
        eq_mdl.alpha[0] = 0;
        eq_mdl.alpha[1] = 0;
        eq_mdl.alpha[2] = 0;
        eq_mdl.alpha[3] = 0;
        eq_mdl.alpha[4] = 1;
        eq_mdl.qs = 10;
        eq_mdl.delta = 1;

        mdls.assign(joints->name.size(), eq_mdl);

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

void ServoIdent::prepare_buffers_for_new_joints_size(sensor_msgs::JointState const& jnt) {
	unsigned int i;
	unsigned int n;

	n = jnt.name.size();

	effort_joints.name = jnt.name;
	effort_joints.position.assign(n, 0);
	effort_joints.velocity.assign(n, 0);
	effort_joints.effort.assign(n, 0);

        joints_measured = effort_joints;

        velocity_prev.resize(n);
        velocity_measured_prev.resize(n);
	num_periods.assign(n, 1);

	n = control_delay;
	joints_buf.reset(n);
	for (i = 0; i <= n; i++)
		joints_buf.pos_to_put() = effort_joints;
	joints = &joints_buf.pos_to_get();
}

bool ServoIdent::startHook() {
	sensor_msgs::JointState joints_samp;
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
	unsigned int njoints;
	unsigned int j;
	unsigned int i;
	double den;
	double y;
	double target_pos;
	double prec_err;

	if (in_joints_measured.read(joints_measured, false) == NewData) {

		if (in_joints_fixed.read(joints_buf.pos_to_put(), false) == NewData) {

			joints = &joints_buf.pos_to_get();
			njoints = joints->name.size();

			if (njoints != joints->position.size() || njoints != joints->velocity.size() || njoints != joints->effort.size()) {

				log(WARN) << "joints message has incorrect structure." << endlog();
				return;
			}

			if (!models_vector_was_sorted) {

				joints_buf.init();
				models_vector_was_sorted = sort_servo_models();
				prepare_buffers_for_new_joints_size(*joints);
				log(INFO) << "Servo models was sorted. Now you should not change it's order!" << endlog();

			}

			if (velocity_prev.size() != njoints) {

				log(WARN) << "Number of servos was changed. Skipping and resorting servo_models." << endlog();
				models_vector_was_sorted = false;
				return;
			}

		}

		njoints = joints_measured.name.size();

		if (njoints != joints_measured.position.size() || njoints != joints_measured.velocity.size()) {

			log(WARN) << "joints_measured message has incorrect structure." << endlog();
			return;
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
				servo_models[j].kp*battery_voltage * (
					-(joints->position[j] - joints_measured.position[i])
				)
				- servo_models[j].alpha[0] * (
					joints->velocity[j] - joints_measured.velocity[i]
				)
				- servo_models[j].alpha[1] * (
					(joints->velocity[j] - velocity_prev[j])
					- (joints_measured.velocity[i] - velocity_measured_prev[j])
				) / (period * num_periods[j])
				-  servo_models[j].alpha[2] * (
					copysign(1, joints->velocity[j])
					- copysign(1, joints_measured.velocity[i])
				)
				- servo_models[j].alpha[3] * (
					copysign(exp(-pow(fabs(joints->velocity[j]/servo_models[j].qs), servo_models[j].delta)),
						joints->velocity[j])
					- copysign(exp(-pow(fabs(joints_measured.velocity[i]/servo_models[j].qs), servo_models[j].delta)),
						joints_measured.velocity[i])
				)
			) / servo_models[j].alpha[4];

			//now do identification, if it is necessary
			if (iter->second.ident_started) {
				target_pos = (
				    servo_models[j].alpha[0] * joints->velocity[j] +
				    servo_models[j].alpha[1] * (joints->velocity[j]-velocity_prev[j])/period +
				    servo_models[j].alpha[2] * copysign(1, joints->velocity[j]) +
				    servo_models[j].alpha[3] * copysign(exp(-pow(fabs(joints->velocity[j]/servo_models[j].qs),
													servo_models[j].delta)), joints->velocity[j]) +
				    servo_models[j].alpha[4] * joints->effort[j]
				  ) / (servo_models[j].kp*battery_voltage) +
				  joints->position[j];

				phi(0) = joints_measured.velocity[i];
				phi(1) = (joints_measured.velocity[i] - velocity_measured_prev[j])
				/ (period * num_periods[j]);
				phi(2) = copysign(1, joints_measured.velocity[i]);
				phi(3) = copysign(exp(-pow(fabs(joints_measured.velocity[i]/servo_models[j].qs), servo_models[j].delta)),
						joints_measured.velocity[i]);
				phi(4) = joints->effort[j];

				y = (target_pos - joints_measured.position[i]) * battery_voltage * servo_models[j].kp;

				den = relaxation_factor + inner_prod(prod(phi, iter->second.P), phi);
				prec_err = y - inner_prod(phi, iter->second.alpha);
				L = prod(iter->second.P, phi) / den;
				iter->second.alpha += L * prec_err;
				iter->second.P = (iter->second.P - prod(outer_prod(L, phi), iter->second.P)) / relaxation_factor;
				iter->second.prec_err_buf.pos_to_put() = prec_err / battery_voltage / servo_models[j].kp;
			}

			velocity_measured_prev[j] = joints_measured.velocity[i];
			num_periods[j] = 0;

		}

		out_torque_error_sorted.write(effort_joints);

		//another control cycle is ending...
		njoints = joints->name.size();
		for (j = 0; j < njoints; j++)
			num_periods[j]++;

		velocity_prev = joints->velocity;

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

			log(WARN) << "Attempt to identify unknown servo" << names[i] << ". Skipped";
		} else {

			j = iter->second.index;
			iter->second.ident_started = true;
			iter->second.alpha(0) = servo_models[j].alpha[0];
			iter->second.alpha(1) = servo_models[j].alpha[1];
			iter->second.alpha(2) = servo_models[j].alpha[2];
			iter->second.alpha(3) = servo_models[j].alpha[3];
			iter->second.alpha(4) = servo_models[j].alpha[4];
			iter->second.P = identity_matrix<double>(5,5);
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
	unsigned int j;

	for (iter = servo_models_data.begin(); iter != servo_models_data.end(); iter++) {

		if (iter->second.ident_started) {

			if (iter->second.prec_err_buf.average_elem() <= treshhold) {

				j = iter->second.index;
				names.push_back(servo_models[j].name);
				iter->second.ident_started = false;
				servo_models[j].alpha[0] = iter->second.alpha(0);
				servo_models[j].alpha[1] = iter->second.alpha(1);
				servo_models[j].alpha[2] = iter->second.alpha(2);
				servo_models[j].alpha[3] = iter->second.alpha(3);
				servo_models[j].alpha[4] = iter->second.alpha(4);
				mdls.push_back(servo_models[j]);
				log(INFO) << "Identification success for " << servo_models[j].name << "!" << endlog();
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
