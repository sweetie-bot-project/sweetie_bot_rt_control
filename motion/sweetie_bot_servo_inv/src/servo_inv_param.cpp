#include "servo_inv_param.hpp"

#include <rtt/Component.hpp>
#include <algorithm>

#include <sweetie_bot_orocos_misc/message_checks.hpp>

using namespace RTT;
using sweetie_bot::logger::Logger;

namespace sweetie_bot {
namespace motion {


ServoInvParam::ServoInvParam(std::string const& name) :
	TaskContext(name),
	log(logger::categoryFromComponentName(name)) {

	if (!log.ready()) {
		RTT::Logger::In in("ServoInvParam");
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
	this->addProperty("sign_dead_zone", sign_dead_zone)
		.doc("Dead zone for detecting sign of velocity, rad/s, should be positive.")
		.set(0.001);

	// set default model to equal
	default_servo_model.name = "";
	default_servo_model.kp = 1.0;
	default_servo_model.kgear = 1.0;
	default_servo_model.alpha[0] = 0.0;
	default_servo_model.alpha[1] = 0.0;
	default_servo_model.alpha[2] = 0.0;
	default_servo_model.alpha[3] = 0.0;
}

struct ModelFinder {
       const std::string s;

       ModelFinder(const std::string &str) : s(str) {};
       bool operator()(const sweetie_bot_servo_model_msg::ServoModel &mdl) const {
               return mdl.name == s;
       }
};

void ServoInvParam::setupServoModelIndex(const std::vector<std::string>& joint_names) {
	// prepare new model index
	servo_model_index.clear();
	servo_model_index.reserve(joint_names.size());
	// add joints to index
	for (const std::string name : joints.name) {
		auto iter = std::find_if(servo_models.begin(), servo_models.end(), [name](const ServoModel& model) -> bool { return model.name == name;} );
		if (iter != servo_models.end()) {
			// add model to index
			servo_model_index.push_back(iter - servo_models.begin());
			// check if model name is unique
			if (std::find_if(iter, servo_models.end(), [name](const ServoModel& model) -> bool { return model.name == name;}) != servo_models.end()) {
				log(WARN) << "Incorrect servo_models property: name '" << name << "' is not unique." << endlog();
			}
		}
		else {
			// use default model
			servo_model_index.push_back(-1); 
		}
	}
	// resize port buffers	
	goals.name = joint_names;
	goals.target_pos.assign(joint_names.size(), 0);
	goals.playtime.assign(joint_names.size(), period);
}

bool ServoInvParam::startHook() 
{
	// clear old servo_model_index.
	servo_model_index.clear();
	// get data sample
	in_joints_fixed.getDataSample(joints);
	if (isValidJointStateAccelNamePosVelAccelEffort(joints)) {
		setupServoModelIndex(joints.name);
	}
	// set data sample
	out_goals.setDataSample(goals);

	log(INFO) << "ServoInvParam is started!" << endlog();
	return true;
}

void ServoInvParam::updateHook() {

	if (in_joints_fixed.read(joints, false) == NewData) {
		// new joints positions received
		// check if message is valid
		if (!isValidJointStateAccelNamePosVelAccelEffort(joints)) {
			log(WARN) << "Message on in_joints_accel_fixed port  has incorrect structure." << endlog();
		}
		else {
			// check if index presents and have th same size as incoming message
		    if (servo_model_index.size() != joints.name.size()) {
				// setup new index
				setupServoModelIndex(joints.name);

				log(WARN) << "Size of the message on in_joints_fixed_fixed has changed. Regenerate index." << endlog();
			}

			for(int i = 0; i < joints.name.size(); i++) {
				int index = servo_model_index[i];
				const ServoModel& servo_model = (index >= 0) ? servo_models[index] : default_servo_model;
				//calculate target position using inverse model of the servo
				goals.target_pos[i] = (
					servo_model.alpha[0] * joints.acceleration[i] +
					servo_model.alpha[1] * joints.velocity[i] +
					servo_model.alpha[2] * sign(joints.velocity[i]) +
					servo_model.alpha[3] * joints.effort[i]
				  ) / (servo_model.kp * battery_voltage) +
				  joints.position[i];

				goals.target_pos[i] *= servo_models[i].kgear;
			}
			// publish goals
			out_goals.write(goals);
		}
	}


	if (in_servo_models.read(models, false) == NewData) {
		//update servo models with new data
		for (const ServoModel& model : models) {
			// find model
			auto iter = std::find_if(servo_models.begin(), servo_models.end(), [&model](const ServoModel& m) -> bool { return m.name == model.name; });
			if (iter != servo_models.end()) {
				// model found
				*iter = model;
			}
			else {
				servo_models.push_back(model);
				// update index
				if (isValidJointStateAccelNamePosVelAccelEffort(joints), servo_model_index.size()) {
					// find joint
					auto joint_it = std::find(joints.name.begin(), joints.name.end(), model.name);
					if (joint_it != joints.name.end()) {
						// update index
						servo_model_index[joint_it - joints.name.end()] = servo_models.size() - 1;
					}
				}
			}
		}
	}


	//update battery voltage rate
	if (in_battery_state.read(battery_state, false) == NewData) {
		battery_voltage = battery_state.voltage;
	}
}

void ServoInvParam::stopHook() {
	log(INFO) << "ServoInvParam stopped!" << endlog();
}

} // nmaespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ServoInvParam)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::ServoInvParam)
