#include <rtt/Component.hpp>
#include <iostream>

#include "servo_inv_lead.hpp"

using namespace RTT;
using sweetie_bot::logger::Logger;

namespace sweetie_bot {

namespace motion {


ServoInvLead::ServoInvLead(std::string const& name) : 
	TaskContext(name),
	log(logger::getDefaultCategory("sweetie_bot.motion") + "." + name)
{
	if (!log.ready()) {
		RTT::Logger::In in("ServoInvLead");
		RTT::log(RTT::Error) << "Logger is not ready!" << endlog();
		this->fatal();
		return;
	}
			
	this->addEventPort("in_joints_fixed", joints_port)
		.doc("Desired joints state. Order of joints should not change. ");
	this->addPort("out_goals", goals_port)
		.doc("Position controlled servos goals."); 
	this->addPort("sync_step", sync_port)
		.doc("Timer event indicating beginig of next control cycle."); 

	this->addProperty("period", period)
		.doc("Control cycle duration (seconds).")
		.set(0.023);
	this->addProperty("lead", lead)
		.doc("Goal position lead in seconds. Goal position is equal desired position plus desired velocity multiplied by lead.")
		.set(0.0115);
}

bool ServoInvLead::startHook()
{
	joints_port.getDataSample(joints);

	position_perv.clear();
	position_perv.reserve(joints.name.size());

	goals.name.resize(joints.name.size());
	goals.target_pos.resize(joints.name.size());
	goals.playtime.assign(joints.name.size(), period + lead);

	goals_port.setDataSample(goals);

	RTT::os::Timer::TimerId timer_id;
	sync_port.readNewest(timer_id);

	log(INFO) << "ServoInvLead started!" << endlog();
	return true;
}

void ServoInvLead::updateHook()
{
	RTT::os::Timer::TimerId timer_id;

	if (sync_port.read(timer_id) == NewData) {
		// Renew state variables.
		// At end of control period desired position of servo must be equal to reference position.
		position_perv = joints.position;
	}
	if (joints_port.read(joints, false) == NewData) {
		unsigned int njoints = joints.name.size();

		if (njoints != joints.position.size() || njoints != joints.velocity.size()) {
			log(WARN) << "goal message has incorrect structure." << endlog();
			return;
		}
		
		goals.name = joints.name;

		if (position_perv.size() == njoints) {
			// take defined lead
			goals.target_pos.resize(njoints);
			for(unsigned int i = 0; i < njoints; i++) {
				goals.target_pos[i] = joints.position[i] + (joints.position[i] - position_perv[i]) * lead / period;
			}
			goals.playtime.assign(njoints, period + lead);
		}
		else {
			// message size has changed, fallback to direct assigment
			goals.target_pos = joints.position;
			goals.playtime.assign(njoints, period);
		}

		goals_port.write(goals);
	}
}



void ServoInvLead::stopHook() {
	log(INFO) << "ServoInvLead stopped!" << endlog();
}

} // nmaespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ServoInvLead)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::ServoInvLead)
