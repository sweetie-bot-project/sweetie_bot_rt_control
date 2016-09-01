#include <rtt/Component.hpp>
#include <iostream>

#include "servo_inv_lead.hpp"

using namespace RTT;

ServoInvLead::ServoInvLead(std::string const& name) : 
	TaskContext(name)
{
	this->addEventPort(joints_port)
		.doc("Desired joints state.");
	this->addPort(goals_port)
		.doc("Position controlled servos goals."); 

	this->addProperty("lead", lead)
		.doc("Servo goal position lead in seconds. Goal position is equal desired position plus desired velocity multiplied by lead.")
		.set(0.0115);
}

bool ServoInvLead::startHook()
{
	Logger::In in("ServoInvLead");
	
	joints_port.getDataSample(joints);

	goals.name.resize(joints.name.size());
	goals.target_pos.resize(joints.name.size());
	goals.playtime.assign(joints.name.size(), lead);

	goals_port.setDataSample(goals);

	log(Info) << "ServoInvLead started!" << endlog();
	return true;
}

void ServoInvLead::updateHook()
{
	if (joints_port.read(joints, false) == NewData) {
		unsigned int njoints = joints.name.size();

		if (njoints != joints.position.size() || njoints != joints.velocity.size()) {
			Logger::In in("ServoInvLead");
			log(Warning) << "goal message has incorrect structure." << endlog();
			return;
		}
		
		goals.name = joints.name;
		goals.target_pos.resize(njoints);
		for(unsigned int i = 0; i < njoints; i++) {
			goals.target_pos[i] = joints.position[i] + joints.velocity[i] * this->lead;
		}
		if (goals.playtime.size() != njoints) {
			goals.playtime.assign(njoints, lead);
		}

		goals_port.write(goals);
	}
}



void ServoInvLead::stopHook() {
	Logger::In in("ServoInvLead");
	log(Info) << "ServoInvLead stopped!" << endlog();
}

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
ORO_CREATE_COMPONENT(ServoInvLead)
