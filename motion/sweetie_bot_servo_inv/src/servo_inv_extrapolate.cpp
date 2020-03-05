#include <rtt/Component.hpp>
#include <iostream>

#include "servo_inv_extrapolate.hpp"

#include <sweetie_bot_orocos_misc/joint_state_check.hpp>

using namespace RTT;
using sweetie_bot::logger::Logger;

namespace sweetie_bot {

namespace motion {


ServoInvExtrapolate::ServoInvExtrapolate(std::string const& name) : 
	TaskContext(name),
	log(logger::categoryFromComponentName(name))
{
	if (!log.ready()) {
		RTT::Logger::In in("ServoInvExtrapolate");
		RTT::log(RTT::Error) << "Logger is not ready!" << endlog();
		this->fatal();
		return;
	}
			
	this->addEventPort("in_joints_fixed", joints_port)
		.doc("Desired joints state. Order of joints should not change. ");
	this->addPort("out_goals", goals_port)
		.doc("Position controlled servos goals."); 

	this->addProperty("period", period)
		.doc("Control cycle duration (seconds).")
		.set(0.0224);
	this->addProperty("lead", lead)
		.doc("Goal position lead in seconds. Goal position is equal desired position plus desired velocity multiplied by lead.")
		.set(0.0112);
	this->addProperty("extrapolate_position", extrapolate_position)
		.doc("Goal position is extrapolated to future using velocity for one period ")
		.set(false);
}

bool ServoInvExtrapolate::processJointStateSample(const sensor_msgs::JointState& joints) 
{
	// prepare transmit buffer
	goals.name = joints.name;
	goals.target_pos.resize(joints.name.size());
	goals.playtime.assign(joints.name.size(), period + lead);
	return true;
}

bool ServoInvExtrapolate::startHook()
{
	joints_port.getDataSample(joints);

	// resize buffers
	if (!processJointStateSample(joints)) return false;

	// provide data sample
	goals_port.setDataSample(goals);

	log(INFO) << "ServoInvExtrapolate started!" << endlog();
	return true;
}

void ServoInvExtrapolate::updateHook()
{
	if (joints_port.read(joints, false) == NewData) {
		if (!isValidJointStatePosVel(joints)) {
			log(WARN) << "JointState message has incorrect structure." << endlog();
			return;
		}

		unsigned int njoints = joints.name.size();
	
		// check if array size has changed
		// we does not check name content: due to fixed requipment it should be always the same.
		if (njoints != goals.name.size()) {
			processJointStateSample(joints);
			log(WARN) << "Size of the in_joints port message has changed." << endlog();
		}

		// calculate extrapolation lead period
		double extrapolation_lead = lead + ( extrapolate_position ? period : 0.0 );
		// perform extrapolation
		for(unsigned int i = 0; i < njoints; i++) {
			goals.target_pos[i] = joints.position[i] + joints.velocity[i]*extrapolation_lead;
		}
		/*if (joints.acceleration.size() != 0) {
			for(unsigned int i = 0; i < njoints; i++) {
				goals.target_pos[i] = joints.position[i] + (joints.velocity[i] + 0.5*joints.acceleration[i]*extrapolation_lead)*extrapolation_lead;
			}
		}*/

		goals_port.write(goals);
	}
}



void ServoInvExtrapolate::stopHook() {
	log(INFO) << "ServoInvExtrapolate stopped!" << endlog();
}

} // nmaespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ServoInvExtrapolate)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::ServoInvExtrapolate)
