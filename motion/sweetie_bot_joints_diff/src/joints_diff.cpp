#include <rtt/Component.hpp>
#include <iostream>

#include "joints_diff.hpp"

using namespace RTT;
using sweetie_bot::logger::Logger;

namespace sweetie_bot {

namespace motion {


JointsDiff::JointsDiff(std::string const& name) :
	TaskContext(name),
	log(logger::categoryFromComponentName(name))
{
	if (!log.ready()) {
		RTT::Logger::In in("JointsDiff");
		RTT::log(RTT::Error) << "Logger is not ready!" << endlog();
		this->fatal();
		return;
	}

	this->addEventPort("in_joints_fixed", joints_port)
		.doc("JointState type.");
	this->addPort("out_joints_accel_fixed", joints_accel_port)
		.doc("JointStateAccel type. It is JointState with acceleration.");
	this->addPort("sync_step", sync_port)
		.doc("Timer event indicating beginig of next control cycle.");

	this->addProperty("period", period)
		.doc("Control cycle duration (seconds).")
		.set(0.0224);
}

bool JointsDiff::startHook()
{
	joints_port.getDataSample(joints);

	vel_prev.assign(joints.name.size(), 0);

	joints_accel.name = joints.name;
	joints_accel.position = joints.position;
	joints_accel.velocity = joints.velocity;
	joints_accel.acceleration.assign(joints.name.size(), 0);
	joints_accel.effort = joints.effort;

	joints_accel_port.setDataSample(joints_accel);

	RTT::os::Timer::TimerId timer_id;
	sync_port.readNewest(timer_id);

	log(INFO) << "JointsDiff started!" << endlog();
	return true;
}

void JointsDiff::updateHook()
{
	RTT::os::Timer::TimerId timer_id;

	if (sync_port.read(timer_id) == NewData) {
		// Renew state variables.
		// At end of control period desired position of servo must be equal to reference position.
		vel_prev = joints.velocity;
	}
	if (joints_port.read(joints, false) == NewData) {
		unsigned int njoints = joints.name.size();

		if (njoints != joints.position.size() || njoints != joints.velocity.size()) {
			log(WARN) << "goal message has incorrect structure." << endlog();
			return;
		}

		joints_accel.name = joints.name;
		joints_accel.position = joints.position;
		joints_accel.velocity = joints.velocity;
		joints_accel.effort = joints.effort;

		if (vel_prev.size() == njoints) {

			for(unsigned int i = 0; i < njoints; i++) {
				joints_accel.acceleration[i] = (joints.velocity[i] - vel_prev[i]) / period;
			}
		}
		else {
			// message size has changed, fallback to direct assigment
			joints_accel.acceleration.assign(njoints, 0);
		}

		joints_accel_port.write(joints_accel);
	}
}



void JointsDiff::stopHook() {
	log(INFO) << "JointsDiff stopped!" << endlog();
}

} // nmaespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(JointsDiff)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::JointsDiff)
