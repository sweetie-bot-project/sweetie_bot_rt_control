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
	this->addProperty("filter_position", filter_position)
		.doc("If true filter will be applied to input position to calculate output position")
		.set(false);
	this->addProperty("calc_velocity", calc_velocity)
		.doc("If true d_filter will be applied to input position to calculate output velocity")
		.set(false);
	this->addProperty("calc_acceleration", calc_acceleration)
		.doc("If true d2_filter will be applied to input position to calculate output acceleration")
		.set(false);
	this->addProperty("filter", filter_param)
		.doc("Using to filter position.");
	this->addProperty("d_filter", d_filter_param)
		.doc("Using to calculate velocity.");
	this->addProperty("d2_filter", d2_filter_param)
		.doc("Using to calculate acceleration.");
}

bool JointsDiff::startHook()
{
	joints_port.getDataSample(joints);

	njoints = joints.name.size();
	joints_accel.name = joints.name;
	joints_accel.position = joints.position;
	joints_accel.velocity = joints.velocity;
	joints_accel.acceleration.assign(njoints, 0);
	joints_accel.effort = joints.effort;

	joints_accel_port.setDataSample(joints_accel);

	filter = filter_param;
	d_filter = d_filter_param;
	d2_filter = d2_filter_param;

	joints_buf.reset(max(filter.size(), max(d_filter.size(), d2_filter.size())), joints.position);

	RTT::os::Timer::TimerId timer_id;
	sync_port.readNewest(timer_id);

	new_cycle = true;

	log(INFO) << "JointsDiff started!" << endlog();
	return true;
}

void JointsDiff::updateHook()
{
	RTT::os::Timer::TimerId timer_id;
	unsigned int i, k;
	double sum;

	if (sync_port.read(timer_id) == NewData) {
		// Renew state variables.
		new_cycle = true;
	}
	if (joints_port.read(joints, false) == NewData) {

		if (njoints != joints.name.size()) {

			njoints = joints.name.size();
			joints_buf.reset(max(filter.size(), max(d_filter.size(), d2_filter.size())), joints.position);
			joints_accel.acceleration.assign(njoints, 0);
			log(WARN) << "changed joints size." << endlog();
			return;

		}
		njoints = joints.name.size();

		if (njoints != joints.position.size() || njoints != joints.velocity.size()) {
			log(WARN) << "goal message has incorrect structure." << endlog();
			return;
		}

		if (new_cycle) {
			joints_buf.pos_to_put() = joints.position;
			new_cycle = false;
		} else {
			joints_buf.pos_to_reput() = joints.position;
		}

		joints_accel.name = joints.name;
		joints_accel.position = joints.position;
		joints_accel.velocity = joints.velocity;
		joints_accel.effort = joints.effort;

		if (filter_position) {

			for (i = 0; i < njoints; i++) {
				sum = 0;
				for (k = 0; k < filter.size(); k++) {
					sum += joints_buf[k][i] * filter[k];
				}
				joints_accel.position[i] = sum;
			}
		}

		if (calc_velocity) {

			for (i = 0; i < njoints; i++) {
				sum = 0;
				for (k = 0; k < d_filter.size(); k++) {
					sum += joints_buf[k][i] * d_filter[k];
				}
				joints_accel.velocity[i] = sum;
			}
		}

		if (calc_acceleration) {

			for (i = 0; i < njoints; i++) {
				sum = 0;
				for (k = 0; k < d2_filter.size(); k++) {
					sum += joints_buf[k][i] * d2_filter[k];
				}
				joints_accel.acceleration[i] = sum;
			}
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
