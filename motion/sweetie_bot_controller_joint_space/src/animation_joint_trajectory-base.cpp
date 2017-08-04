#include "animation_joint_trajectory-base.hpp"

#include <rtt/Component.hpp>

#include <sweetie_bot_orocos_misc/joint_state_check.hpp>
#include <sweetie_bot_orocos_misc/get_subservice_by_type.hpp>

using namespace RTT;

namespace sweetie_bot {
namespace motion {
namespace controller {

//TODO move somewhere
AnimJointTrajectoryBase::AnimJointTrajectoryBase(std::string const& name) : 
	TaskContext(name),
	log(logger::categoryFromComponentName(name))
{

	// ports
	// PORTS: input
	this->addEventPort("sync", sync_port)
		.doc("Timer syncronization event. This event triggers controller execution cycle.");
	this->addPort("in_joints_sorted", in_joints_port).
		doc("Full sorted actual robot pose (from sensors or from agregator).");
	// PORTS: output
	this->addPort("out_joints_ref_fixed", out_joints_port).
		doc("Reference joint positions for agregator.");
	// PROPERTIES
	this->addProperty("period", period)
		.doc("Discretization period (s)");
	// SERVICE: reqiures
	robot_model = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));
}

bool AnimJointTrajectoryBase::dataOnPortHook( RTT::base::PortInterface* portInterface ) 
{
	// Process EventPorts messages callbacks if component is in configured state,
	// so actionlib hooks works even if component is stopped.
	//
	// WARNING: works only in OROCOS 2.9 !!!
	//
    return this->isConfigured();
}

bool AnimJointTrajectoryBase::configureHook()
{
	// Get ResourceClient plugin interface.
	resource_client = getSubServiceByType<ResourceClientInterface>(this->provides().get());
	if (!resource_client) {
		log(ERROR) << "ResourceClient plugin is not loaded." << endlog();
		return false;
	}
	// check if RobotModel Service presents
	if (!robot_model->ready()) {
		log(ERROR) << "RobotModel service is not ready." << endlog();
		return false;
	}
	this->n_joints_fullpose = robot_model->listJoints("").size();
	// check if filter present
	filter = getSubServiceByType<filter::TransientJointStateInterface>(this->provides().get());
	if (filter) {
		log(INFO) << "Trajectory Filter service is loaded." << endlog();
	}
	// reset time
	time_from_start = 0;
	return true;
}

bool AnimJointTrajectoryBase::startHook()
{
	in_joints_port.read(actual_fullpose, true);
	// reset filter
	if (filter) {
		bool filter_reset = false;
		if (isValidJointStateNamePos(actual_fullpose, n_joints_fullpose) && this->goal_pending) {
			goal_pending->prepareJointStateBuffer(actual_pose);
			this->goal_pending->selectJoints(actual_fullpose, actual_pose);
			filter_reset = filter->reset(actual_pose, period);
		}
		if (!filter_reset) log(ERROR) << "JointState filter reset has failed (no pending goal, pose unavailable or filter internal error). " << endlog();
	}
	// clear sync port buffer
	RTT::os::Timer::TimerId timer_id;
	sync_port.readNewest(timer_id);
	// check if port connected
	if (!sync_port.connected()) log(WARN) << "'sync' port is not connected." << endlog();
	if (!in_joints_port.connected()) log(WARN) << in_joints_port.getName() << " port is not connected." << endlog();
	if (!out_joints_port.connected()) log(WARN) << out_joints_port.getName() << " port is not connected." << endlog();
	// now update hook will be periodically executed
	return true;
}

void AnimJointTrajectoryBase::updateHook()
{
	// check messages on resource_assigment port
	resource_client->step();

	// syncronize with sync messages
	{
		RTT::os::Timer::TimerId unused;
		if (sync_port.read(unused) != NewData) return;
	}

	if (this->resource_client->isOperational() && this->goal_active) {
		// read pose of robot
		in_joints_port.read(actual_fullpose, false);
		// get actual position of controlled joints
		if (isValidJointStatePos(actual_fullpose, n_joints_fullpose)) {
			this->goal_active->selectJoints(actual_fullpose, actual_pose);
		}
		// get desired pose	
		bool on_goal = this->goal_active->getJointState(time_from_start, ref_pose);
		if (log(DEBUG)) {
			log() << " t = " << time_from_start << " actual: " << actual_pose << endlog();
			log() << " t = " << time_from_start << " ref: " << ref_pose << endlog();
		}

		// perform trajectory smoothing and put result in ref_pose
		if (filter && filter->update(actual_pose, ref_pose, ref_pose)) {
			if (log(DEBUG)) {
				log() << " t = " << time_from_start << " filtered: " << ref_pose << endlog();
			}
		}

		// call implementation dependent code
		operationalHook(on_goal);
		
		// publish new reference position
		out_joints_port.write(ref_pose);
	}
	// shift time
	this->time_from_start += this->period;
}



void AnimJointTrajectoryBase::cleanupHook() 
{
	goal_active.reset();
	resource_client = 0; // in PreOperational state resource_client is ont available
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

