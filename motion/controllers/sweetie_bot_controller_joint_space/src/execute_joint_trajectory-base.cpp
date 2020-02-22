#include "execute_joint_trajectory-base.hpp"

#include <rtt/Component.hpp>

#include <sweetie_bot_orocos_misc/joint_state_check.hpp>
#include <sweetie_bot_orocos_misc/get_subservice_by_type.hpp>

using namespace RTT;

namespace sweetie_bot {
namespace motion {
namespace controller {

//TODO move somewhere
ExecuteJointTrajectoryBase::ExecuteJointTrajectoryBase(std::string const& name) : 
	TaskContext(name),
	log(logger::categoryFromComponentName(name))
{

	// ports
	// PORTS: input
	this->addEventPort("sync", sync_port)
		.doc("Timer syncronization event. This event triggers controller execution cycle.");
	this->addPort("in_joints_sorted", in_joints_port).
		doc("Full sorted actual robot pose (from sensors or from aggregator).");
	// PORTS: output
	this->addPort("out_joints_ref_fixed", out_joints_port).
		doc("Reference joint positions for aggregator.");
	this->addPort("out_supports", out_supports_port).
		doc("Expected contact points for trajectory.");
	// PROPERTIES
	this->addProperty("period", period)
		.doc("Discretization period (s)");
	std::ostringstream desc;
	desc << "Type of algorithm wich will be use for interpolation: " << (int) ModifiedAkima << " -- modified Akima spline, " << (int) ModifiedCubic << " -- modified cubic spline, " 
		<< (int) AkimaSpline << " -- Akima spline, " << (int) CubicSpline << " -- cubic spline.";
	this->addProperty("algorithm", algorithm_type)
		.doc(desc.str())
		.set(InterpolationAlgorithmType::ModifiedAkima);
	this->addProperty("stop_threshold", stop_threshold)
		.doc("If joints positions difference is less then threshold (rad) they assumed to be equal.")
		.set(0.0005);
	// SERVICE: reqiures
	robot_model = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));
}

bool ExecuteJointTrajectoryBase::dataOnPortHook( RTT::base::PortInterface* portInterface ) 
{
	// Process EventPorts messages callbacks if component is in configured state,
	// so actionlib hooks works even if component is stopped.
	//
	// WARNING: works only in OROCOS 2.9 !!!
	//
    return this->isConfigured();
}

bool ExecuteJointTrajectoryBase::configureHook()
{
	// Get ResourceClient plugin interface.
	resource_client = getSubServiceByType<ResourceClientInterface>(this->provides().get());
	if (!resource_client) {
		log(ERROR) << "ResourceClient plugin is not loaded." << endlog();
		return false;
	}
	// check if RobotModel Service presents
	if (!robot_model->ready() || !robot_model->isConfigured()) {
		log(ERROR) << "RobotModel service is not ready." << endlog();
		return false;
	}
	this->n_joints_fullpose = robot_model->listJoints().size();
	// check if filter present
	filter = getSubServiceByType<filter::FilterJointStateInterface>(this->provides().get());
	if (filter) {
		log(INFO) << "Trajectory Filter service is loaded." << endlog();
	}
	// reset time
	time_from_start = 0;
	return true;
}

bool ExecuteJointTrajectoryBase::startHook()
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

void ExecuteJointTrajectoryBase::updateHook()
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
		this->goal_active->getSupportState(time_from_start, support_state);

		// perform trajectory smoothing and put result in ref_pose
		if (filter) filter->update(actual_pose, ref_pose, ref_pose);

		// call implementation dependent code
		operationalHook(on_goal);
		
		// publish new reference position
		if (support_state.name.size() > 0) out_supports_port.write(support_state);
		out_joints_port.write(ref_pose);
	}
	// shift time
	this->time_from_start += this->period;
}



void ExecuteJointTrajectoryBase::cleanupHook() 
{
	goal_active.reset();
	resource_client = 0; // in PreOperational state resource_client is ont available
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

