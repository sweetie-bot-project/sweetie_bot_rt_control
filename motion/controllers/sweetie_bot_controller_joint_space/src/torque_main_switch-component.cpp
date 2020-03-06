#include "torque_main_switch-component.hpp"

#include <rtt/Component.hpp>

#include "sweetie_bot_orocos_misc/get_subservice_by_type.hpp"
#include "sweetie_bot_orocos_misc/joint_state_check.hpp"

using namespace RTT;
using namespace std;

static std::ostream& operator<<(std::ostream& s, const std::vector<std::string>& strings) 
{
	s << "[ ";
	for(auto it = strings.begin(); it != strings.end(); it++) s << *it << ", ";
	s << " ]";
	return s;
}

namespace sweetie_bot {
namespace motion {
namespace controller {

TorqueMainSwitch::TorqueMainSwitch(std::string const& name)  : 
	SimpleControllerBase(name)
{
	this->provides()->doc("In operatioanl state switch servos torque off and feedfodward actual position to reference.");

	// ports
	// PORTS: input
	this->addPort("in_joints_actual", in_joints_port).
		doc("Full sorted actual robot pose (from sensors or from aggregator).");

	// PORTS: output
	this->addPort("out_joints_ref", out_joints_port).
		doc("Reference joint positions for aggregator.");

	// PROPERTIES
	this->addProperty("herkulex_scheds", herkulex_scheds)
		.doc("List of HerkulexSched components should be stopped when servos are being switched off.");

	this->addProperty("herkulex_arrays", herkulex_arrays)
		.doc("List of controlled HerkulexArrays components.");

	this->addProperty("velocity_zeroing", velocity_zeroing)
		.doc("Set velocity to zero before coping it to reference pose. Can be handy if velocity measurements are noisy.")
		.set(false);

	base::PropertyBase * support_legs = this->getProperty("controlled_chains");
	Property< std::vector<std::string> > support_legs_prop(support_legs);
	if (support_legs_prop.ready()) {
		std::vector<std::string> legs = { "leg1", "leg2", "leg3", "leg4", "head" };
		support_legs->setName("controlled_groups");
		support_legs->setDescription("Robot kinematic chains which are in contact.");
		support_legs_prop.set(legs);
	}

	// operations: provided
	// Service: reqires
	robot_model = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));
	
	// other actions
	log(INFO) << "TorqueMainSwitch is constructed!" << endlog();
}

bool TorqueMainSwitch::configureHook_impl()
{
	// INITIALIZATION
	// check if RobotModel Service presents
	if (!robot_model->ready() || !robot_model->isConfigured()) {
		log(ERROR) << "RobotModel service is not ready." << endlog();
		return false;
	}
	//int n_joints_fullpose = robot_model->listJoints("").size();
	// allocate memory
	in_joints_port.getDataSample(actual_fullpose);
	// set ports data samples
	out_joints_port.setDataSample(actual_fullpose);

	// Get necessary OperationCallers
	if (herkulex_arrays.size() == 0) {
		log(WARN) << "No HerkulexArrays is provided." << endlog();
		return false;
	}
	if (herkulex_arrays.size() != herkulex_scheds.size()) {
		log(WARN) << "Number of HerkulexArrays and HerkulexScheds is not same. Configuration may be wrong." << endlog();
	}
	// clear list of controlled groups
	controlled_groups.clear();

	// check HerkulexArrays
	// form joint list and get OperationCallers
	herkulex_groups.clear();
	std::map<std::string, int> joints_to_herkulex_group_map;
	for(int i = 0; i < herkulex_arrays.size(); i++) {
		// add herkulex group info 
		herkulex_groups.emplace_back();
		HerkulexGroupInfo& group = herkulex_groups.back();
		group.array = herkulex_arrays[i];
		group.sched = herkulex_scheds[i];
		// get HerkulexArray 
		TaskContext * herk_array = this->getPeer(group.array);
		if ( !herk_array ) {
			log(ERROR) << "Peer with name `" << group.array << "` is not found." << endlog();
			return false;
		}
		// check HerkulexSched
		if ( ! this->getPeer(group.sched) ) {
			log(ERROR) << "Peer with name `" << group.sched << "` is not found." << endlog();
			return false;
		}
		// get setRegisterRAM operation
		group.setTorqueFree_caller = OperationCaller< bool(std::string, bool) >(herk_array->getOperation("setTorqueFree"), this->engine());
		if ( ! group.setTorqueFree_caller.ready() ) {
			log(ERROR) << "`" << group.array << ".setTorqueFree()` operation is not ready." << endlog();
			return false;
		}
		// get list of servos
		OperationCaller< vector<string>() > listServos_caller(herk_array->getOperation("listServos"), this->engine()) ;
		if ( ! listServos_caller.ready() ) {
			log(ERROR) << "`" << group.array << ".listServos` operation is not ready." << endlog();
			return false;
		}
		std::vector<std::string> new_joints = listServos_caller();
		int index = herkulex_groups.size() - 1;
		for(const std::string& joint_name : new_joints) {
			joints_to_herkulex_group_map[joint_name] = index;
		}
	}

	// extract information about joints from RobotModel
	joint_groups_index.clear();
	std::vector<std::string> group_names = robot_model->listGroups();
	for (const auto& group_name : group_names) {
		// add new group
		auto emplace_it = joint_groups_index.emplace( group_name, JointGroupInfo() ); // returns (map iterator, bool) pair
		JointGroupInfo& group = emplace_it.first->second;
		// add joints 
		std::vector<std::string> joints_names = robot_model->getGroupJoints(group_name);
		for (const auto& joint_name : joints_names) {
			// find name in herkulex joints list
			auto joint_it = joints_to_herkulex_group_map.find(joint_name);
			if (joint_it == joints_to_herkulex_group_map.end()) continue;
			int group_index = joint_it->second;
			// joint found, add information about it
			group.joints.emplace_back();
			JointInfo& joint = group.joints.back();
			joint.name = joint_name;
			joint.setTorqueFree_caller = &herkulex_groups[group_index].setTorqueFree_caller;
			// associate joint group with herkulex group
			auto group_it = std::find(group.herkulex_groups_induces.begin(), group.herkulex_groups_induces.end(), joint_it->second);
			if (group_it == group.herkulex_groups_induces.end()) group.herkulex_groups_induces.push_back(group_index);
		}
	}	

	if (log(DEBUG)) {
		for(auto group_it = joint_groups_index.begin(); group_it != joint_groups_index.end(); group_it++) {
			log() << "Kinematics group: " << group_it->first << " joints: [ ";
			for(JointInfo& joint : group_it->second.joints) log() << joint.name << ", ";
			log() << " ], scheds : [ "; 
			for(int i : group_it->second.herkulex_groups_induces) log() << herkulex_groups[i].sched << ", ";
			log() << " ], arrays : [ "; 
			for(int i : group_it->second.herkulex_groups_induces) log() << herkulex_groups[i].array << ", ";
			log() << "]" << endlog(); 
		}
	}

	log(INFO) << "TorqueMainSwitch is configured !" << endlog();
	return true;
}

bool TorqueMainSwitch::startHook_impl(StateChangeReason reason)
{
	in_joints_port.getDataSample(actual_fullpose);
	// clear sync port buffer
	RTT::os::Timer::TimerId timer_id;
	sync_port.readNewest(timer_id);
	// now update hook will be periodically executed
	log(INFO) << "TorqueMainSwitch is started !" << endlog();
	return true;
}

bool TorqueMainSwitch::setSchedulersActive(const JointGroupInfo& group , bool is_active) 
{
	bool success = true;

	for(int index : group.herkulex_groups_induces) {
		const std::string& sched_name =  herkulex_groups[index].sched;

		TaskContext * sched = this->getPeer( sched_name );
		if (!sched) {
			success = false;
			log(WARN) << "Unable to get scheduler: " << sched_name << endlog();
			continue;
		}
		if (!is_active && sched->isRunning()) {
			if (!sched->stop()) {
				success = false;
				log(ERROR) << "Error during stopping scheduler " << sched_name << endlog();
			}
		}
		else if(is_active && !sched->isRunning()) {
			if (!sched->start()) {
				success = false;
				log(ERROR) << "Error during starting  scheduler " << sched_name << endlog();
			}
		}
		else {
			// no action performed: someone already stopped or started scheduler
			success = false;
		}
	}
	return success;
}

bool TorqueMainSwitch::setAllServosTorqueFree(bool torque_is_off) 
{
	bool success = true;
	
	for(const std::string& group_name : controlled_groups) {
		auto it = joint_groups_index.find(group_name);
		if (it == joint_groups_index.end()) {
			log(WARN) << "Unknown joint group: " << group_name << endlog();
			continue;
		}
		JointGroupInfo& group = it->second;

		// switch off all scedulers
		bool was_running = setSchedulersActive(group, false);

		// switch all servos torque off
		// TODO asyncronic call
		// TODO retry 
		for(const JointInfo& joint : group.joints) {
			if ( !(joint.setTorqueFree_caller->ready() && joint.setTorqueFree_caller->call(joint.name, torque_is_off)) ) {
				success = false;
			}
		}
		// switch on all scedulers
		setSchedulersActive(group, was_running);
	}

	return success;
}

bool TorqueMainSwitch::resourceChangedHook_impl(const std::vector<std::string>& set_operational_goal_resources, const std::vector<std::string>& requested_resources)
{
	// check if resources available
	if (! resource_client->hasResources(requested_resources)) {
		// if resource set as changed then stop controller
		return false;
	}
	// buffer kinematics group list
	controlled_groups = requested_resources;
	// unconditionally switch servos off
	log(INFO) << "Setting servo torque OFF for groups " << controlled_groups << endlog();
	if ( !setAllServosTorqueFree(true) ) {
		log(ERROR) << "Unable to set servo torque!" << endlog();
		return false;
	}
	// form joint list to be published
	pose_published.name.clear();
	for(const std::string& group_name : requested_resources) {
		auto it = joint_groups_index.find(group_name);
		if (it == joint_groups_index.end()) continue; // already logged in setAllServosTorqueFree
		// add joints to list
		for(const JointInfo& joint : it->second.joints) {
			pose_published.name.push_back(joint.name);
		}
	}
	// allocate output buffer
	int n_joints = pose_published.name.size();
	pose_published.position.resize(n_joints);
	pose_published.velocity.assign(n_joints, 0);
}

void TorqueMainSwitch::updateHook_impl()
{
	// read port
	in_joints_port.read(actual_fullpose, true);

	if (!isValidJointStatePosVel(actual_fullpose)) {
		log(WARN) << "invalid pose on in_joints_actual port" << endlog();
		return;
	}
	// form new pose vector
	for(int i = 0; i < actual_fullpose.name.size(); i++) {
		auto it = std::find(pose_published.name.begin(), pose_published.name.end(), actual_fullpose.name[i]); // TODO more effective
		if (it == pose_published.name.end()) continue;
		// calculate index
		int j = it - pose_published.name.begin();
		// copy pose
		pose_published.position[j] = actual_fullpose.position[i];
		if (velocity_zeroing) pose_published.velocity[j] = 0;
		else pose_published.velocity[j] = actual_fullpose.velocity[i];
	}
	// publish new reference position
	out_joints_port.write(pose_published);
}

/* 
 * Preempts the controllers and releases its resources.
 */
void TorqueMainSwitch::stopHook_impl(StateChangeReason reason) 
{
	// change servos state
	log(INFO) << "Setting servo torque ON" << endlog();
	if ( !setAllServosTorqueFree(false) ) {
		log(ERROR) << "Unable to set servo torque!" << endlog();
	}
	log(INFO) << "TorqueMainSwitch is stopped!" << endlog();
}

void TorqueMainSwitch::cleanupHook_impl() 
{
	controlled_groups.clear();
	joint_groups_index.clear();
	herkulex_groups.clear();
	log(INFO) << "TorqueMainSwitch cleaning up !" << endlog();
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(TorqueMainSwitch)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::controller::TorqueMainSwitch)
