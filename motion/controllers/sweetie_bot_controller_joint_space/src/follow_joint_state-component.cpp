#include "follow_joint_state-component.hpp"

#include <rtt/Component.hpp>

#include "sweetie_bot_orocos_misc/get_subservice_by_type.hpp"
#include "sweetie_bot_orocos_misc/joint_state_check.hpp"

using namespace RTT;
using namespace std;

inline std::ostream& operator<<(std::ostream& s, const std::vector<std::string>& strings) 
{
	s << "[ ";
	for(auto it = strings.begin(); it != strings.end(); it++) s << *it << ", ";
	s << " ]";
	return s;
}

namespace sweetie_bot {
namespace motion {
namespace controller {


FollowJointState::FollowJointState(std::string const& name)  : 
	SimpleControllerBase(name)
{
	this->provides()->doc("Feedforward JointState reference from high-level to aggregator.");

	// ports
	// PORTS: input
	this->addPort("in_joints_sorted", in_joints_port).
		doc("Full sorted actual robot pose (from sensors or from aggregator).");

	this->addPort("in_joints_ref", in_joints_ref_port).
		doc("Desired joint positions.");

	// PORTS: output
	this->addPort("out_joints_src_reset", out_joints_src_reset_port).
		doc("After start actual position is published for activation_delay secondsd on this port.");

	this->addPort("out_joints_ref_fixed", out_joints_port).
		doc("Reference joint positions for aggregator.");

	this->addPort("out_supports", out_supports_port).
		doc("Active contact list. All controlled kinematic chains are assumed free.");

	// properties
	this->addProperty("activation_delay", activation_delay).
		doc("After start wait for activation_delay seconds before start processing input and publish actual pose on out_joints_src_reset.")
		.set(0.0);

	this->addProperty("publish_supports", publish_supports).
		doc("Publish list of active contacts. All controlled kinematic chains are assumed free.")
		.set(true);

	// Service: requires
	robot_model = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));
	
	// other actions
	log(INFO) << "FollowJointState is constructed!" << endlog();
}

/**
 * Create joint index for given kinematic chain list. 
 * Adjust actual_pose and ref_pose buffer sizes.
 */
bool FollowJointState::formJointIndex(const vector<string>& controlled_chains)
{
	controlled_joints.clear();
	ref_pose.name.clear();

	vector<string> joint_names;
	for(auto chain = controlled_chains.begin(); chain != controlled_chains.end(); chain++) {
		joint_names = robot_model->listJoints(*chain);
		if (joint_names.size() == 0) {
			log(ERROR) << "Empty joint group `" << *chain << "`." << endlog();
			return false;
		}
		for(auto joint = joint_names.begin(); joint != joint_names.end(); joint++) {
			// add joint to index
			controlled_joints.insert(std::make_pair(*joint, JointIndex(robot_model->getJointIndex(*joint), controlled_joints.size())));
			ref_pose.name.push_back(*joint);
		}
	}
	if (log(INFO)) {
		log() << "Controlled chains: " << controlled_chains << endlog();
		log() << "Controlled joints: " << ref_pose.name << endlog();
	}

	// allocate memory
	unsigned int sz = controlled_joints.size();
	ref_pose.position.assign(sz, 0);
	ref_pose.velocity.assign(sz, 0);
	//ref_pose.effort.assign(sz, 0);
	actual_pose = ref_pose;
	// supports
	supports.name = controlled_chains;
	supports.contact.assign(supports.name.size(), "");
	supports.support.assign(supports.name.size(), 0.0);

	// set timestamp to delay actual input processing
	activation_timestamp = os::TimeService::Instance()->getTicks();

	return true;
}


bool FollowJointState::configureHook_impl()
{
	// INITIALIZATION
	// check if filter present
	filter = getSubServiceByType<filter::FilterJointStateInterface>(this->provides().get());
	if (filter) {
		log(INFO) << "Trajectory Filter service is loaded." << endlog();
	}
	// check if RobotModel Service presents
	if (!robot_model->ready() || !robot_model->isConfigured()) {
		log(ERROR) << "RobotModel service is not ready." << endlog();
		return false;
	}
	n_joints_fullpose = robot_model->listJoints("").size();
	// build joints index 
	if (!formJointIndex(default_resource_set)) return false;
	// allocate memory
	actual_fullpose.name.resize(n_joints_fullpose);
	actual_fullpose.position.resize(n_joints_fullpose);
	actual_fullpose.velocity.resize(n_joints_fullpose);
	// set ports data samples
	out_joints_port.setDataSample(ref_pose);
	out_joints_src_reset_port.setDataSample(actual_fullpose);
	out_supports_port.setDataSample(supports);

	log(INFO) << "FollowJointState is configured !" << endlog();
	return true;
}

/* 
 * Tries to make the controller operational. 
 *
 * Sends a resource request. Later a reply to which will be processed by the plugin's 
 * callback resourceChangedHook. If it returns true controller will become operational.
 * in the updateHook if all resources were allocated or not, if some are lacking.
 * This is checked using the controller's resourceChangedHook.
 */
bool FollowJointState::startHook_impl()
{
	in_joints_port.getDataSample(actual_fullpose);
	in_joints_ref_port.getDataSample(ref_pose_unsorted);
	// now update hook will be periodically executed if controller is operational
	log(INFO) << "FollowJointState is started !" << endlog();
	return true;
}

bool FollowJointState::resourceChangedHook_impl(const std::vector<std::string>& requested_resource_set)
{
	// Controller is able to work with arbitrary joint set.
	// Just rebuild index and continue.
	if (!formJointIndex(resource_client->listResources())) return false;

	// inform about support state change
	if (publish_supports) out_supports_port.write(supports);

	// Now we are starting motion. Due to activation_delay 
	// reference position is not available. So we have to guess it.
	// Choice: stop in current position.
	
	// get actual pose
	in_joints_port.read(actual_fullpose);
	// copy controlled joint from actual_pose
	if (isValidJointStatePos(actual_fullpose, n_joints_fullpose)) {
		for(JointIndexes::const_iterator it = controlled_joints.begin(); it != controlled_joints.end(); it++) {
			actual_pose.position[it->second.index] = actual_fullpose.position[it->second.index_fullpose];
			if (actual_fullpose.velocity.size()) actual_pose.velocity[it->second.index] = actual_fullpose.velocity[it->second.index_fullpose];
			else actual_pose.velocity[it->second.index] = 0.0;
		}
	}
	else {
		log(WARN) << "Actual pose is unavailable. ." << endlog();
		actual_pose.position.assign(controlled_joints.size(), 0.0);
		actual_pose.velocity.assign(controlled_joints.size(), 0.0);
	}
	// reset filter
	if (filter) {
		if (!filter->reset(actual_pose, period)) log(ERROR) << "JointState filter reset has failed." << endlog();
	}
	// set reference position
	ref_pose.position = actual_pose.position;
	ref_pose.velocity.assign(controlled_joints.size(), 0.0);
	return true;
}

void FollowJointState::updateHook_impl()
{
	// read port
	if (in_joints_port.read(actual_fullpose, false) == NewData) {
		// copy controlled joint from actual_pose
		if (isValidJointStatePos(actual_fullpose, n_joints_fullpose)) {
			for(JointIndexes::const_iterator it = controlled_joints.begin(); it != controlled_joints.end(); it++) {
				actual_pose.position[it->second.index] = actual_fullpose.position[it->second.index_fullpose];
				if (actual_fullpose.velocity.size()) actual_pose.velocity[it->second.index] = actual_fullpose.velocity[it->second.index_fullpose];
				else actual_pose.velocity[it->second.index] = 0;
				//TODO effort support
			}
		}
	}

	// delay actulal input processing
	// This delay allows non-relatime components to adapt to robot pose.
	if (os::TimeService::Instance()->secondsSince(activation_timestamp) < activation_delay) {
		// publish pose for non-realtime controller to adjust it to current pose
		out_joints_src_reset_port.write(actual_fullpose);
	}
	else {
		// read port	
		if (in_joints_ref_port.read(ref_pose_unsorted, false) == NewData) {
			// copy controlled joint to ref_pose
			if (isValidJointStateNamePos(ref_pose_unsorted)) {
				for(int i = 0; i < ref_pose_unsorted.name.size(); i++) {
					JointIndexes::const_iterator found = controlled_joints.find(ref_pose_unsorted.name[i]);
					if (found != controlled_joints.end()) {
						// we control this joint so copy it
						ref_pose.position[found->second.index] = ref_pose_unsorted.position[i];
						if (ref_pose_unsorted.velocity.size()) ref_pose.velocity[found->second.index] = ref_pose_unsorted.velocity[i];
						else ref_pose.velocity[found->second.index] = 0.0;
						//TODO effort support
					}
				}
			}
		}
	}

	if (log(DEBUG)) {
		double t = os::TimeService::Instance()->secondsSince(activation_timestamp);
		log() << " t = " << t << " actual: " << actual_pose << endlog();
		log() << " t = " << t << " ref: " << ref_pose << endlog();
	}
	// perform trajectory smoothing
	// actual_pose represents state and is modified in place
	if (filter && filter->update(actual_pose, ref_pose, actual_pose)){
		// now refrence in actual_pose
		out_joints_port.write(actual_pose);

		if (log(DEBUG)) {
			double t = os::TimeService::Instance()->secondsSince(activation_timestamp);
			log() << " t = " << t << " filtered: " << actual_pose << endlog();
		}
	}
	else {
		// publish without smoothing
		out_joints_port.write(ref_pose);
	}

	// publish supports
	// if (publish_supports) out_supports_port.write(supports);
}

/* 
 * Preempts the controllers and releases its resources.
 */
void FollowJointState::stopHook_impl() 
{
	// deinitialization
	// release all resources
	log(INFO) << "FollowJointState is stopped!" << endlog();
}

void FollowJointState::cleanupHook_impl() 
{
	// free memory, close files and etc
	log(INFO) << "FollowJointState cleaning up !" << endlog();
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(FollowJointState)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::controller::FollowJointState)
