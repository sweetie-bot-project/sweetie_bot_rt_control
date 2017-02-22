#include "follow_joint_state-component.hpp"

#include <rtt/Component.hpp>

using namespace RTT;
using namespace std;

namespace sweetie_bot {
namespace motion {
namespace controller {

//TODO move somewhere

/**
 * @brief Find OROCOS subservice which implements given interface.
 * Find OROCOS subservice which implements given interface ServiceInterface.
 * @param service Pointer to parent service.
 * @return Pointer to found subservice or zero if nothing found.
 **/
template<class ServiceInterface> ServiceInterface * getSubServiceByInterface(Service * service) 
{
	if (!service) return nullptr;

	ServiceInterface * found_service;
	Service::ProviderNames subservices;

	subservices = service->getProviderNames();
	for(Service::ProviderNames::const_iterator name = subservices.begin(); name != subservices.end(); name++) {
        found_service = dynamic_cast<ServiceInterface*>(service->getService(*name).get());
		if (found_service) return found_service;
	}
	return nullptr;
}

std::ostream& operator<<(std::ostream& s, const std::vector<std::string>& strings) 
{
	s << "[ ";
	for(auto it = strings.begin(); it != strings.end(); it++) s << *it << ", ";
	s << " ]";
	return s;
}

bool isValidJointState(const sensor_msgs::JointState& msg, int sz = -1)
{
	if (msg.name.size() != 0) {
		if (sz < 0) sz = msg.name.size();
		else if (sz != msg.name.size()) return false;
	}
	if (msg.position.size() != 0) {
		if (sz < 0) sz = msg.position.size();
		else if (sz != msg.position.size()) return false;
	}
	if (msg.velocity.size() != 0) {
		if (sz < 0) sz = msg.velocity.size();
		else if (sz != msg.velocity.size()) return false;
	}
	if (msg.effort.size() != 0) {
		if (sz < 0) sz = msg.effort.size();
		else if (sz != msg.effort.size()) return false;
	}
	return true;
}



FollowJointState::FollowJointState(std::string const& name)  : 
	TaskContext(name, RTT::base::TaskCore::PreOperational),
	robot_model(this),
	log(logger::getDefaultCategory("sweetie.motion") + ".controller." + name)
{
	this->provides()->doc("Feedforward JointState reference from high-level to agregator.");

	// ports
	// PORTS: input
	this->addEventPort("sync", sync_port)
		.doc("Timer syncronization event. This event triggers controller execution cycle.");

	this->addPort("in_joints_sorted", in_joints_port).
		doc("Full sorted actual robot pose (from sensors or from agregator).");

	this->addPort("in_joints_ref", in_joints_ref_port).
		doc("Desired joint positions.");

	// PORTS: output
	this->addPort("out_joints_ref_fixed", in_joints_ref_port).
		doc("Reference joint positions for agregator.");
		OutputPort<sensor_msgs::JointState> out_joints_port;

	// properties
	this->addProperty("controlled_chains", controlled_chains).
		doc("List of controlled joint groups (kinematic chains, resources).");

	// operations: provided
	this->addOperation("rosSetOperational", &FollowJointState::rosSetOperational, this)
		.doc("ROS compatible start/stop operation (std_srvs::SetBool).");

	// other actions
	log(INFO) << "FollowJointState is constructed!" << endlog();
}

bool FollowJointState::configureHook()
{
	// INITIALIZATION
	// check if ResourceClient Service presents
	resource_client = getSubServiceByInterface<ResourceClientInterface>(this->provides().get());
	if (!resource_client) {
		log(ERROR) << "ResourceClient plugin is not loaded." << endlog();
		return false;
	}
	// check if RobotModel Service presents
	if (!robot_model.ready()) {
		log(ERROR) << "RobotModel service is not ready." << endlog();
		return false;
	}
	// build joints index 
	controlled_joints.clear();
	ref_pose.name.clear();

	vector<string> joint_names;
	for(auto chain = controlled_chains.begin(); chain != controlled_chains.end(); chain++) {
		joint_names = robot_model.listJoints(*chain);
		if (joint_names.size() == 0) {
			log(ERROR) << "Empty joint group `" << *chain << "`." << endlog();
			return false;
		}
		for(auto joint = joint_names.begin(); joint != joint_names.end(); joint++) {
			// add joint to index
			controlled_joints.insert(std::make_pair(*joint, JointIndex(robot_model.getJointIndex(*joint), controlled_joints.size())));
			ref_pose.name.push_back(*joint);
		}
	}
	if (log(INFO)) {
		// TODO overload something?
		log() << "Controlled chains: [ "; 
		for(auto it = controlled_chains.begin(); it != controlled_chains.end(); it++) log() << *it << ", ";
		log() << " ]." << endlog();
		log(INFO) << "Controlled joints: [ "; 
		for(auto it = ref_pose.name.begin(); it != ref_pose.name.end(); it++) log() << *it << ", ";
		log() << " ]." << endlog();
	}

	// allocate memory
	unsigned int sz = controlled_joints.size();
	ref_pose.position.assign(sz, 0);
	ref_pose.velocity.assign(sz, 0);
	ref_pose.effort.assign(sz, 0);
	actual_pose = ref_pose;

	// set ports data samples
	out_joints_port.setDataSample(ref_pose);

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
bool FollowJointState::startHook()
{
	in_joints_port.getDataSample(actual_fullpose);
	in_joints_ref_port.getDataSample(ref_pose_unsorted);
	// request resources
	resource_client->resourceChangeRequest(controlled_chains);
	// clear sync port buffer
	RTT::os::Timer::TimerId timer_id;
	sync_port.readNewest(timer_id);
	// now update hook will be periodically executed
	log(INFO) << "FollowJointState is started !" << endlog();
	return true;
}

void FollowJointState::updateHook()
{
	// let resource_client do it stuff
	resource_client->step();	

	// syncronize with sync messages
	{
		RTT::os::Timer::TimerId unused;
		if (sync_port.read(unused) != NewData) return;
	}

	// main operational 
	int state = resource_client->getState();
	if (state & ResourceClient::OPERATIONAL) {
		// read ports
		in_joints_ref_port.read(ref_pose_unsorted, false);
		in_joints_port.read(actual_fullpose, false);

		// copy controlled joint to actual_pose
		if (isValidJointState(actual_fullpose)) {
			for(JointIndexes::const_iterator it = controlled_joints.begin(); it != controlled_joints.end(); it++) {
				if (actual_fullpose.position.size()) 
					actual_pose.position[it->second.index] = actual_fullpose.position[it->second.index_fullpose];
				if (actual_fullpose.velocity.size()) 
					actual_pose.velocity[it->second.index] = actual_fullpose.velocity[it->second.index_fullpose];
				//TODO effort support
			}
		}
		// fill ref_pose with default vaules
		ref_pose.position = actual_pose.position;
		ref_pose.velocity = actual_pose.velocity;

		// copy controlled joint to ref_pose
		if (isValidJointState(ref_pose_unsorted)) {
			for(int i = 0; i < ref_pose_unsorted.name.size(); i++) {
				JointIndexes::const_iterator found = controlled_joints.find(ref_pose_unsorted.name[i]);
				if (found != controlled_joints.end()) {
					// we control this joint so copy it
					if (ref_pose_unsorted.position.size()) 
						ref_pose.position[found->second.index] = ref_pose_unsorted.position[i];
					if (ref_pose_unsorted.velocity.size()) 
						ref_pose.velocity[found->second.index] = ref_pose_unsorted.velocity[i];
					//TODO effort support
				}
			}
		}

		// perform trajectory smoothing
		// TODO smoothing
		
		// publish new reference position
		out_joints_port.write(ref_pose);
	}
	else if (state == ResourceClient::NONOPERATIONAL) {
		log(INFO) << "FollowJointState is exiting  operational state !" << endlog();
		this->stop();
	}
}

/* 
 * Preempts the controllers and releases its resources.
 */
void FollowJointState::stopHook() 
{
	// user calls stop() directly 
	if (resource_client->isOperational()) resource_client->stopOperational();
	// deinitialization
	// release all resources
	log(INFO) << "FollowJointState is stopped!" << endlog();
}

void FollowJointState::cleanupHook() 
{
	// free memory, close files and etc
	log(INFO) << "FollowJointState cleaning up !" << endlog();
}

/**
 * ROS comaptible start/stop operation.
 */
bool FollowJointState::rosSetOperational(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
	if (req.data) {
		resp.success = start();
		resp.message = "start() is called.";
	}
	else {
		stop();
		resp.success = true;
		resp.message = "stop() is called.";
	}
	return true;
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
