#include "follow_pose-component.hpp"

#include <rtt/Component.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <ros/time.h>

#include <sweetie_bot_orocos_misc/get_subservice_by_type.hpp>
#include <sweetie_bot_orocos_misc/message_checks.hpp>
#include <sweetie_bot_orocos_misc/math.hpp>

using namespace RTT;


inline std::ostream& operator<<(std::ostream& s, const KDL::Vector& v) 
{
	s << "[" << v.x() << " " << v.y() << " " << v.z() << " ]";
	return s;
}

inline std::ostream& operator<<(std::ostream& s, const KDL::Twist& v) 
{
	s << "[ rot = " << v.rot << ", vel = " << v.vel << " ]";
	return s;
}
inline std::ostream& operator<<(std::ostream& s, const KDL::Rotation& R) 
{
	s << std::endl;
	s << R(0,0) << " " << R(0,1) << " " << R(0,2) << std::endl;
	s << R(1,0) << " " << R(1,1) << " " << R(1,2) << std::endl;
	s << R(2,0) << " " << R(2,1) << " " << R(2,2) << std::endl;
	return s;
}

namespace sweetie_bot {
namespace motion {
namespace controller {

FollowPose::FollowPose(std::string const& name)  : 
	SimpleControllerBase(name),
	poseToJointStatePublish("poseToJointStatePublish", this->engine()) // operation caller
{
	// ports input 
	this->addPort("in_limbs", in_limbs_port)
		.doc("Robot limbs postions. They are used only to setup anchors on component start.");
	this->addPort("in_base", in_base_port)
		.doc("Robot base link pose in world frame.");
	this->addPort("in_pose_ref", in_pose_ref_port)
		.doc("Target kineamtic cahin pose in world frame.");
	// ports input 
	this->addPort("out_limbs_ref", out_limbs_ref_port)
		.doc("Limb next position to achive target given current robot state. It is computed by component each control cycle. ");
	this->addPort("out_supports", out_supports_port)
		.doc("Active contact list.");

	// properties
	// operations: provided
	// operations: required
	this->requires()->addOperationCaller(poseToJointStatePublish); // kinematics service

	// Service: reqires
	robot_model = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));

	log(INFO) << "FollowPose is constructed!" << endlog();
}

bool FollowPose::configureHook_impl()
{
	// INITIALIZATION
	// check if filter present
	filter = getSubServiceByType<filter::FilterRigidBodyStateInterface>(this->provides().get());
	/*if (!filter) {
		log(ERROR) << "RigidBodyState filter service is not loaded." << endlog();
		return false;
	}*/

	// check if RobotModel Service presents
	if (!robot_model->ready() || !robot_model->isConfigured()) {
		log(ERROR) << "RobotModel service is not ready." << endlog();
		return false;
	}

	// check poseToJointStatePublish caller
	if (poseToJointStatePublish.ready()) {
		log(INFO) << "poseToJointStatePublish operation is connected. Syncronous IK interface is used." << endlog();
	}
	else {
		log(INFO) << "poseToJointStatePublish operation is disconnected. Asyncronous IK interface via port is used." << endlog();
	}

	// allocate memory
	limb_next.name.resize(1);
	limb_next.frame.resize(1);
	limb_next.twist.resize(1);
	limb_ref.name.clear();
	limb_ref.frame.resize(1);
	limb_ref.twist.resize(1);
	supports.name.resize(1);
	supports.contact.assign(1, "");
	supports.support.assign(1, 0.0);

	// data samples
	out_supports_port.setDataSample(supports);
	out_limbs_ref_port.setDataSample(limb_next);

	log(INFO) << "FollowPose is configured !" << endlog();
	return true;
}

bool FollowPose::startHook_impl()
{

	// data samples
	in_base_port.getDataSample(base);
	in_limbs_port.getDataSample(limbs);

	// now update hook will be periodically executed
	log(INFO) << "FollowPose is started !" << endlog();
	return true;
}


bool FollowPose::processResourceSet_impl(const std::vector<std::string>& set_operational_goal_chains, std::vector<std::string>& resources_to_request)
{
	// check if list of controlled chains contains only one element
	if (set_operational_goal_chains.size() != 1) {
		log(ERROR) << "FollowPose can control only one kinematics chain." << endlog();
		return false;
	}
	resources_to_request = robot_model->getChainsGroups(set_operational_goal_chains);
	return true;
}

bool FollowPose::resourceChangedHook_impl(const std::vector<std::string>& set_operational_goal_chains, const std::vector<std::string>& requested_resources)
{
	// check in resource available
	// we do not have to test size: checkResourceSet_impl was called before
	if (!resource_client->hasResource(requested_resources[0])) return false;

	const std::string& chain_name = set_operational_goal_chains[0];
	chain_index = robot_model->getChainIndex(chain_name);
	if (chain_index < 0) {
		log(ERROR) << "Kinematic chain " << chain_name << " is not registered in robot model." << endlog();
		return false;
	}

	// set chain name in output messages
	limb_next.name[0] = chain_name;
	supports.name[0] = chain_name;

	// set reference pose to current limb pose
	// check if limb data is available
	if (in_limbs_port.read(limbs, true) == NoData || !isValidRigidBodyStateNameFrame(limbs) || limbs.name.size() <= chain_index || limbs.name[chain_index] != chain_name) {
		log(ERROR) << "Limbs pose is unknown (in_limbs_port) or incorrect. Unable to start." << endlog();
		return false;
	}
	// check base pose data
	if (in_base_port.read(base, true) == NoData || !isValidRigidBodyStateNameFrame(base, 1)) {
		log(ERROR) << "Base pose is unknown (in_base_port) or incorrect." << endlog();
		return false;
	}
	// limb pose in world frame
	limb_ref.frame[0] = base.frame[0]*limbs.frame[chain_index];
	// reset filter
	if (filter && !filter->reset(limb_ref, period)) {
		log(ERROR) << "RigidBodyState filter reset has failed." << endlog();
		return false;
	}

	log(INFO) << "FollowPose controlled chain: " << set_operational_goal_chains[0] << " chain_index = " << chain_index << endlog();
	return true;
}


void FollowPose::updateHook_impl()
{
	// publish support state
	out_supports_port.write(supports);

	// get current base pose and speed
	if (in_base_port.read(base, false) == NewData) {
		if (!isValidRigidBodyStateNameFrameTwist(base, 1) || base.name[0] != "base_link") {
			log(WARN) << "Incorrect base_link pose is received. Skip iteration." << endlog();
			return;
		}
	}
	if (in_limbs_port.read(limbs, false) == NewData) {
		if (!isValidRigidBodyStateNameFrameTwist(limbs) || limbs.name.size() <= chain_index) {
			log(WARN) << "Incorrect limb poses message is received. Skip iteration." << endlog();
			return;
		}
	}

	// get reference pose
	{
		geometry_msgs::PoseStamped pose_stamped;
		if (in_pose_ref_port.readNewest(pose_stamped, false) == NewData) {
			// convert to KDL 
			normalizeQuaternionMsg(pose_stamped.pose.orientation);
			tf::poseMsgToKDL(pose_stamped.pose, limb_ref.frame[0]);
		}
	}
	
	// convert to world frame
	// TODO perform calculation in base_link frame: add to filter reference speed handling
	limb_next.frame[0] = base.frame[0]*limbs.frame[chain_index];
	limb_next.twist[0] = base.twist[0] + base.frame[0]*limbs.twist[chain_index];

	if (filter) {
		// apply filter to calculate next pose
		filter->update(limb_next, limb_ref, limb_next);
		// return back to base_link frame and correct speed
		limb_next.frame[0] = base.frame[0].Inverse()*limb_next.frame[0];
		limb_next.twist[0] = base.frame[0].Inverse(limb_next.twist[0] - base.twist[0]);
	}
	else {
		// move immediately to ref pose
		limb_next.frame[0] = base.frame[0].Inverse()*limb_ref.frame[0];
		limb_next.twist[0] = base.frame[0].Inverse(-base.twist[0]);
	}
	limb_next.header.stamp = ros::Time::now();

	bool ik_success = true;
	// pass result to kinematics
	if (poseToJointStatePublish.ready()) {
		// syncronous interface
		ik_success = poseToJointStatePublish(limb_next);
	} 
	else {
		// async interface
		out_limbs_ref_port.write(limb_next);
	}

	if (log(DEBUG)) {
		log() << "ik_success = " << ik_success << " sync_mode = " << poseToJointStatePublish.ready() << " chain_index = "  << chain_index << endlog();
	}
}

/* 
 * Preempts the controllers and releases its resources.
 */
void FollowPose::stopHook_impl() 
{
	// deinitialization
	// release all resources
	log(INFO) << "FollowPose is stopped!" << endlog();
}

void FollowPose::cleanupHook_impl() 
{
	// free memory, close files and etc
	log(INFO) << "FollowPose cleaning up !" << endlog();
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(FollowPose)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::controller::FollowPose)
