#include "look_at_joints-component.hpp"

#include <rtt/Component.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <ros/time.h>

#include <sweetie_bot_orocos_misc/get_subservice_by_type.hpp>
#include <sweetie_bot_orocos_misc/joint_state_check.hpp>
#include <sweetie_bot_orocos_misc/message_checks.hpp>
#include <sweetie_bot_orocos_misc/math.hpp>

using namespace RTT;


static inline std::ostream& operator<<(std::ostream& s, const KDL::Vector& v) 
{
	s << "[" << v.x() << " " << v.y() << " " << v.z() << " ]";
	return s;
}

static inline std::ostream& operator<<(std::ostream& s, const KDL::Twist& v) 
{
	s << "[ rot = " << v.rot << ", vel = " << v.vel << " ]";
	return s;
}
static inline std::ostream& operator<<(std::ostream& s, const KDL::Rotation& R) 
{
	s << std::endl;
	s << R(0,0) << " " << R(0,1) << " " << R(0,2) << std::endl;
	s << R(1,0) << " " << R(1,1) << " " << R(1,2) << std::endl;
	s << R(2,0) << " " << R(2,1) << " " << R(2,2) << std::endl;
	return s;
}

static std::ostream& operator<<(std::ostream& s, const std::vector<std::string>& strings) 
{
	s << "[ ";
	for(auto it = strings.begin(); it != strings.end(); it++) s << *it << ", ";
	s << " ]";
	return s;
}

static std::ostream& operator<<(std::ostream& s, const std::vector<int>& ints) 
{
	s << "[ ";
	for(auto it = ints.begin(); it != ints.end(); it++) s << *it << ", ";
	s << " ]";
	return s;
}


namespace sweetie_bot {
namespace motion {
namespace controller {

LookAtJoints::LookAtJoints(std::string const& name)  : 
	SimpleControllerBase(name),
	poseToJointState("poseToJointState", this->engine()) // operation caller
{
	// ports input 
	this->addPort("in_joints_sorted", in_joints_port)
		.doc("Robot pose in joint space.");
	this->addPort("in_limbs", in_limbs_port)
		.doc("Robot limbs positions in base_link frame.");
	this->addPort("in_base", in_base_port)
		.doc("Robot base_link pose in world frame.");
	this->addPort("in_pose_ref", in_pose_ref_port)
		.doc("Target pose in world frame. Robot should look in derection specified by its position.");
	// ports output 
	this->addPort("out_supports", out_supports_port)
		.doc("Active contact list. Controlled kineamtic chain is assumed to be free.");
	this->addPort("out_joints_ref", out_joints_ref_port)
		.doc("Reference joints state calculated bu component. It includes head position and also yaw and pitch for eyes animation if `pitch_yaw_joints` is set.");

	// properties
	this->addProperty("pitch_yaw_joints", pitch_yaw_joints)
		.doc("Names of pitch and yaw joints which represents \"eyes\" position. Must be empty or contain two elements. ");
	// operations: provided
	// operations: required
	this->requires()->addOperationCaller(poseToJointState); // kinematics service

	// Service: requires
	robot_model = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));

	log(INFO) << "LookAtJoints is constructed!" << endlog();
}

bool LookAtJoints::configureHook_impl()
{
	// INITIALIZATION
	// check if filter present
	filter = getSubServiceByType<filter::FilterJointStateInterface>(this->provides().get());
	if (!filter) {
		log(WARN) << "Filter is not present!" << endlog();
	}

	// check if RobotModel Service presents
	if (!robot_model->ready() || !robot_model->isConfigured()) {
		log(ERROR) << "RobotModel service is not ready." << endlog();
		return false;
	}

	// check poseToJointState caller
	if (!poseToJointState.ready()) {
		log(ERROR) << "poseToJointState() operation is disconnected. " << endlog();
		return false;
	}

	// check eyes animation pitch and yaw property
	if (pitch_yaw_joints.size() > 0) {
		if (pitch_yaw_joints.size() != 2) {
			log(ERROR) << "'pitch_yaw_joints' must contains zero or two elements." << endlog();
			return false;
		}
		if (robot_model->getJointIndex(pitch_yaw_joints[0]) < 0 || robot_model->getJointIndex(pitch_yaw_joints[1]) < 0) {
			log(ERROR) << "Unknown joint in 'pitch_yaw_joints' property." << endlog();
			return false;
		}
	}

	// allocate memory
	// cartesian state
	limb_ref.name.resize(1);
	limb_ref.frame.resize(1);
	limb_ref.twist.resize(1);
	supports.name.resize(1);
	supports.contact.assign(1, "");
	supports.support.assign(1, 0.0);

	// data samples
	out_supports_port.setDataSample(supports);

	log(INFO) << "LookAtJoints is configured !" << endlog();
	return true;
}

bool LookAtJoints::startHook_impl()
{

	// data samples
	in_base_port.getDataSample(base);
	in_limbs_port.getDataSample(limbs);

	// now update hook will be periodically executed
	log(INFO) << "LookAtJoints is started !" << endlog();
	return true;
}


bool LookAtJoints::processResourceSet_impl(const std::vector<std::string>& set_operational_goal_chains, std::vector<std::string>& resources_to_request)
{
	// check if list of controlled chains contains only one element
	if (set_operational_goal_chains.size() != 1) {
		log(ERROR) << "LookAtJoints can control only one kinematics chain." << endlog();
		return false;
	}
	// get coresponding group
	resources_to_request = robot_model->getChainsGroups(set_operational_goal_chains);
	// add pitch, yaw joints related groups
	std::vector<std::string> pith_yaw_groups = robot_model->getJointsGroups(pitch_yaw_joints);
	resources_to_request.insert(resources_to_request.end(), pith_yaw_groups.begin(), pith_yaw_groups.end());
	return true;
}

bool LookAtJoints::formJointIndex(const std::vector<string>& joint_list) 
{
	int n_joints = joint_list.size() + pitch_yaw_joints.size();
	// create joints_next buffer
	joints_next.name = joint_list;
	if (pitch_yaw_joints.size() != 0) {
		joints_next.name.push_back(pitch_yaw_joints[0]); 
		joints_next.name.push_back(pitch_yaw_joints[1]); 
	}
	joints_next.position.resize(n_joints);
	joints_next.velocity.resize(n_joints);
	// create joint index
	joint_index.clear();
	joint_index.reserve(n_joints);
	for(std::string joint_name : joints_next.name) {
		int index = robot_model->getJointIndex(joint_name);
		if (index < 0) return false;
		joint_index.push_back(index);
	}
	// reserve memory (it is resized by poseToJointState() call)
	joints_ref.name.reserve(n_joints);
	joints_ref.position.reserve(n_joints);
	joints_ref.velocity.reserve(n_joints);
	return true;
}

bool LookAtJoints::resourceChangedHook_impl(const std::vector<std::string>& set_operational_goal_chains, const std::vector<std::string>& requested_resources)
{
	// check in resource available
	if (!resource_client->hasResources(requested_resources)) return false;

	const std::string& chain_name = set_operational_goal_chains[0];
	chain_index = robot_model->getChainIndex(chain_name);
	if (chain_index < 0) {
		log(ERROR) << "Kinematic chain " << chain_name << " is not registered in robot model." << endlog();
		return false;
	}

	// set chain name in output messages
	limb_ref.name[0] = chain_name;
	supports.name[0] = chain_name;
	
	if (!formJointIndex(robot_model->getChainJoints(chain_name))) {
		log(ERROR) << "Unable get joint index for chain " << chain_name << endlog();
		return false;
	}

	// get initial pose
	if (in_joints_port.read(joints_fullpose, true) != RTT::NoData && isValidJointStatePosVel(joints_fullpose)) {
		// TODO size check
		// copy postion and velocity
		for(int i = 0; i < joint_index.size(); i++) {
			joints_next.position[i] = joints_fullpose.position[joint_index[i]];
			joints_next.velocity[i] = joints_fullpose.velocity[joint_index[i]];
		}
	}
	else {
		log(ERROR) << "Unable to get current pose in joint space." << endlog();
		return false;
	}

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
	// prefered ee pose
	target_position = limbs.frame[chain_index].p;
	// initial target point
	limb_ref.frame[0] = base.frame[0]*limbs.frame[chain_index];
	target_point = limb_ref.frame[0] * KDL::Vector(1.0, 0.0, 0.0);

	// reset filter
	if (filter && !filter->reset(joints_next, period)) {
		log(ERROR) << "JointState filter reset has failed." << endlog();
		return false;
	}

	log(INFO) << "LookAtJoints controlled chain: " << set_operational_goal_chains[0] << " chain_index = " << chain_index << endlog();
	return true;
}


void LookAtJoints::updateHook_impl()
{
	// publish support state
	out_supports_port.write(supports);

	// get current pose in joint space
	if (in_joints_port.read(joints_fullpose, false) == NewData) {
		// copy controlled joint from actual pose
		if (isValidJointStatePos(joints_fullpose)) {
			for(int i = 0; i < joint_index.size(); i++) {
				joints_next.position[i] = joints_fullpose.position[ joint_index[i] ];
				if (joints_fullpose.velocity.size()) joints_next.velocity[i] = joints_fullpose.velocity[ joint_index[i] ];
				else joints_next.velocity[i] = 0;
			}
		}
	}

	// get current base pose and speed
	if (in_base_port.read(base, false) == NewData) {
		if (!isValidRigidBodyStateNameFrame(base, 1) || base.name[0] != "base_link") {
			log(WARN) << "Incorrect base_link pose is received. Skip iteration." << endlog();
			return;
		}
	}
	if (in_limbs_port.read(limbs, false) == NewData) {
		if (!isValidRigidBodyStateNameFrame(limbs) || limbs.name.size() <= chain_index) {
			log(WARN) << "Incorrect limb poses message is received. Skip iteration." << endlog();
			return;
		}
	}

	// get reference pose
	geometry_msgs::PoseStamped pose_stamped;
	if (in_pose_ref_port.readNewest(pose_stamped, false) == NewData) {
		// convert to KDL 
		tf::pointMsgToKDL(pose_stamped.pose.position, target_point);
	}

	//
	// calculate desired pose (in world frame)
	//
	KDL::Vector x_axis, y_axis, z_axis;
	const KDL::Vector up(0.0, 0.0, 1.0);
	// convert end effector pose to world frame
	KDL::Frame ee_frame  = base.frame[0] * limbs.frame[chain_index];
	// calculate direction to the target point and use it as x axis
	x_axis = target_point - ee_frame.p;
	x_axis.Normalize();
	// calculate y axis candidate
	y_axis = x_axis * up;
	// Normalize y_axis. If y_axis is equal to zero KDL which means that x points up or down
	// this function returns Vector(1,0,0) which is good enough,
	// TODO minimize rotation angle in this case
	y_axis.Normalize(); 
	// calculate z_axis
	z_axis = x_axis * y_axis;
	// check if z points up, flip it if necessary
	if (KDL::dot(z_axis, up) < 0.0) {
		z_axis = -z_axis;
		y_axis = -y_axis;
	}
	// form desired pose in base_link frame
	limb_ref.frame[0].M = base.frame[0].M.Inverse() * KDL::Rotation(x_axis, y_axis, z_axis);
	limb_ref.frame[0].p = target_position;

	//
	// invoke inverse kinematics
	//

	bool ik_success = true;
	// pass result to kinematics
	ik_success = poseToJointState(limb_ref, joints_ref);
	if (log(DEBUG)) {
		log() << "ik_success = " << ik_success << " sync_mode = " << poseToJointState.ready() << " chain_index = "  << chain_index << endlog();
	}
	if (!ik_success) {
		// use next value instead of IK result
		joints_ref.position.clear();
		joints_ref.position.insert(joints_ref.position.end(), joints_next.position.begin(), joints_next.position.end() - pitch_yaw_joints.size());
		joints_ref.velocity.assign(joints_ref.name.size(), 0.0);
	}
	else {
		// check if result is sane
		if (joints_ref.name.size() + pitch_yaw_joints.size() != joint_index.size()) {
			log(WARN) << "Skip iteration. Invalid joint index: " << joints_ref.name << " + " << pitch_yaw_joints << " vs " << joint_index << endlog();
			formJointIndex(joints_ref.name);
			return;
		}
	}

	//
	// calculate pitch and yaw for eyes if necessary
	//
	if (pitch_yaw_joints.size() > 0) {
		// add joints 
		joints_ref.name.push_back(pitch_yaw_joints[0]);
		joints_ref.name.push_back(pitch_yaw_joints[1]);
		// calculate pose
		KDL::Vector tpos = target_point; // target position in base_link frame
		double rxy = std::sqrt(tpos.x()*tpos.x() + tpos.y()*tpos.y());
		joints_ref.position.push_back( std::atan2(tpos.z(), rxy) ); // pitch
		joints_ref.position.push_back( -std::atan2(tpos.y(), tpos.x()) ); // yaw
		joints_ref.velocity.push_back(0.0); 
		joints_ref.velocity.push_back(0.0);
	}

	//
	// apply joint filter
	//
	if (filter) {
		// apply filter to calculate next pose and publish result
		filter->update(joints_next, joints_ref, joints_next);
		joints_next.header.stamp = ros::Time::now();
		out_joints_ref_port.write(joints_next);
	}
	else {
		// publish reference pose
		joints_ref.header.stamp = ros::Time::now();
		out_joints_ref_port.write(joints_ref);
	}
}

/* 
 * Preempts the controllers and releases its resources.
 */
void LookAtJoints::stopHook_impl() 
{
	// deinitialization
	// release all resources
	log(INFO) << "LookAtJoints is stopped!" << endlog();
}

void LookAtJoints::cleanupHook_impl() 
{
	// free memory, close files and etc
	log(INFO) << "LookAtJoints cleaning up !" << endlog();
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

ORO_CREATE_COMPONENT(sweetie_bot::motion::controller::LookAtJoints)
