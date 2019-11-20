#include "look_at-component.hpp"

#include <rtt/Component.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <ros/time.h>

#include <sweetie_bot_orocos_misc/get_subservice_by_type.hpp>
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

namespace sweetie_bot {
namespace motion {
namespace controller {

LookAt::LookAt(std::string const& name)  : 
	SimpleControllerBase(name),
	poseToJointStatePublish("poseToJointStatePublish", this->engine()) // operation caller
{
	// ports input 
	this->addPort("in_limbs", in_limbs_port)
		.doc("Robot limbs positions in base_link frame.");
	this->addPort("in_base", in_base_port)
		.doc("Robot base_link pose in world frame.");
	this->addPort("in_pose_ref", in_pose_ref_port)
		.doc("Target pose in world frame. Robot should look in derection specified by its position.");
	// ports input 
	this->addPort("out_limbs_ref", out_limbs_ref_port)
		.doc("Limb reference position. It is updated by component each control cycle to change limb pose that its last link x-axis points to  in_pose_ref position and z-axis is oriented upward. ");
	this->addPort("out_supports", out_supports_port)
		.doc("Active contact list. Controlled kineamtic chain is assumed to be free.");
	this->addPort("out_joints_ref", out_joints_ref_port)
		.doc("Reference joints state for eyes animation. It publish pitch and yaw of target pose in the frame of the last link of controlled kineamtic chain. Published joints' name is set via `pitch_yaw_joints`.");

	// properties
	this->addProperty("pitch_yaw_joints", pitch_yaw_joints)
		.doc("Names of pitch and yaw joints which represents \"eyes\" position. Must be empty or contain two elements. ");
	// operations: provided
	// operations: required
	this->requires()->addOperationCaller(poseToJointStatePublish); // kinematics service

	// Service: requires
	robot_model = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));

	log(INFO) << "LookAt is constructed!" << endlog();
}

bool LookAt::configureHook_impl()
{
	// INITIALIZATION
	// check if filter present
	filter_pose = getSubServiceByType<filter::FilterRigidBodyStateInterface>(this->provides().get());
	filter_joints = getSubServiceByType<filter::FilterJointStateInterface>(this->provides().get());

	// check if RobotModel Service presents
	if (!robot_model->ready() || !robot_model->isConfigured()) {
		log(ERROR) << "RobotModel service is not ready." << endlog();
		return false;
	}

	// check poseToJointStatePublish caller
	if (poseToJointStatePublish.ready()) {
		log(INFO) << "poseToJointStatePublish operation is connected. Synchronous IK interface is used." << endlog();
	}
	else {
		log(INFO) << "poseToJointStatePublish operation is disconnected. Asynchronous IK interface via port is used." << endlog();
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
	limb_next.name.resize(1);
	limb_next.frame.resize(1);
	limb_next.twist.resize(1);
	limb_ref.name.clear();
	limb_ref.frame.resize(1);
	limb_ref.twist.resize(1);
	supports.name.resize(1);
	supports.contact.assign(1, "");
	supports.support.assign(1, 0.0);
	// joints state
	int n_joints = pitch_yaw_joints.size();
	joints_ref.name = pitch_yaw_joints;
	joints_ref.position.assign(n_joints, 0.0);
	joints_ref.velocity.assign(n_joints, 0.0);
	joints_next = joints_ref;

	// data samples
	out_supports_port.setDataSample(supports);
	out_limbs_ref_port.setDataSample(limb_next);
	out_joints_ref_port.setDataSample(joints_next);

	log(INFO) << "LookAt is configured !" << endlog();
	return true;
}

bool LookAt::startHook_impl()
{

	// data samples
	in_base_port.getDataSample(base);
	in_limbs_port.getDataSample(limbs);

	// now update hook will be periodically executed
	log(INFO) << "LookAt is started !" << endlog();
	return true;
}


bool LookAt::processResourceSet_impl(const std::vector<std::string>& set_operational_goal_chains, std::vector<std::string>& resources_to_request)
{
	// check if list of controlled chains contains only one element
	if (set_operational_goal_chains.size() != 1) {
		log(ERROR) << "LookAt can control only one kinematics chain." << endlog();
		return false;
	}
	resources_to_request = robot_model->getChainsGroups(set_operational_goal_chains);
	return true;
}

bool LookAt::resourceChangedHook_impl(const std::vector<std::string>& set_operational_goal_chains, const std::vector<std::string>& requested_resources)
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
	target_point = limb_ref.frame[0] * KDL::Vector(1.0, 0.0, 0.0);
	target_position = limbs.frame[chain_index].p;
	// joints to zero
	joints_ref.position.assign(joints_ref.position.size(), 0.0);

	// reset filter
	if (filter_pose && !filter_pose->reset(limb_ref, period)) {
		log(ERROR) << "RigidBodyState filter reset has failed." << endlog();
		return false;
	}
	if (filter_joints && !filter_joints->reset(joints_ref, period)) {
		log(ERROR) << "RigidBodyState filter reset has failed." << endlog();
		return false;
	}

	log(INFO) << "LookAt controlled chain: " << set_operational_goal_chains[0] << " chain_index = " << chain_index << endlog();
	return true;
}


void LookAt::updateHook_impl()
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
	geometry_msgs::PoseStamped pose_stamped;
	if (in_pose_ref_port.readNewest(pose_stamped, false) == NewData) {
		// convert to KDL 
		tf::pointMsgToKDL(pose_stamped.pose.position, target_point);
	}
	
	// convert limb pose to world frame
	limb_next.frame[0] = base.frame[0] * limbs.frame[chain_index];
	limb_next.twist[0] = base.twist[0] + base.frame[0]*limbs.twist[chain_index];

	//
	// calculate desired pose (in world frame)
	//
	KDL::Vector x_axis, y_axis, z_axis;
	const KDL::Vector up(0.0, 0.0, 1.0);
	// current end effector frame relative to world
	KDL::Frame& ee_frame = limb_next.frame[0]; 
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
	// form desired frame
	limb_ref.frame[0].M = KDL::Rotation(x_axis, y_axis, z_axis);
	limb_ref.frame[0].p = base.frame[0] * target_position;

	// 
	// apply filter
	//
	if (filter_pose) {
		// apply filter to calculate next pose
		filter_pose->update(limb_next, limb_ref, limb_next);
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

	//
	// invoke inverse kinematics
	//

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

	//
	// calculate pitch and yaw for eyes if necessary
	//
	if (joints_ref.name.size() > 0) {
		KDL::Vector tpos = limb_next.frame[0].Inverse(base.frame[0].Inverse(target_point)); // target position in base_link frame
		double rxy = std::sqrt(tpos.x()*tpos.x() + tpos.y()*tpos.y());
		joints_ref.position[0] = std::atan2(tpos.z(), rxy); // pitch
		joints_ref.position[1] = -std::atan2(tpos.y(), tpos.x()); // yaw
		// apply joint filter
		if (filter_joints) {
			// apply filter to calculate next pose and publish result
			filter_joints->update(joints_next, joints_ref, joints_next);
			joints_next.header.stamp = ros::Time::now();
			out_joints_ref_port.write(joints_next);
		}
		else {
			// publish reference pose
			joints_ref.header.stamp = ros::Time::now();
			out_joints_ref_port.write(joints_ref);
		}
	}
}

/* 
 * Preempts the controllers and releases its resources.
 */
void LookAt::stopHook_impl() 
{
	// deinitialization
	// release all resources
	log(INFO) << "LookAt is stopped!" << endlog();
}

void LookAt::cleanupHook_impl() 
{
	// free memory, close files and etc
	log(INFO) << "LookAt cleaning up !" << endlog();
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

ORO_CREATE_COMPONENT(sweetie_bot::motion::controller::LookAt)
