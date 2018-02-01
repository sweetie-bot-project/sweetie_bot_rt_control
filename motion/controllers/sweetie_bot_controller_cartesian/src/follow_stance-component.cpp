#include "follow_stance-component.hpp"

#include <rtt/Component.hpp>
#include <kdl_conversions/kdl_msg.h>

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

FollowStance::FollowStance(std::string const& name)  : 
	TaskContext(name, RTT::base::TaskCore::PreOperational),
	log(logger::categoryFromComponentName(name)),
	poseToJointStatePublish("poseToJointStatePublish", this->engine()) // operation caller
{
	// ports
	this->addPort("in_limbs", in_limbs_port)
		.doc("Robot limbs postions. They are used only to setup anchors on component start.");
	this->addPort("in_base", in_base_port)
		.doc("Robot base link pose in world frame.");
	this->addPort("in_base_ref", in_base_ref_port)
		.doc("Target robot base link pose.");
	this->addPort("out_base_ref", out_base_ref_port)
		.doc("Base next position to achive target given current robot state. It is computed by component each control cycle.");
	this->addPort("out_limbs_ref", out_limbs_ref_port)
		.doc("Limbs next position to achive target given current robot state. It is computed by component each control cycle. ");
	this->addPort("out_supports", out_supports_port)
		.doc("Active contact list.");
	this->addEventPort("sync", sync_port)
		.doc("Timer syncronization port.");

	// properties
	std::vector<std::string> legs = { "leg1", "leg2", "leg3", "leg4" };
	this->addProperty("support_legs", support_legs).
		doc("Robot kinematic chains which are in contact.").
		set(legs);
	this->addProperty("period", period)
		.doc("Discretization period (s).");
	this->addProperty("Kp", Kp)
		.doc("PD regulator coefficient.")
		.set(1);
	this->addProperty("Kd", Kv)
		.doc("PD regulator coefficient.")
		.set(2);
	this->addProperty("base_pose_feedback", base_pose_feedback)
		.doc("If it is false component ignores in_base_port after receving initial pose. Reference trajectory is calculated without any feedback. In most cases this way is more robust. "
				"To comform kinematic constraints inverse kinematics should be connected syncronously via poseToJointStatePublish operation. ")
		.set(false);
	// operations: provided
	this->addOperation("rosSetOperational", &FollowStance::rosSetOperational, this)
		.doc("ROS compatible start/stop operation (std_srvs::SetBool).");
	// operations: required
	this->requires()->addOperationCaller(poseToJointStatePublish); // kinematics service

	// Service: reqires
	robot_model = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));

	log(INFO) << "FollowStance is constructed!" << endlog();
}

bool FollowStance::resourceChangedHook()
{
	return resource_client->hasResources(support_legs);
}

bool FollowStance::setupSupports(const vector<string>& support_legs)
{
	// check if limb data is available
	if (in_limbs_port.read(limbs, true) == NoData) {
		log(ERROR) << "Limbs pose is unknown (in_limbs_port). Unable setup anchors." << endlog();
		return false;
	}
	if (!isValidRigidBodyStateNameFrame(limbs)) {
		log(ERROR) << "Incorrect message on  in_limbs_port. Unable setup anchors." << endlog();
		return false;
	}
	// check base pose data
	if (in_base_port.read(base, true) == NoData) {
		log(ERROR) << "Base pose is unknown (in_base_port). Unable setup anchors." << endlog();
		return false;
	}
	if (!isValidRigidBodyStateNameFrame(base, 1)) {
		log(ERROR) << "Incorrect message on in_base_port. Unable setup anchors." << endlog();
		return false;
	}

	// contact list
	supports.name = support_legs;
	supports.support.assign(support_legs.size(), 1.0);

	support_leg_anchors.clear();
	supports.contact.clear();
	for(  const std::string& leg : support_legs ) {
		// add contact
		supports.contact.push_back(robot_model->getChainDefaultContact(leg)); // TODO contact configuration
		if (supports.contact.back() == "") {
			log(ERROR) << "No contact information for chain `" << leg << "`." << endlog();
			return false;
		}
		// add anchor
		auto it = std::find(limbs.name.begin(), limbs.name.end(), leg);
		if (it == limbs.name.end()) {
			log(ERROR) << "No information about limb `" << leg << "` pose on in_limbs_port. Unable setup anchors." << endlog();
			return false;
		}
		support_leg_anchors.push_back( base.frame[0] * limbs.frame[it - limbs.name.begin()] ); // leg position in world frame
	}

	// limbs will be used as buffer for limb pose publication
	limbs.name = support_legs;
	limbs.frame.resize(support_legs.size());
	limbs.twist.resize(support_legs.size());
	limbs.wrench.clear();

	if (log(INFO)) {
		log() << "Set anchors for chains [ "; 
		for( const string& name :  supports.name ) log() << name << ", ";
		log() << " ]." << endlog();
	}

	// reset target
	base_ref = base.frame[0];

	return true;
}

bool FollowStance::configureHook()
{
	// INITIALIZATION
	// check if ResourceClient Service presents: try to find it amoung loaded services
	resource_client = getSubServiceByType<ResourceClientInterface>(this->provides().get());
	if (!resource_client) {
		log(ERROR) << "ResourceClient plugin is not loaded." << endlog();
		return false;
	}
	resource_client->setResourceChangeHook(boost::bind(&FollowStance::resourceChangedHook, this));

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
		if (!base_pose_feedback) {
			log(WARN) << "Controller is running without any feedback. Connect operation to stop pose integration if kinematic constrains are violated." << endlog();
		}
	}

	// allocate memory
	int n_chains = support_legs.size();
	limbs.name = support_legs;
	limbs.frame.resize(n_chains);
	limbs.twist.resize(n_chains);
	limbs.wrench.resize(n_chains);
	base.name.resize(1);
	base.frame.resize(1);
	base.twist.resize(1);
	base_next.name.resize(1); base_next.name[0] = "base_link";
	base_next.frame.resize(1);
	base_next.twist.resize(1);
	supports.name = support_legs;
	supports.contact.resize(n_chains);
	supports.support.resize(n_chains);

	// data samples
	out_supports_port.setDataSample(supports);
	out_limbs_ref_port.setDataSample(limbs);
	out_base_ref_port.setDataSample(base_next);

	log(INFO) << "FollowStance is configured !" << endlog();
	return true;
}

bool FollowStance::startHook()
{
	if (!setupSupports(support_legs)) return false;
	ik_success = true;
	resource_client->resourceChangeRequest(support_legs);

	// data samples
	in_base_port.getDataSample(base);

	// now update hook will be periodically executed
	log(INFO) << "FollowStance is started !" << endlog();
	return true;
}


void FollowStance::updateHook()
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

		// publish support state
		out_supports_port.write(supports);

		if (base_pose_feedback || !ik_success) {
			// get base_link pose: this overrides component state
			// if IK failed this is the best behavior
			if (in_base_port.read(base, true) == NoData || !isValidRigidBodyStateFrameTwist(base, 1) || base.name[0] != "base_link") {
				// no information about robot pose --- skip iteration
				return;
			}
		}

		// get reference pose
		{
			geometry_msgs::PoseStamped pose_stamped;
			if (in_base_ref_port.read(pose_stamped, false) == NewData) {
					// convert to KDL 
					normalizeQuaternionMsg(pose_stamped.pose.orientation);
					tf::poseMsgToKDL(pose_stamped.pose, base_ref);
			}
		}
		
		// calculate speed of the origin of base_link in world frame
		KDL::Vector dp = base.twist[0].vel + base.twist[0].rot*base.frame[0].p; // speed of base_link origin
		// Re = inv(Rr)*R, logm(Re)
		// calculate matrix log: inv(Rr)*R = exp(S(Re_axis*Re_angle))
		KDL::Vector Re_axis; 
		double Re_angle;
		rotationMatrixToAngleAxis(base.frame[0].M.Inverse()*base_ref.M, Re_axis, Re_angle);

		// now calculate acceleration: PD regulators for pe = pr - p and Re 
		KDL::Twist accel;
		accel.rot = (-Kv)*base.twist[0].rot + (Kp)*(base_ref.M*(Re_axis*Re_angle));
		accel.vel = (-Kv)*dp + Kp*(base_ref.p - base.frame[0].p) - accel.rot*base.frame[0].p - base.twist[0].rot*dp;
		// integrate base_link pose
		// TODO Is there better way to integrate pose?
		base_next.twist[0] = base.twist[0] + (0.5*period)*accel;
		KDL::Rotation Rt = KDL::Rot(base_next.twist[0].rot*period);
		base_next.frame[0].M = Rt*base.frame[0].M;
		base_next.frame[0].p = Rt*base.frame[0].p + base_next.twist[0].vel*period;
		base_next.twist[0] += (0.5*period)*accel;

		// calculate new limb poses
		KDL::Twist legs_twist = - base.frame[0].Inverse(base.twist[0]); // move to base_link frame and change direction
		KDL::Frame base_inv = base.frame[0].Inverse(); 
		for(int k = 0; k < support_leg_anchors.size(); k++) {
			limbs.frame[k] =  base_inv * support_leg_anchors[k];
			limbs.twist[k] = legs_twist;
		}
		// pass result to kinematics
		if (poseToJointStatePublish.ready()) {
			// syncronous interface
			ik_success = poseToJointStatePublish(limbs);
			if (ik_success) out_base_ref_port.write(base_next);
			else  {
				// do not move base if IK failed
				base.twist[0] = KDL::Twist::Zero(); 
				out_base_ref_port.write(base); 
			}
		} 
		else {
			// async interface: assume always success
			ik_success = true;;
			out_base_ref_port.write(base_next);
			out_limbs_ref_port.write(limbs);
		}

		if (log(DEBUG)) {
			log() << "base.p = " << base.frame[0].p << " base_ref.p =" <<  base_ref.p << " angle_error = " << Re_angle << " angle_axis = " << Re_axis << std::endl;
			log() << "base.M = \n" << base.frame[0].M << " base_ref.M = \n" << base_ref.M << std::endl;
			log() << "base.twist = " << base.twist[0] << std::endl;
			log() << "accel = " << accel << std::endl;
			log() << "base_new.p = " << base_next.frame[0].p <<  std::endl;
			log() << "base_new.M = \n" << base_next.frame[0].M << std::endl;
			log() << "base_new.twist = " << base_next.twist[0] << std::endl;
			log() << "ik_success = " << ik_success << " sync_mode = " << poseToJointStatePublish.ready() << endlog();
		}

		if (ik_success && !base_pose_feedback) {
			// kinematics succesed and we are working without feedback (in_base_port is ignored)
			// store result base_link pose to use at the next time step
			base.frame[0] = base_next.frame[0];
			base.twist[0] = base_next.twist[0];
		}

	}
	else if (state == ResourceClient::NONOPERATIONAL) {
		log(INFO) << "FollowStance is exiting  operational state !" << endlog();
		this->stop();
	}
}

/* 
 * Preempts the controllers and releases its resources.
 */
void FollowStance::stopHook() 
{
	// user calls stop() directly 
	if (resource_client->isOperational()) resource_client->stopOperational();
	// deinitialization
	// release all resources
	log(INFO) << "FollowStance is stopped!" << endlog();
}

void FollowStance::cleanupHook() 
{
	// free memory, close files and etc
	log(INFO) << "FollowStance cleaning up !" << endlog();
}

/**
 * ROS comaptible start/stop operation.
 */
bool FollowStance::rosSetOperational(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
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
 * ORO_LIST_COMPONENT_TYPE(FollowStance)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::controller::FollowStance)
