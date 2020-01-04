#include "follow_stance-component.hpp"

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

FollowStance::FollowStance(std::string const& name)  : 
	SimpleControllerBase(name),
	poseToJointStatePublish("poseToJointStatePublish", this->engine()) // operation caller
{
	// ports input 
	this->addPort("in_limbs_fixed", in_limbs_port)
		.doc("Robot limbs postions. They are used only to setup anchors on component start.");
	this->addPort("in_base", in_base_port)
		.doc("Robot base_link current pose in world frame.");
	this->addPort("in_base_ref", in_base_ref_port)
		.doc("Target base_link pose in world frame.");
	this->addPort("in_balance", in_balance_port)
		.doc("Information about robot balance.");
	// ports  output
	this->addPort("out_base_ref", out_base_ref_port)
		.doc("Base reference position. It is computed by component each control cycle.");
	this->addPort("out_limbs_ref", out_limbs_ref_port)
		.doc("Limbs reference position. It is computed by component each control cycle. ");
	this->addPort("out_supports", out_supports_port)
		.doc("Active contact list.");

	// properties
	base::PropertyBase * support_legs = this->getProperty("controlled_chains");
	Property< std::vector<std::string> > support_legs_prop(support_legs);
	if (support_legs_prop.ready()) {
		std::vector<std::string> legs = { "leg1", "leg2", "leg3", "leg4" };
		support_legs->setName("support_legs");
		support_legs->setDescription("Robot kinematic chains which are in contact.");
		support_legs_prop.set(legs);
	}
	else {
		log(ERROR) << "Property `controlled_chains` is not present. Dazed and confused." << endlog();
		this->exception();
	}

	this->addProperty("pose_feedback", pose_feedback)
		.doc("If it is false component ignores in_base and in_limbs_fixed ports after receving initial pose. Reference trajectory is"
				"calculated without any feedback. In most cases this way is more robust if robot pose cannot be suddently changed."
				"To comform kinematic constraints inverse kinematics should be connected syncronously via poseToJointStatePublish operation. ")
		.set(false);
	this->addProperty("activation_delay", activation_delay)
		.doc("Delay in control cycles between first support state publication and initial pose acquition. Allows correctly process odometry pose adjustments.")
		.set(5);
	this->addProperty("check_balance", balance_check)
		.doc("Ignore reference pose that violates static stability conditions.")
		.set(true);
	this->addProperty("keep_balance", balance_keep)
		.doc("If staic stability condtions are violated move to failsafe pose. May not end well.")
		.set(false);
	this->addProperty("keep_balance_safe_z_min", safe_pose_z_min)
		.doc("If static balance condition is violated in 'keep_balace' mode robot tries to move its center of mass to geometrical the center of support polygone (safe pose). This parameter specifies minimal z coordinate of safe pose.")
		.set(0.0);
	this->addProperty("keep_balance_safe_z_max", safe_pose_z_max)
		.doc("If static balance condition is violated robot tries to move it center of mass to geometrical center of support polygone (safe pose). This parameter specifies maximal z coordinate of safe pose.")
		.set(1.0);
	// operations: provided
	// operations: required
	this->requires()->addOperationCaller(poseToJointStatePublish); // kinematics service

	// Service: reqires
	robot_model = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));

	log(INFO) << "FollowStance is constructed!" << endlog();
}

bool FollowStance::setupSupports(const std::vector<std::string>& support_legs)
{
	// check if limb data is available
	if (in_limbs_port.read(limbs_full, true) == NoData) {
		log(ERROR) << "Limbs pose is unknown (in_limbs_port). Unable setup anchors." << endlog();
		return false;
	}
	if (!isValidRigidBodyStateNameFrame(limbs_full)) {
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

    // sort limbs in natural order
	// TODO more effective? 
	//std::sort(support_legs.begin(), support_legs.end(), [robot_model](const std::string& s1, const std::string& s1) { return robot_model->getChainIndex(s1) < robot_model->getChainIndex(s2); })
	std::vector<std::string> kinematic_chains = robot_model->listChains();

	// contact list
	support_leg_anchors.clear();
	supports.name.clear();
	supports.support.assign(support_legs.size(), 1.0);
	supports.contact.clear();
	support_leg_index.clear();
	for(  const std::string& leg : kinematic_chains ) {
		// check if leg is marked as support
		if ( std::find(support_legs.begin(), support_legs.end(), leg) == support_legs.end() ) continue;
		if (supports.name.size() == support_legs.size()) break;
		// add contact
		supports.name.push_back(leg);
		supports.contact.push_back(robot_model->getChainDefaultContact(leg)); // TODO contact configuration
		if (supports.contact.back() == "") {
			log(ERROR) << "No contact information for chain `" << leg << "`." << endlog();
			return false;
		}
		// add anchor
		auto it = std::find(limbs_full.name.begin(), limbs_full.name.end(), leg);
		if (it == limbs_full.name.end()) {
			log(ERROR) << "No information about limb `" << leg << "` pose on in_limbs_port. Unable setup anchors." << endlog();
			return false;
		}
		support_leg_anchors.push_back( base.frame[0] * limbs_full.frame[it - limbs_full.name.begin()] ); // leg position in world frame
		// add leg index inside limbs message
		support_leg_index.push_back(it - limbs_full.name.begin()); 
	}
	if (supports.name.size() == 0) {
		log(ERROR) << "No correct kinematic chains are supplied." << endlog();
		return false;
	}
	if (supports.name.size() != support_legs.size()) {
		log(WARN) << "Unknown kinematic chain in support_legs list." << endlog();
	}

	// limbs will be used as buffer for limb pose publication
	limbs.name = supports.name;
	limbs.frame.resize(supports.name.size());
	limbs.twist.resize(supports.name.size());
	limbs.wrench.clear();

	if (log(INFO)) {
		log() << "Set anchors for chains [ "; 
		for( const std::string& name :  supports.name ) log() << name << ", ";
		log() << " ]." << endlog();
	}

	// reset target
	base_ref.frame[0] = base.frame[0];
	base_ref.twist[0] = KDL::Twist::Zero();

	// reset filters
	if (!filter->reset(base, period)) {
		log(ERROR) << "JointState filter reset has failed." << endlog();
		return false;
	}

	// data samples
	out_supports_port.setDataSample(supports);
	out_limbs_ref_port.setDataSample(limbs);

	return true;
}

bool FollowStance::configureHook_impl()
{
	// INITIALIZATION
	// check if filter present
	filter = getSubServiceByType<filter::FilterRigidBodyStateInterface>(this->provides().get());
	if (!filter) {
		log(ERROR) << "RigidBodyState filter service is not loaded." << endlog();
		return false;
	}

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
		if (!pose_feedback) {
			log(WARN) << "Controller is running without any feedback. Connect operation to stop pose integration if kinematic constrains are violated." << endlog();
		}
	}

	if (safe_pose_z_min < 0.0 || safe_pose_z_max < safe_pose_z_min) {
		log(ERROR) << "safe_pose_z properties are incorrect: ensure 0 <= safe_pose_z_min <= safe_pose_z_max." << endlog();
	}

	// allocate memory
	//int n_chains = support_legs.size();
	//limbs.name.resize(n_chains);
	//limbs.frame.resize(n_chains);
	//limbs.twist.resize(n_chains);
	//limbs.wrench.resize(n_chains);
	base.name.resize(1);
	base.frame.resize(1);
	base.twist.resize(1);
	base_ref.frame.resize(1);
	base_ref.twist.resize(1);
	base_next.name.resize(1); base_next.name[0] = "base_link";
	base_next.frame.resize(1);
	base_next.twist.resize(1);
	//supports.name = support_legs;
	//supports.contact.resize(n_chains);
	//supports.support.resize(n_chains);

	// data samples
	//out_supports_port.setDataSample(supports);
	//out_limbs_ref_port.setDataSample(limbs);
	out_base_ref_port.setDataSample(base_next);

	log(INFO) << "FollowStance is configured !" << endlog();
	return true;
}

bool FollowStance::startHook_impl()
{
	ik_success = true;
	if (!pose_feedback) activation_delay_counter = activation_delay;
	else activation_delay_counter = 0;

	// data samples
	in_base_port.getDataSample(base);
	in_balance_port.getDataSample(balance);

	// now update hook will be periodically executed
	log(INFO) << "FollowStance is started !" << endlog();
	return true;
}

bool FollowStance::processResourceSet_impl(const std::vector<std::string>& set_operational_goal_chains, std::vector<std::string>& resources_to_request)
{
	// check if supplied resourses represents chains
	bool chains_set = std::all_of(set_operational_goal_chains.begin(), set_operational_goal_chains.end(), [this](const std::string& name) { return robot_model->getChainIndex(name) >= 0; } );
	if (!chains_set) {
		log(ERROR) << "FollowStance: resource set must contains kinematic chains." << endlog();
		return false;
	}
	resources_to_request = robot_model->getChainsGroups(set_operational_goal_chains);
	return true;
}


bool FollowStance::resourceChangedHook_impl(const std::vector<std::string>& set_operational_goal_chains, const std::vector<std::string>& requested_resources)
{
	if (!resource_client->hasResources(requested_resources)) return false;

	ik_success = true;
	return setupSupports(set_operational_goal_chains);
}

bool FollowStance::isInsideSupportPolygone(const KDL::Vector point) 
{
	if (balance.support_points.size() <= 2) {
		// too few supports
		return false;	
	}
	else {
		// check balance condititon: point must be inside polygone
		double x = balance.support_points[0].x() - balance.support_points.back().x();
		double y = balance.support_points[0].y() - balance.support_points.back().y();
		double px = point.x() - balance.support_points.back().x();
		double py = point.y() - balance.support_points.back().y();
		bool in_balance =  -y*px + x*py >= 0;
		// log(WARN) << " x = " << x << " y = " << y << " px = " << px << " py = " << py << " in_balance = " << in_balance <<  endlog();
		for (int k = 1; k < balance.support_points.size() && in_balance; k++) {
			// check if point is in left half-plane
			x = balance.support_points[k].x() - balance.support_points[k-1].x();
			y = balance.support_points[k].y() - balance.support_points[k-1].y();
			px = point.x() - balance.support_points[k-1].x();
			py = point.y() - balance.support_points[k-1].y();
			in_balance =  -y*px + x*py >= 0;
			// log(WARN) << " x = " << x << " y = " << y << " px = " << px << " py = " << py << " in_balance = " << in_balance <<  endlog();
		}
		return in_balance;
	}
}	

void FollowStance::checkBalance() 
{
	if (in_balance_port.read(balance, false) != NewData) {
		// nothing changed 
		return;
	}

	// check if target pose satisfies stability conditions 
	if (! isInsideSupportPolygone(balance.CoM + base_ref.frame[0].p - base.frame[0].p) ) {
		// stop motion 
		base_ref.frame[0] = base.frame[0];
		base_ref.twist[0] = KDL::Twist::Zero();

		log(DEBUG) << " Reference is outside of support polygone: stop at " << base.frame[0].p << endlog();
	}

	// check if current pose is staticlly stable	
	if (balance_keep && ! isInsideSupportPolygone(balance.CoM)) {
		// set reference pose to geometrical center
		KDL::Vector center = KDL::Vector::Zero();
		for ( const KDL::Vector& p : balance.support_points ) center += p;
		center = center / balance.support_points.size();
		center[0] -= balance.CoM.x() - base.frame[0].p.x();
		center[1] -= balance.CoM.y() - base.frame[0].p.y();
		// calculate safe height
		double z = base.frame[0].p.z();
		if (z > safe_pose_z_max) z = safe_pose_z_max;
		else if (z < safe_pose_z_min) z = safe_pose_z_min;
		center[2] = z;

		base_ref.frame[0].p = center;
		base_ref.twist[0] = KDL::Twist::Zero();

		log(DEBUG) << "CoM is outside of support polygone: stop at " << base.frame[0].p << " move base to " << center << endlog();
		return;
	}
}


void FollowStance::updateHook_impl()
{
	// publish support state
	out_supports_port.write(supports);

	// get base pose
	if (pose_feedback || !ik_success || activation_delay_counter > 0) {
		// get base_link pose: this overrides component state
		// if IK failed this is the best behavior
		if (in_base_port.read(base, true) == NoData || !isValidRigidBodyStateNameFrameTwist(base, 1) || base.name[0] != "base_link") {
			// no information about robot pose --- skip iteration
			return;
		}
	}

	// get limbs pose
	if (pose_feedback || activation_delay_counter > 0) {
		// get limbs poses and check it for sanity
		int limbs_full_size_prev = limbs_full.name.size();
		if (in_limbs_port.read(limbs_full, false) == NewData && isValidRigidBodyStateNameFrameTwist(base)) {
			// check if message has the same size
			if (limbs_full.name.size() != limbs_full_size_prev) {
				log(WARN) << "Message on in_limbs_fixed port has changed size. Call setupSupports()." << endlog();
				std::vector<std::string> support_legs = supports.name; // support_legs property may contain different value due to actionlib activation
				setupSupports(support_legs); //we cannot pass supports.name directly due to method side effects
			}
			// reestablish anchors
			if (activation_delay_counter > 0) {
				for(int i = 0; i < support_leg_anchors.size(); i++) {
					support_leg_anchors[i] = base.frame[0] * limbs_full.frame[support_leg_index[i]]; // leg position in world frame
				}
			}
		}
	}

	// if activation delay is active do nothing
	if (!pose_feedback && activation_delay_counter > 0) {
		activation_delay_counter--;
		return;
	}

	// get reference pose
	{
		geometry_msgs::PoseStamped pose_stamped;
		if (in_base_ref_port.readNewest(pose_stamped, false) == NewData) {
			// convert to KDL 
			normalizeQuaternionMsg(pose_stamped.pose.orientation);
			tf::poseMsgToKDL(pose_stamped.pose, base_ref.frame[0]);
		}
	}
	
	// check balance (base_ref is chenged if conditions violated)
	if (balance_check) checkBalance();

	// apply filter to calculate next pose
	filter->update(base, base_ref, base_next);

	// calculate new limb poses
	KDL::Twist legs_twist = - base_next.frame[0].Inverse(base_next.twist[0]); // move to base_link frame and change direction
	if (pose_feedback) {
		// use base and limbs_full. 
		// NOTE twist is ignored!
		KDL::Frame shift = base_next.frame[0].Inverse() * base.frame[0]; 
		log(DEBUG) << "shift.p = " << shift.p << " shift.M =" <<  shift.M  << endlog();
		for(int k = 0; k < limbs.name.size(); k++) {
			limbs.frame[k] =  shift * limbs_full.frame[support_leg_index[k]];
			limbs.twist[k] = legs_twist;
		}
		log(DEBUG) << "limbs_next[0].p = " << limbs.frame[0].p << "limbs[0].p = " << limbs_full.frame[0].p << endlog();
	}
	else {
		// use anchors
		KDL::Frame base_inv = base_next.frame[0].Inverse(); 
		for(int k = 0; k < support_leg_anchors.size(); k++) {
			limbs.frame[k] =  base_inv * support_leg_anchors[k];
			limbs.twist[k] = legs_twist;
		}
	}
	limbs.header.stamp = ros::Time::now();
	base_next.header.stamp = ros::Time::now();
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

	if (ik_success && !pose_feedback) {
		// kinematics succesed and we are working without feedback (in_base_port is ignored)
		// store result base_link pose to use at the next time step
		base.frame[0] = base_next.frame[0];
		base.twist[0] = base_next.twist[0];
	}

	if (log(DEBUG)) {
		KDL::Vector Re_axis;
		double Re_angle;
		rotationMatrixToAngleAxis(base.frame[0].M.Inverse()*base_ref.frame[0].M, Re_axis, Re_angle);

		log() << "base.p = " << base.frame[0].p << " base_ref.p =" <<  base_ref.frame[0].p << " angle_error = " << Re_angle << " angle_axis = " << Re_axis << std::endl;
		log() << "base.M = \n" << base.frame[0].M << " base_ref.M = \n" << base_ref.frame[0].M << std::endl;
		log() << "base.twist = " << base.twist[0] << std::endl;
		//log() << "accel = " << accel << std::endl;
		log() << "base_new.p = " << base_next.frame[0].p <<  std::endl;
		log() << "base_new.M = \n" << base_next.frame[0].M << std::endl;
		log() << "base_new.twist = " << base_next.twist[0] << std::endl;
		log() << "ik_success = " << ik_success << " sync_mode = " << poseToJointStatePublish.ready() << endlog();
	}
}

/* 
 * Preempts the controllers and releases its resources.
 */
void FollowStance::stopHook_impl() 
{
	// deinitialization
	// release all resources
	log(INFO) << "FollowStance is stopped!" << endlog();
}

void FollowStance::cleanupHook_impl() 
{
	// free memory, close files and etc
	log(INFO) << "FollowStance cleaning up !" << endlog();
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
