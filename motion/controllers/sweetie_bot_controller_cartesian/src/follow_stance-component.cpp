#include "follow_stance-component.hpp"

#include <rtt/Component.hpp>
#include <kdl_conversions/kdl_msg.h>

#include <sweetie_bot_orocos_misc/get_subservice_by_type.hpp>
#include <sweetie_bot_orocos_misc/message_checks.hpp>

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
	log(logger::categoryFromComponentName(name))
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
	// operations: provided
	this->addOperation("rosSetOperational", &FollowStance::rosSetOperational, this)
		.doc("ROS compatible start/stop operation (std_srvs::SetBool).");

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

	// allocate memory
	int n_chains = support_legs.size();
	limbs.name = support_legs;
	limbs.frame.resize(n_chains);
	limbs.twist.resize(n_chains);
	limbs.wrench.resize(n_chains);
	base.name.resize(1);
	base.frame.resize(1);
	base.wrench.resize(1);
	supports.name = support_legs;
	supports.contact.resize(n_chains);
	supports.support.resize(n_chains);

	// data samples
	out_supports_port.setDataSample(supports);
	out_limbs_ref_port.setDataSample(limbs);
	out_base_ref_port.setDataSample(base);

	log(INFO) << "FollowStance is configured !" << endlog();
	return true;
}

bool FollowStance::startHook()
{
	if (!setupSupports(support_legs)) return false;

	resource_client->resourceChangeRequest(support_legs);

	// data samples
	in_base_port.getDataSample(base);

	// now update hook will be periodically executed
	log(INFO) << "FollowStance is started !" << endlog();
	return true;
}


static void rotationMatrixToAngleAxis(const KDL::Rotation& m, KDL::Vector& axis, double& angle) {
	const double epsilon = 0.01; // margin to allow for rounding errors
	const double epsilon2 = 0.1; // margin to distinguish between 0 and 180 degrees
	// optional check that input is pure rotation, 'isRotationMatrix' is defined at:
	// https://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/
	if ((std::abs(m(0,1)-m(1,0)) < epsilon) && (std::abs(m(0,2)-m(2,0))< epsilon) && (std::abs(m(1,2)-m(2,1))< epsilon)) {
		// singularity found
		// first check for identity matrix which must have +1 for all terms
		//  in leading diagonaland zero in other terms
		if ((std::abs(m(0,1)+m(1,0)) < epsilon2) && (std::abs(m(0,2)+m(2,0)) < epsilon2) && (std::abs(m(1,2)+m(2,1)) < epsilon2) && (std::abs(m(0,0)+m(1,1)+m(2,2)-3) < epsilon2)) {
			// this singularity is identity matrix so angle = 0
			// zero angle, arbitrary axis
			angle = 0.0;
			axis = KDL::Vector(1,0,0);
			return;
		}
		// otherwise this singularity is angle = 180
		angle = 3.1415926f;
		double xx = (m(0,0)+1)/2;
		double yy = (m(1,1)+1)/2;
		double zz = (m(2,2)+1)/2;
		double xy = (m(0,1)+m(1,0))/4;
		double xz = (m(0,2)+m(2,0))/4;
		double yz = (m(1,2)+m(2,1))/4;
		if ((xx > yy) && (xx > zz)) { // m(0,0) is the largest diagonal term
			if (xx< epsilon) {
				axis = KDL::Vector(0, 0.7071, 0.7071);
			} else {
				double x = std::sqrt(xx);
				axis = KDL::Vector(x, xy/x, xz/x);
			}
		} else if (yy > zz) { // m(1,1) is the largest diagonal term
			if (yy< epsilon) {
				axis = KDL::Vector(0.7071, 0, 0.7071);
			} else {
				double y = std::sqrt(yy);
				axis = KDL::Vector(xy/y, y, yz/y);
			}	
		} else { // m(2,2) is the largest diagonal term so base result on this
			if (zz< epsilon) {
				axis = KDL::Vector(0.7071, 0.7071, 0);
			} else {
				double z = std::sqrt(zz);
				axis = KDL::Vector(xz/z, yz/z, z);
			}
		}
		// return 180 deg rotation
		return;	
	}
	// as we have reached here there are no singularities so we can handle normally
	double s = std::sqrt((m(2,1) - m(1,2))*(m(2,1) - m(1,2))
		+(m(0,2) - m(2,0))*(m(0,2) - m(2,0))
		+(m(1,0) - m(0,1))*(m(1,0) - m(0,1))); // used to normalise
	if (std::abs(s) < 0.001) s=1; 
		// prevent divide by zero, should not happen if matrix is orthogonal and should be
		// caught by singularity test above, but I've left it in just in case
	angle = std::acos((m(0,0) + m(1,1) + m(2,2) - 1)/2);
	axis = KDL::Vector((m(2,1) - m(1,2))/s, (m(0,2) - m(2,0))/s, (m(1,0) - m(0,1))/s);
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

		// get base_link pose
		if (in_base_port.read(base, true) == NoData || !isValidRigidBodyStateFrameTwist(base, 1) || base.name[0] != "base_link") {
			// no information about robot pose --- skip iteration
			return;
		}

		// get reference pose
		{
			geometry_msgs::PoseStamped pose_stamped;
			RTT::FlowStatus result = in_base_ref_port.read(pose_stamped, false);
			switch (result) {
				case NewData:
					// convert to KDL 
					// TODO sanity checks
					tf::poseMsgToKDL(pose_stamped.pose, base_ref);
					break;
				case OldData:
				case NoData:
					// do not touch target
					break;
			}
		}
		
		// calculate speed of the origin of base_link in world frame
		KDL::Vector dp = base.twist[0].vel + base.twist[0].rot*base.frame[0].p; // speed of base_link origin
		// Re = inv(Rr)*R, logm(Re)
		// calculate matrix log: inv(Rr)*R = exp(S(Re_axis*Re_angle))
		KDL::Vector Re_axis; 
		double Re_angle;
		rotationMatrixToAngleAxis(base.frame[0].M.Inverse()*base_ref.M, Re_axis, Re_angle);

		if (log(DEBUG)) {
			log() << "base.p = " << base.frame[0].p << " base_ref.p =" <<  base_ref.p << " angle_error = " << Re_angle << " angle_axis = " << Re_axis << std::endl;
			log() << "base.M = \n" << base.frame[0].M << " base_ref.M = \n" << base_ref.M << std::endl;
			log() << "base.twist = " << base.twist[0] << endlog();
		}
		// now calculate acceleration: PD regulators for pe = pr - p and Re 
		KDL::Twist accel;
		accel.rot = (-Kv)*base.twist[0].rot + (Kp)*(base_ref.M*(Re_axis*Re_angle));
		accel.vel = (-Kv)*dp + Kp*(base_ref.p - base.frame[0].p) - accel.rot*base.frame[0].p - base.twist[0].rot*dp;
		// integrate base_link pose
		// TODO Is there better way to integrate pose?
		base.twist[0] += (0.5*period)*accel;
		KDL::Rotation Rt = KDL::Rot(base.twist[0].rot*period);
		base.frame[0].M = Rt*base.frame[0].M;
		base.frame[0].p = Rt*base.frame[0].p + base.twist[0].vel*period;
		//base.frame[0].M = addDelta(base.frame[0], base.twist[0], period);
		base.twist[0] += (0.5*period)*accel;

		// TODO synchronous call to kinematics_inv
		out_base_ref_port.write(base);

		// calculate new limb poses
		KDL::Twist legs_twist = - base.frame[0].Inverse(base.twist[0]); // move to base_link frame and change direction
		KDL::Frame base_inv = base.frame[0].Inverse(); 
		for(int k = 0; k < support_leg_anchors.size(); k++) {
			limbs.frame[k] =  base_inv * support_leg_anchors[k];
			limbs.twist[k] = legs_twist;
		}
		out_limbs_ref_port.write(limbs);

		if (log(DEBUG)) {
			log() << "accel = " << accel << std::endl;
			log() << "base_new.p = " << base.frame[0].p <<  std::endl;
			log() << "base_new.M = \n" << base.frame[0].M << std::endl;
			log() << "base_new.twist = " << base.twist[0] << endlog();
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
