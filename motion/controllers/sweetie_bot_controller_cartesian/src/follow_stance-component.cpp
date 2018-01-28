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
	this->addPort("in_joints_accel_sorted", in_joints_accel_port)
		.doc("Current robot state with acceleration computed dynamics inverse controller. Full sorted pose.");
	this->addPort("out_joints_ref", out_joints_port)
		.doc("Result of integration of joint state pose received on in_joints_accel_port. Position is clmped accroding to limits. ");
	this->addPort("in_base", in_base_port)
		.doc("Computed base position according to odometry calculations in world coordinates.");
	this->addPort("in_base_ref", in_base_ref_port)
		.doc("Desired base position in world coordinates.");
	this->addPort("out_base_accel", out_accel_port)
		.doc("Desired robot base acceleration.");
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
	// joints
	joint_index.clear();
	joints_ref.name.clear();
	// contact list
	supports.name.clear();
	supports.contact.clear();
	supports.support.clear();

	vector<string> joint_names;
	for(auto chain = support_legs.begin(); chain != support_legs.end(); chain++) {
		joint_names = robot_model->listJoints(*chain);
		if (joint_names.size() == 0) {
			log(ERROR) << "Empty kinematic chain `" << *chain << "`." << endlog();
			return false;
		}
		// form joint index
		for(auto joint = joint_names.begin(); joint != joint_names.end(); joint++) {
			// add joint to index
			joint_index.push_back(robot_model->getJointIndex(*joint));
			joints_ref.name.push_back(*joint);
		}
		// add contact
		supports.name.push_back(*chain);
		supports.contact.push_back(robot_model->getChainDefaultContact(*chain)); // TODO contact configuration
		if (supports.contact.back() == "") {
			log(ERROR) << "No contact information for chain `" << *chain << "`." << endlog();
			return false;
		}
		supports.support.push_back(1.0);
	}
	if (log(INFO)) {
		log() << "Controlled chains: [ "; 
		for( const string& name :  supports.name ) log() << name << ", ";
		log() << " ]." << endlog();
		log() << "Contacts: [ "; 
		for( const string& contact :  supports.contact ) log() << contact << ", ";
		log() << " ]." << endlog();
	}

	// allocate memory
	unsigned int sz = joints_ref.name.size();
	joints_ref.position.resize(sz);
	joints_ref.velocity.resize(sz);

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
	n_joints_fullpose = robot_model->listJoints("").size();

	// process support_legs parameter, form "joint_index" and "supports" SupportState message
	// also reserve memory in "joints_ref"
	if (!setupSupports(support_legs)) return false;

	// data samples
	out_supports_port.setDataSample(supports);
	out_joints_port.setDataSample(joints_ref);

	log(INFO) << "FollowStance is configured !" << endlog();
	return true;
}

bool FollowStance::startHook()
{
	resource_client->resourceChangeRequest(support_legs);
	// data samples
	in_joints_accel_port.getDataSample(joints_accel);
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

		// read joints port and integrate it
		bool joints_ref_updated = false;
		if (in_joints_accel_port.read(joints_accel, false) == NewData) {
			//check if message is correct
			if (isValidJointStateAccelPosVelAccel(joints_accel, n_joints_fullpose)) {
				// integrate pose
				for(int k = 0; k < joint_index.size(); k++) {
					int index = joint_index[k]; // fullpose index
					joints_ref.position[k] = joints_accel.position[index] + joints_accel.velocity[index]*period;
					joints_ref.velocity[k] = joints_accel.velocity[index] + joints_accel.acceleration[index]*period;
				}
				joints_ref_updated = true;
			}
		}

		// get base_link pose
		if (in_base_port.read(base, true) != NoData && isValidRigidBodyStateFrameTwist(base, 1) && base.name[0] == "base_link") {
			// get reference pose
			geometry_msgs::PoseStamped pose_stamped;
			if (in_base_ref_port.read(pose_stamped, true) != NoData) {
				KDL::Frame base_ref;
				tf::poseMsgToKDL(pose_stamped.pose, base_ref);
				// calculate base frame origin speed
				KDL::Vector dp = base.twist[0].vel + base.twist[0].rot*base.frame[0].p; // speed of base_link origin
				// Re = inv(Rr)*R, logm(Re)
				KDL::Vector Re_axis; 
				double Re_angle;
				rotationMatrixToAngleAxis(base_ref.M.Inverse()*base.frame[0].M, Re_axis, Re_angle);
				// now calculate acceleration: PD regulators for pe = pr - p and Re 
				KDL::Twist accel;
				accel.rot = (-Kv)*base.twist[0].rot + (Kp)*(base_ref.M*(Re_axis*Re_angle));
				accel.vel = (-Kv)*dp + Kp*(base_ref.p - base.frame[0].p) - accel.rot*base.frame[0].p - base.twist[0].rot*dp;
				// publish
				out_accel_port.write(accel);

				if (log(DEBUG)) {
					log() << "base.p = " << base.frame[0].p << " base_ref.p =" <<  base_ref.p << " angle_error = " << Re_angle << " angle_axis = " << Re_axis << std::endl;
					log() << "base.twist = " << base.twist[0] << std::endl;
					log() << "accel = " << accel << std::endl;
					log() << "base.M = " << base.frame[0].M << " base_ref.M = " << base_ref.M << endlog();
				}
			}
		}

		// publish new position
		if (joints_ref_updated) out_joints_port.write(joints_ref);
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
