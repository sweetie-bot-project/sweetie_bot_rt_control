#include "odometry-component.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>

#include <rtt/Component.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <ros/time.h>


using namespace RTT;
using namespace KDL;
using namespace Eigen;

namespace sweetie_bot {
namespace motion {

Odometry::Odometry(std::string const& name) : 
	TaskContext(name, PreOperational),
	log(logger::categoryFromComponentName(name))
{
	if (!log.ready()) {
		RTT::Logger::In in("Odometry");
		RTT::log(RTT::Error) << "Logger is not ready!" << RTT::endlog();
	}
	// PORTS
	this->addPort("in_support_fixed", support_port).
		doc("List of end effectors which are in contact. `point` fields are ignored. ");
	this->addEventPort("in_limbs_fixed", limbs_port).
		doc("Positions of robot end effectors relative to the base.");
	this->addEventPort("in_base", body_reset_port).
		doc("Position of robot `base_link`. Reset odometry results.");
	this->addPort("out_base", body_port).
		doc("Position of robot `base_link` relative it pose at start.");
	this->addPort("out_tf", tf_port).
		doc("Publish position of robot base for ROS components.");
	// PROPERTIES
	this->addProperty("legs", legs).
		doc("List of end effectors which can be in contact. (Kinematic chains names, legs)."); 
	this->addProperty("contact_points", contact_points_prop).
		doc("Default position of contact points in end effector coordinates. Format: [ x1, y1, z1, x2, y2, z3, ...].").
		set(vector<double>({ 0.0, 0.0, 0.0 }));
	this->addProperty("odometry_frame", odometry_frame)
		.doc("Stationary frame name (header.frame_id field value in publised tf messages).")
		.set("odom_combined");
	this->addProperty("tf_prefix", base_link_tf_prefix)
		.doc("tf_prefix for base_link in output tf messages.")
		.set("");


	log(INFO) << "Odometry constructed !" << endlog();
}

bool Odometry::configureHook()
{
	// get default contact point location
	if (contact_points_prop.size() % 3 != 0 || contact_points_prop.size() == 0) {
		log(ERROR) << "Incorrect contact_point coordiantes. Array size must be divisible by 3 and not equal to zero." << endlog();
		return false;
	}
	// add points to list
	default_contact_points.clear();
	for(int i = 0; i < contact_points_prop.size(); i += 3) {
		default_contact_points.emplace_back(contact_points_prop[i], contact_points_prop[i+1], contact_points_prop[i+2]);
	}
	// reserve LimbState array
	limbs.clear();
	for ( const string& name : legs ) limbs.emplace_back(name);
	// reserve memory
	body_pose.name.resize(1);
	body_pose.frame.resize(1);
	body_pose.twist.resize(1);
	body_pose.name[0] = "base_link";
	body_tf.transforms.resize(1);
	body_tf.transforms[0].header.frame_id = odometry_frame;
	if (base_link_tf_prefix != "") body_tf.transforms[0].child_frame_id = base_link_tf_prefix + "/base_link";
	else body_tf.transforms[0].child_frame_id = "base_link";
	// initialization is finished
	log(INFO) << "Odometry configured !" << endlog();
	return true;
}

bool Odometry::startHook()
{
	// clear contact data
	for ( LimbState& limb : limbs )
		limb.was_in_contact = false;
	// reset body pose
	body_anchor = Frame::Identity();
	body_pose.frame[0] = Frame::Identity();
	body_pose.twist[0] = Twist::Zero();
	// get data sample
	limbs_port.getDataSample(limb_poses);
	support_port.getDataSample(support_state);
	body_reset_port.getDataSample(body_reset_pose);
	// read support state (if it is not published periodically)
	support_state.name.clear();
	support_state.support.clear();
	support_port.read(support_state, true); // get OldState if it is available

	log(INFO) << "Odometry started !" << endlog();
	return true;
}

bool Odometry::integrateBodyPose() 
{	
	// clear contact point list
	contact_points.clear();
	contact_points_prev.clear();
	Vector center_point = Vector::Zero();
	Vector center_point_prev = Vector::Zero();
	// iterate over end effectors which is possible in contact
	bool contact_set_changed = false;
	for(int ind = 0; ind < limbs.size(); ind++) {
		// check received pose
		// TODO allows to skip limbs
		if (limb_poses.name[ind] != limbs[ind].name) {
			log(ERROR) << "Incorrect limbs order in RigidBodyState message." << endlog();
			return false;
		}
		if (support_state.name[ind] != limbs[ind].name) {
			log(ERROR) << "Incorrect limbs order in SupportState message." << endlog();
			return false;
		}
		KDL::Frame& limb_current_pose = limb_poses.frame[ind];
		// iterate over contact list
		// they also is sorted in natural order
		bool is_in_contact = support_state.support[ind] > 0.0;
		if (is_in_contact != limbs[ind].was_in_contact) contact_set_changed = true;
		// process contact
		if (is_in_contact && limbs[ind].was_in_contact) {
			// this limb can be used for odometry calculation
			// add points for transform calculation
			for ( const Vector& point : default_contact_points ) {
				// position
				contact_points_prev.push_back( limbs[ind].previous_pose * point );
				center_point_prev += contact_points_prev.back();
				contact_points.push_back( limb_current_pose * point );
				center_point += contact_points.back();
				// TODO twist

			}
		}
		// update contact state
		limbs[ind].was_in_contact = is_in_contact;
	}
	if (contact_set_changed) {
		// update limb state
		for(int ind = 0; ind < limbs.size(); ind++) limbs[ind].previous_pose = limb_poses.frame[ind];
		log(INFO) << "Contact set has changed." << endlog();
	}
	log(DEBUG) << "Total " << contact_points.size() << " contact points are registered." << endlog();
	if (contact_points.size() < 3) {
		log(WARN) << "To few contact points. Skip iteration." << endlog();
		return false;
	}
	// calculate centroid 
	int n_points = contact_points.size();
	center_point = center_point / n_points;
	center_point_prev = center_point_prev / n_points;
	// calculate H matrix
	Eigen::Matrix3d H = Matrix3d::Zero();
	for (int k = 0; k < n_points; k++) {
		// outer product
		contact_points[k] -= center_point;
		contact_points_prev[k] -= center_point_prev;
		H += Map<Vector3d>(contact_points[k].data) * Map<Vector3d>(contact_points_prev[k].data).transpose();
	}
	// SVD decomposition
	JacobiSVD<Matrix3d> svd(H, ComputeFullU | ComputeFullV);
	// calculate trasform
	Frame T;
	// rotation
	Map< Matrix<double,3,3,Eigen::RowMajor> > R(T.M.data);
	R = svd.matrixV() * svd.matrixU().transpose();
	// translation
	T.p = center_point_prev - T.M*center_point;
	// pose calculation
	body_pose.frame[0] = body_anchor * T;
	if (contact_set_changed) body_anchor = body_pose.frame[0];
	return true;
}

static inline bool isValidRigidBodyStateNameFrame(const sweetie_bot_kinematics_msgs::RigidBodyState& msg, int sz = -1) {
	if (sz < 0) sz = msg.name.size();
	else if (sz != msg.name.size()) return false;
	if (sz != msg.frame.size()) return false;
	if (sz != msg.twist.size() && msg.twist.size() != 0) return false;
	return true;
}

static inline bool isValidSupportState(const sweetie_bot_kinematics_msgs::SupportState& msg, int sz = -1) {
	if (sz < 0) sz = msg.name.size();
	else if (sz != msg.name.size()) return false;
	if (sz != msg.support.size()) return false;
	return true;
}

void Odometry::updateHook()
{
	bool pose_overrided = false;

	// get limbs state
	if (limbs_port.read(limb_poses, false) != NewData ) return; // nothing to do
	// check if message is valid
	if (!isValidRigidBodyStateNameFrame(limb_poses) || limb_poses.name.size() < limbs.size()) {
		log(ERROR) << "Incorrect RigidBodyState message on limbs_port: its is too small or field size is inconsistent. msg.name.size = " << limb_poses.name.size() << endlog();
		return;
	}
	// get support state
	if (support_port.read(support_state, false) == NoData) {
		log(ERROR) << "SupportState is unknown." << endlog();
		return;
	}
	if (!isValidSupportState(support_state) || support_state.name.size() < limbs.size()) {
		log(ERROR) << "Incorrect SupportState message: its is too samll or field size is inconsistent. msg.name.size = " << support_state.name.size() << endlog();
		return;
	}

	// if we have received pose it overrides odomery results
	if ( body_reset_port.read(body_reset_pose, false) == NewData ) {
		if (!isValidRigidBodyStateNameFrame(body_reset_pose)) {
			log(WARN) << "Received incorrect RigidBodyState message on in_body_reset port." << endlog();
		}
		else {
			auto it = find(body_reset_pose.name.begin(), body_reset_pose.name.end(), "base_link");
			if (it != body_reset_pose.name.end()) {
				// body pose was found
				int index = distance(body_reset_pose.name.begin(), it);
				// reset odometry result
				body_anchor = body_reset_pose.frame[index];
				body_pose.frame[0] = body_reset_pose.frame[index];
				if (body_reset_pose.twist.size()) body_pose.twist[0] = body_reset_pose.twist[index];
				else body_pose.twist[0] = Twist::Zero();
				// update stored limb state and contact list
				for(int ind = 0; ind < limbs.size(); ind++) {
					limbs[ind].was_in_contact = support_state.support[ind] > 0.0;	
					limbs[ind].previous_pose = limb_poses.frame[ind];
				}
				// prevent pose integration
				pose_overrided = true;
			}
		}
	}

	// if we already have new pose odometry calculations are unnecessary
	if (!pose_overrided) {
		integrateBodyPose();
	}

	// pose is publised unconditionally
	// TODO trottle if not updated?
	// RigidBodyState message
	body_port.write(body_pose);
	// tf message
	body_tf.transforms[0].header.stamp = ros::Time::now();
	tf::transformKDLToMsg(body_pose.frame[0], body_tf.transforms[0].transform);
	tf_port.write(body_tf);
}

void Odometry::stopHook() 
{
	log(INFO) << "Odometry is stopped." << endlog();
}

} // namespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Odometry)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::Odometry)
