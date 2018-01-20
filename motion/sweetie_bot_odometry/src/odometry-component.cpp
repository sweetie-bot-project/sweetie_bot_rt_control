#include "odometry-component.hpp"

#include <cmath>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <rtt/Component.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <ros/time.h>

#include <sweetie_bot_orocos_misc/message_checks.hpp>

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
	this->addPort("in_supports_fixed", support_port).
		doc("List of end effectors which are in contact. `point` fields are ignored. ");
	this->addEventPort("in_limbs_fixed", limbs_port).
		doc("Positions of robot end effectors relative to the base.");
	this->addEventPort("in_base", body_reset_port).
		doc("Position of robot `base_link`. Reset odometry results.");
	this->addPort("out_base", body_port).
		doc("Position of robot `base_link` relative it pose at start.");
	this->addPort("out_tf", tf_port).
		doc("Publish position of robot base for ROS components.");
	/*this->addPort("out_twist", twist_port).
		doc("Publish twist of robot base for ROS components.");*/
	// PROPERTIES
	this->addProperty("legs", legs).
		doc("List of end effectors which can be in contact. (Kinematic chains names, legs)."); 
	this->addProperty("force_contact_z_to_zero", force_contact_z_to_zero).
		doc("Assume that first contact point always have zero Z coordinate.").
		set(false);
	this->addProperty("odometry_frame", odometry_frame)
		.doc("Stationary frame name (header.frame_id field value in publised tf messages).")
		.set("odom_combined");
	this->addProperty("tf_prefix", base_link_tf_prefix)
		.doc("tf_prefix for base_link in output tf messages.")
		.set("");

	this->addOperation("setIdentity", &Odometry::setIdentity, this)
		.doc("Set body transform to Identity.");

	// Service: requires
	robot_model = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));

	log(INFO) << "Odometry constructed !" << endlog();
}

bool Odometry::configureHook()
{
	// check if RobotModel Service presents
	if (!robot_model->ready() || !robot_model->isConfigured()) {
		log(ERROR) << "RobotModel service is not ready." << endlog();
		return false;
	}
	// reserve LimbState array
	limbs.clear();
	for ( const string& name : legs ) {
		int index = robot_model->getChainIndex(name);
		if (index < 0) {
			log(ERROR) << "Kinematic chain (leg) " << name << " does not exists." << endlog();
			return false;
		}
		limbs.emplace_back(name);
		limbs.back().index = index;
		//TODO reserve memory in buffers
	}
	// reserve memory
	body_pose.name.resize(1);
	body_pose.frame.resize(1);
	body_pose.twist.resize(1);
	body_pose.name[0] = "base_link";
	body_tf.transforms.resize(1);
	body_tf.transforms[0].header.frame_id = odometry_frame;
	if (base_link_tf_prefix != "") body_tf.transforms[0].child_frame_id = base_link_tf_prefix + "/base_link";
	else body_tf.transforms[0].child_frame_id = "base_link";
	// body_twist.header.frame_id = odometry_frame;
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
	body_pose.frame[0] = Frame::Identity();
	body_pose.twist[0] = Twist::Zero();
	// get data sample
	limbs_port.getDataSample(limb_poses);
	support_port.getDataSample(support_state);
	body_reset_port.getDataSample(body_reset_pose);
	// read support state (if it is not published periodically)
	support_state.name.clear();
	support_state.contact.clear();
	support_state.support.clear();
	support_port.read(support_state, true); // get OldState if it is available
	// display WARN again
	too_few_contact_warn_counter = 0;

	log(INFO) << "Odometry started !" << endlog();
	return true;
}


void Odometry::estimateRigidBodyPose(const std::vector<Vector>& contact_points_body, const std::vector<Vector>& contact_points_anchor, Frame& T)
{
	int n_points = contact_points_body.size();
	// calculate centroid
	Vector center_point_body = Vector::Zero();
	Vector center_point_anchor = Vector::Zero();
	for( const Vector& p : contact_points_body ) center_point_body += p;
	for( const Vector& p : contact_points_anchor ) center_point_anchor += p;
	center_point_anchor = center_point_anchor / n_points;
	center_point_body = center_point_body / n_points;

	// Some debug information
	if (log(DEBUG)) {
		log() << "Total " << contact_points_body.size() << " == " << contact_points_anchor.size() << " contact points are registered." << std::endl;
		for ( const Vector& v : contact_points_body ) log() << v.x() << " " << v.y() << " " << v.z() << std::endl;
		log() << endlog();
	}
	// Calculate transform
	// calculate H matrix
	Eigen::Matrix3d H = Matrix3d::Zero();
	for (int k = 0; k < n_points; k++) {
		// outer product
		H += (Map<const Vector3d>(contact_points_body[k].data) - Map<Vector3d>(center_point_body.data)) * (Map<const Vector3d>(contact_points_anchor[k].data) - Map<Vector3d>(center_point_anchor.data)).transpose();
	}
	// SVD decomposition
	JacobiSVD<Matrix3d> svd(H, ComputeFullU | ComputeFullV);
	// rotation
	Map< Matrix<double,3,3,Eigen::RowMajor> > R(T.M.data);
	R = svd.matrixV() * svd.matrixU().transpose();
	// translation
	T.p = center_point_anchor - T.M*center_point_body;
	if (log(DEBUG)) {
		log() << "U = " << svd.matrixU() << std::endl << "V = " << svd.matrixV() << "S = " << svd.singularValues() <<  std::endl << "T.M = " << R << std::endl << " T.p = " << Map<Vector3d>(T.p.data) << endlog();
	}
}

void Odometry::estimateVelocity() 
{
	// TODO implement correct speed estimation: LS solution
	KDL::Twist avg_body_twist = KDL::Twist::Zero();
	int n_support_limbs = 0;	
	for(int ind = 0; ind < limbs.size(); ind++) {
		if (limbs[ind].is_in_contact) {
			avg_body_twist -= limb_poses.twist[ind];
			n_support_limbs++;
		}
	}
	avg_body_twist = avg_body_twist / n_support_limbs;
	// result
	// TODO Why it has to be inversed?!!
	body_pose.twist[0] = body_pose.frame[0] * avg_body_twist;
}

void Odometry::updateAnchor(bool check_support_state)
{
	contact_points_anchor.clear();
	contact_points_body.clear();
	for(int ind = 0; ind < limbs.size(); ind++) {
		LimbState& limb = limbs[ind];
		// check if contact list has changed
		if (check_support_state) {
			limb.is_in_contact = support_state.support[ind] > 0.0;
			limb.contact_name_changed = limb.contact_name != support_state.contact[ind];
		}
		// update limb state
		limb.was_in_contact = limb.is_in_contact;
		if (limb.contact_name_changed) {
			limb.contact_name = support_state.contact[ind];
			// retrive contact points coordinates
			//limb.contact_points_limb = robot_model->getContactPoints(limb.contact_name);
			limb.contact_points_limb.clear();
			robot_model->addContactPointsToBuffer(limb.contact_name, limb.contact_points_limb); 
		}
		if (limb.is_in_contact && limb.contact_points_limb.size() > 0) {
			double z_shift = 0.0;
			// add points to anchor
			contact_points_anchor.push_back( body_pose.frame[0] * limb_poses.frame[ind] * limb.contact_points_limb[0] ); // calculate absolute coordinates of the point
			if (force_contact_z_to_zero) {
				// project point to Z=0 plane
				z_shift = - contact_points_anchor.back().data[2];
				contact_points_anchor.back().data[2] = 0.0;
			}
			for(auto it = limb.contact_points_limb.begin() + 1; it != limb.contact_points_limb.end(); it++) {
				contact_points_anchor.push_back( body_pose.frame[0] * limb_poses.frame[ind] * (*it) );
				contact_points_anchor.back().data[2] += z_shift;
			}
			// add points to body
			for ( Vector& p : limb.contact_points_limb ) contact_points_body.push_back( limb_poses.frame[ind] * p );
		}
	}
}

bool Odometry::integrateBodyPose() 
{	
	// iterate over end effectors which is possible in contact
	bool contact_set_changed = false;
	int n_points = 0;
	for(int ind = 0; ind < limbs.size(); ind++) {
		LimbState& limb = limbs[ind];
		// check received pose
		// TODO allows to skip limbs
		if (limb_poses.name[ind] != limb.name) {
			log(ERROR) << "Incorrect limbs order in RigidBodyState message." << endlog();
			return false;
		}
		if (support_state.name[ind] != limb.name) {
			log(ERROR) << "Incorrect limbs order in SupportState message." << endlog();
			return false;
		}
		// check contact
		limb.is_in_contact = support_state.support[ind] > 0.0;
		limb.contact_name_changed = limb.contact_name != support_state.contact[ind];
		// check if contact has changed
		if (limb.is_in_contact != limb.was_in_contact) contact_set_changed = true;
		if (limb.was_in_contact && limb.contact_name_changed) contact_set_changed = true;
		// calculate number of contact points which can be used to pose calcuation
		if (limb.is_in_contact && limb.was_in_contact && !limb.contact_name_changed) {
			n_points += limb.contact_points_limb.size();
		}
	}

	// check point number
	if (n_points < 3 ) {
		if (n_points > 0) {
			// warn if we have only one or two points
			if (!too_few_contact_warn_counter) log(WARN) << "Too few contact points. Skiped iterations. " << endlog();
			too_few_contact_warn_counter += 1;
		}
		// do not change pose estimation
		if (contact_set_changed) updateAnchor(true);
		return false;
	}
	if (too_few_contact_warn_counter) {
		log(WARN) << "Have enough contact points again. " << too_few_contact_warn_counter << " iterations skipped." << endlog();
		too_few_contact_warn_counter = 0;
	}

	// estimate body position
	if (!contact_set_changed) {
		// use anchor points to calculate body position
		
		// Iterate over limbs: update contact points coordinates in body frame. Absolute cooordinates is the same.
		// Order and number of points is the same: contact points set did not changed.
		contact_points_body.clear();
		for(int ind = 0; ind < limbs.size(); ind++)
			if (limbs[ind].is_in_contact) {
				for ( const Vector& point : limbs[ind].contact_points_limb ) {
					contact_points_body.push_back( limb_poses.frame[ind] * point );
				}
			}

		// pose calculation
		estimateRigidBodyPose(contact_points_body, contact_points_anchor, body_pose.frame[0]);
	}
	else {
		//contact set has changed
		log(INFO) << "Contact set has changed." << endlog();
		
		// anchor points can't be used to calculate transform
		// calculate pose increment over last period
		contact_points_anchor.clear();
		auto it = contact_points_body.begin();
		for(int ind = 0; ind < limbs.size(); ind++) {
			if (limbs[ind].was_in_contact) {
				// add points coordinates in body[k+1]
				if (limbs[ind].is_in_contact && !limbs[ind].contact_name_changed) {
					for ( const Vector& point : limbs[ind].contact_points_limb ) {
						contact_points_anchor.push_back( limb_poses.frame[ind] * point );
					}
					it += limbs[ind].contact_points_limb.size();
				}
				else {
					// delete lost contact point
					it = contact_points_body.erase(it, it + limbs[ind].contact_points_limb.size());
				}
			}
		}

		// estimate pose increment
		Frame T; // transform from body[k+1] to body [k]
		estimateRigidBodyPose(contact_points_anchor, contact_points_body, T);
		body_pose.frame[0] = body_pose.frame[0] * T;

		// update contact point list
		updateAnchor(false);
	}

	//speed estimation
	estimateVelocity();

	return true;
}


static inline void normalizeQuaternionMsg(geometry_msgs::Quaternion& q) {
	auto s = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
	q.x /= s; q.y /= s; q.z /= s; q.w /= s;
}

bool Odometry::setIdentity() 
{
	if (!isValidRigidBodyStateNameFrame(limb_poses) || limb_poses.name.size() < limbs.size()) {
		log(ERROR) << "Incorrect RigidBodyState message on limbs_port: its is too small or field size is inconsistent. msg.name.size = " << limb_poses.name.size() << endlog();
		return false;
	}
	if (!isValidSupportStateNameSuppCont(support_state) || support_state.name.size() < limbs.size()) {
		log(ERROR) << "SupportState is unknown." << endlog();
		return false;
	}
	// reset odometry result
	body_pose.frame[0] = Frame::Identity();
	body_pose.twist[0] = Twist::Zero();
	// update stored limb state and contact list
	updateAnchor(true);
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
	if (!isValidSupportStateNameSuppCont(support_state) || support_state.name.size() < limbs.size()) {
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
				body_pose.frame[0] = body_reset_pose.frame[index];
				if (body_reset_pose.twist.size()) body_pose.twist[0] = body_reset_pose.twist[index];
				else body_pose.twist[0] = Twist::Zero();
				// update stored limb state and contact list
				updateAnchor(true);
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
	ros::Time stamp = ros::Time::now();
	// RigidBodyState message
	body_pose.header.stamp = stamp;
	body_port.write(body_pose);
	// tf message
	body_tf.transforms[0].header.stamp = stamp;
	tf::transformKDLToMsg(body_pose.frame[0], body_tf.transforms[0].transform);
	// fix for incorrect kdl::Rotation::GetQuaternion conversation
	normalizeQuaternionMsg(body_tf.transforms[0].transform.rotation);
	tf_port.write(body_tf);
	// twist message
	/*body_twist.header.stamp = stamp;
	tf::twistKDLToMsg(body_pose.twist[0], body_twist.twist);
	twist_port.write(body_twist);*/
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
