#include "cartesian_trajectory_cache.hpp"

#include <Eigen/Geometry> 
#include <kdl_conversions/kdl_msg.h>
#include <sweetie_bot_robot_model/robot_model.hpp>

namespace sweetie_bot {
namespace motion {
namespace controller {

/*static inline std::ostream& operator<<(std::ostream& s, const KDL::Vector& v) 
{
	s << "[" << v.x() << " " << v.y() << " " << v.z() << " ]";
	return s;
}*/

static double angularDistance(const KDL::Rotation& rot, const geometry_msgs::Quaternion& quat) 
{
	Eigen::Quaterniond w1;
	rot.GetQuaternion(w1.x(), w1.y(), w1.z(), w1.w());
	w1.normalize();  // fix for incorrect kdl::Rotation::GetQuaternion conversation
	Eigen::Quaterniond w2(quat.w, quat.x, quat.y, quat.z);
	w2.normalize();
	return abs(w1.angularDistance(w2));
}

static double angularDistance(const KDL::Rotation& rot1, const KDL::Rotation& rot2) 
{
	Eigen::Quaterniond w1, w2;
	rot1.GetQuaternion(w1.x(), w1.y(), w1.z(), w1.w());
	w1.normalize(); // fix for incorrect kdl::Rotation::GetQuaternion conversation
	rot2.GetQuaternion(w2.x(), w2.y(), w2.z(), w2.w());
	w2.normalize(); // fix for incorrect kdl::Rotation::GetQuaternion conversation
	return w1.angularDistance(w2);
}

static double distance2(const KDL::Vector& v1, const geometry_msgs::Point v2) {
	return (v1.x() - v2.x)*(v1.x() - v2.x) + (v1.y() - v2.y)*(v1.y() - v2.y) + (v1.z() - v2.z)*(v1.z() - v2.z);
}

static double distance(const KDL::Vector& v1, const geometry_msgs::Point& v2) {
	return std::sqrt(distance2(v1, v2));
}

static double distance(const KDL::Vector& v1, const KDL::Vector& v2) {
	return (v1 - v2).Norm();
}

/*static bool CartesianTrajectoryCache::checkRigidBodyTrajectoryPosVelContact(const RigidBodyTrajectory& msg, int n) 
{
	if (msg.position.size() != n) return false;
	if (msg.orientation.size() != n) return false;
	if (msg.velocity.size() != n) return false;
	if (msg.contact.size() != n) return false;
	return true;
}*/
	

CartesianTrajectoryCache::CartesianTrajectoryCache(FollowStepSequenceGoal_const_shared_ptr goal, RobotModel * robot_model, double period)
{
	if (!robot_model || !robot_model->ready()) throw std::invalid_argument("CartesianTrajectoryCache: RobotModel is not ready");

	// append flag is not supported
	if (goal->append) {
		throw std::invalid_argument("CartesianTrajectoryCache: append flag is not supported.");
	}
	// get number of end effectors
	int n_chains = goal->ee_motion.size();
	if (n_chains < 1) {
		throw std::invalid_argument("CartesianTrajectoryCache: at least one end effector must be specified.");
	}
	// get number of points
	int n_points = goal->time_from_start.size();;
	if (n_points < 2) {
		throw std::invalid_argument("CartesianTrajectoryCache: trajectory must contain at least two points.");
	}
	// check if sizes are consistent
	if (goal->base_motion.points.size() != 0 && goal->base_motion.points.size() != n_points) {
		throw std::invalid_argument("CartesianTrajectoryCache: incorrect number of points in base motion.");
	}
	// process chains
	this->limb_index_fullpose.reserve(n_chains);
	this->contact_name.reserve(n_chains);
	for (int k = 0; k < n_chains; k++) {
		// check chain size
		if (goal->ee_motion[k].points.size() != n_points) {
			throw std::invalid_argument("CartesianTrajectoryCache: incorrect number of points in ee motion.");
		}
		// chains must be registered in robot model
		int index = robot_model->getChainIndex(goal->ee_motion[k].name);
		if (index < 0) {
			throw std::out_of_range("CartesianTrajectoryCache: unknown kinematic chain name: " + goal->ee_motion[k].name);
		}
		this->limb_index_fullpose.push_back(index);
		// save default contact names
		this->contact_name.push_back(robot_model->getChainDefaultContact(goal->ee_motion[k].name));
	}
	// time intervals must be equal to period
	for(int k = 0; k < n_points; k++) {
		if (std::abs(goal->time_from_start[k] - period*k) > 0.01*period*k ) { // 1% error tolerance
			throw std::invalid_argument("CartesianTrajectoryCache: period is not equal to component period.");
		}
	}

	// state initialization
	this->step_sequence_list.push_back(goal);
	this->time_step = 0;
}


bool CartesianTrajectoryCache::step() 
{
	// check current trajectory chunk is not finished
	if (time_step < step_sequence_list.front()->time_from_start.size() - 1) {
		time_step++;
		return true;
	}
	// check if next chunk is available 
	if (step_sequence_list.size() > 1) {
		step_sequence_list.pop_front();
		time_step = 0;
		return true;
	}
	// do not touch marker
	return false;
}

void CartesianTrajectoryCache::prepareEndEffectorStateBuffer(RigidBodyState& limbs) const
{
	const FollowStepSequenceGoal& msg = *step_sequence_list.front();
	int n_limbs = msg.ee_motion.size();
	limbs.name.clear(); 
	limbs.name.reserve(n_limbs);
	for(int k = 0; k < n_limbs; k++) limbs.name.push_back(msg.ee_motion[k].name);
	limbs.frame.resize(n_limbs);
	limbs.twist.resize(n_limbs);
	limbs.wrench.clear();
}

std::vector<std::string> CartesianTrajectoryCache::getRequiredChains() const 
{
	const FollowStepSequenceGoal& msg = *step_sequence_list.front();
	std::vector<std::string> names;
	names.reserve( msg.ee_motion.size() );
	for(const auto& ee : msg.ee_motion) names.push_back(ee.name);
	return names;
}

bool CartesianTrajectoryCache::getBaseState(RigidBodyState& base) const
{
	const FollowStepSequenceGoal& msg = *step_sequence_list.front();
	if (msg.base_motion.points.size() == 0) return false;
	tf::poseMsgToKDL(msg.base_motion.points[time_step].pose, base.frame[0]);
	base.twist[0] = msg.base_motion.points[time_step].twist;
	return true;
}

void CartesianTrajectoryCache::getEndEffectorState(RigidBodyState& limbs) const
{
	const FollowStepSequenceGoal& msg = *step_sequence_list.front();
	int n_limbs = msg.ee_motion.size();
	for(int k = 0; k < n_limbs; k++) {
		tf::poseMsgToKDL(msg.ee_motion[k].points[time_step].pose, limbs.frame[k]);
		limbs.twist[k] = msg.ee_motion[k].points[time_step].twist;
	}
}

int CartesianTrajectoryCache::checkPathToleranceFullpose(const RigidBodyState& limbs_full) const 
{
	const FollowStepSequenceGoal& msg = *step_sequence_list.front();
	int n_limbs = msg.ee_motion.size();
	// check end_effector pose: both poses are presented in base_link frame
	for(int k = 0; k < n_limbs; k++) {
		/*Eigen::Quaterniond w1;
		limbs_full.frame[limb_index_fullpose[k]].M.GetQuaternion(w1.x(), w1.y(), w1.z(), w1.w());
		w1.normalize();  // fix for incorrect kdl::Rotation::GetQuaternion conversation
		std::cout <<  limb_index_fullpose[k] <<  "real.p = " << limbs_full.frame[limb_index_fullpose[k]].p << " desired.p =" <<  msg.ee_motion[k].points[time_step].pose.position << std::endl;
		std::cout << "real.angle = " << w1.coeffs().transpose() << " desired.angle = " << msg.ee_motion[k].points[time_step].pose.orientation << " angle_error = " << angularDistance( limbs_full.frame[limb_index_fullpose[k]].M, msg.ee_motion[k].points[time_step].pose.orientation ) << std::endl;*/

		if ( distance( limbs_full.frame[limb_index_fullpose[k]].p, msg.ee_motion[k].points[time_step].pose.position ) > msg.position_tolerance ) return limb_index_fullpose[k];
		if ( angularDistance( limbs_full.frame[limb_index_fullpose[k]].M, msg.ee_motion[k].points[time_step].pose.orientation ) > msg.orientation_tolerance ) return limb_index_fullpose[k];
	}
	return -1;	
}	
	
	
int CartesianTrajectoryCache::checkPathTolerance(const RigidBodyState& limbs_real, const RigidBodyState& limbs_desired) const
{
	const FollowStepSequenceGoal& msg = *step_sequence_list.front();
	int n_limbs = msg.ee_motion.size();
	// check end_effector pose: both poses are presented in base_link frame
	for(int k = 0; k < n_limbs; k++) {
		if ( distance( limbs_real.frame[k].p, limbs_desired.frame[k].p ) > msg.position_tolerance ) return k;
		if ( angularDistance( limbs_real.frame[k].M, limbs_desired.frame[k].M ) > msg.orientation_tolerance ) return k;
	}
	return -1;	
}

void CartesianTrajectoryCache::selectEndEffector(const RigidBodyState& in_sorted, RigidBodyState& out) const
{
	const FollowStepSequenceGoal& msg = *step_sequence_list.front();
	int n_limbs = msg.ee_motion.size();
	for(int k = 0; k < n_limbs; k++) {
		out.frame[k] = in_sorted.frame[limb_index_fullpose[k]];
		out.twist[k] = in_sorted.twist[limb_index_fullpose[k]];
	}	
}
		

void CartesianTrajectoryCache::prepareSupportStateBuffer(SupportState& support) const
{
	const FollowStepSequenceGoal& msg = *step_sequence_list.front();
	int n_limbs = msg.ee_motion.size();
	support.name.clear(); 
	support.name.reserve(n_limbs);
	for(int k = 0; k < n_limbs; k++) support.name.push_back(msg.ee_motion[k].name);
	support.contact = contact_name;
	support.support.resize(n_limbs);
}

void CartesianTrajectoryCache::getSupportState(SupportState& support) const
{
	const FollowStepSequenceGoal& msg = *step_sequence_list.front();
	int n_limbs = msg.ee_motion.size();
	for(int k = 0; k < n_limbs; k++) {
		support.support[k] = msg.ee_motion[k].points[time_step].contact ? 1.0 : 0.0;
	}
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

