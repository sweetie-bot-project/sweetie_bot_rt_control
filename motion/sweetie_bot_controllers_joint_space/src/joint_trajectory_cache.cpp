#include "joint_trajectory_cache.hpp"

#include <limits>
#include <cstddef>

#include "sweetie_bot_robot_model/robot_model-simple.hpp"

namespace sweetie_bot {
namespace motion {
namespace controller {

JointTrajectoryCache::JointTrajectoryCache(const control_msgs::FollowJointTrajectoryGoal& goal, RobotModel * robot_model)
{
	if (!robot_model || !robot_model->ready()) throw std::invalid_argument("JointTrajectoryCache: RobotModel is not ready");

	int n_joints = goal.trajectory.joint_names.size();
	// exract names
	this->names = goal.trajectory.joint_names;
	// build index
	this->index_fullpose.resize(names.size());
	for(int i = 0; i < names.size(); i++) {
		index_fullpose[i] = robot_model->getJointIndex(names[i]);
		if (index_fullpose[i] < 0) throw std::invalid_argument("JointTrajectoryCache: Unknown joint: " + this->names[i]);
	}
	// get chains list for subsequent resource requests.
	this->chains = robot_model->getJointChains(this->names);
	// now extract and interpolate trajectory
	interpolateTrajectory(goal.trajectory);
	// cache tolerance: speed and acceleration is ignored
	this->goal_tolerance.resize(n_joints);
	this->path_tolerance.resize(n_joints);
	// now only same order and same size tolerance values are supported
	// TODO: add velocity tolerance support
	// TODO: allow random joints order
	// TODO: default tolerance parameteres
	if (goal.path_tolerance.size() == 0 && goal.goal_tolerance.size() == 0) {
		// no tolerance limits provided. Set then to infinite
		for(int joint = 0; joint < n_joints; joint++) {
			this->path_tolerance[joint] = std::numeric_limits<double>::max();
			this->goal_tolerance[joint] = std::numeric_limits<double>::max();
		}
	}
	else {
		// check tolerance limits arrays
		if (goal.path_tolerance.size() != n_joints || goal.goal_tolerance.size() != n_joints) throw std::invalid_argument("JointTrajectoryCache: inconsitent tolerance array size.");
		for(int joint = 0; joint < n_joints; joint++) {
			if (goal.path_tolerance[joint].name != names[joint]) throw std::invalid_argument("JointTrajectoryCache: inconsitent joint order in path_tolerance.");
			if (goal.goal_tolerance[joint].name != names[joint]) throw std::invalid_argument("JointTrajectoryCache: inconsitent joint order in goal_tolerance.");

			this->path_tolerance[joint] = goal.path_tolerance[joint].position;
			this->goal_tolerance[joint] = goal.goal_tolerance[joint].position;
		}
	}
	// get goal time tolerance
	this->goal_time_tolerance = goal.goal_time_tolerance.toSec();
	if (this->goal_time_tolerance == 0.0) this->goal_time_tolerance = 1.0; // sane value TODO: default tolerance parameteres
	else if (this->goal_time_tolerance < 0.0) this->goal_time_tolerance = std::numeric_limits<double>::max(); // effective infifnity
}

/**
 * interpolate trajectory ans set goal time
 */
void JointTrajectoryCache::interpolateTrajectory(const trajectory_msgs::JointTrajectory& trajectory) 
{
	int n_joints = trajectory.joint_names.size();
	int n_samples = trajectory.points.size();
	if (n_samples < 2) throw::std::invalid_argument("JointTrajectoryCache: trajectory must contains at least 2 samples.");
	if (trajectory.points[0].time_from_start.toSec() != 0) throw::std::invalid_argument("JointTrajectoryCache: trajectory start time is not equal to zero.");

	alglib::real_1d_array t;
	std::vector<alglib::real_1d_array> joint_trajectory(n_joints);
	//allocate memory
	t.setlength(n_samples);
	for(int joint = 0; joint < n_joints; joint++) joint_trajectory[joint].setlength(n_samples);
	//copy trajectory data
	for(int k = 0; k < n_samples; k++) {
		if (trajectory.points[k].positions.size() != n_joints) throw::std::invalid_argument("JointTrajectoryCache: malformed trajectory_msgs::JointTrajectory.");
		t[k] = trajectory.points[k].time_from_start.toSec();
		for(int joint = 0; joint < n_joints; joint++) {
			joint_trajectory[joint][k] = trajectory.points[k].positions[joint];
		}
	}
	//perform interpolation
	this->joint_splines.resize(n_joints);
	for(int joint = 0; joint < n_joints; joint++) {
		alglib::spline1dbuildcubic(t, joint_trajectory[joint], this->joint_splines[joint]);
		// velocity an acceleration is ignored
	}
	this->goal_time = trajectory.points[n_samples-1].time_from_start.toSec();
}

void JointTrajectoryCache::prepareJointStateBuffer(JointState& joints) const
{
	joints.name = names;
	joints.position.resize(names.size());
	joints.velocity.resize(names.size());
	joints.effort.clear();
}

bool JointTrajectoryCache::getJointState(double t, JointState& joints) const
{
	if (t < 0) t = 0;
	else if (t > goal_time) t = goal_time;

	for(int joint = 0; joint < this->joint_splines.size(); joint++) {
		double accel;
		alglib::spline1ddiff(this->joint_splines[joint], t, joints.position[joint], joints.velocity[joint], accel);
	}
	return t == goal_time;
}

int JointTrajectoryCache::checkPathToleranceFullpose(const JointState& joints_real_sorted, double t) const
{
	if (t < 0) t = 0;
	else if (t > goal_time) t = goal_time;
	// check position tolerance
	for(int joint = 0; joint < this->path_tolerance.size(); joint++) {
		double desired_joint_position;
		desired_joint_position = alglib::spline1dcalc(this->joint_splines[joint], 0); 
		if (abs(joints_real_sorted.position[this->index_fullpose[joint]] - desired_joint_position) > this->path_tolerance[joint]) {
			return this->index_fullpose[joint];
		}
	}
	// speed is ignored
	return -1;
}

int JointTrajectoryCache::checkPathTolerance(const JointState& joints_real, const JointState& joints_desired) const
{
	// check position tolerance
	for(int joint = 0; joint < this->goal_tolerance.size(); joint++) {
		if (abs(joints_real.position[joint] - joints_desired.position[joint]) > this->path_tolerance[joint]) {
			return joint;
		}
	}
	// speed is ignored
	return -1;
}

int JointTrajectoryCache::checkGoalTolerance(const JointState& joints_real, const JointState& joints_desired ) const
{
	// check position tolerance
	for(int joint = 0; joint < this->goal_tolerance.size(); joint++) {
		if (abs(joints_real.position[joint] - joints_desired.position[joint]) > this->goal_tolerance[joint]) {
			return joint;
		}
	}
	// speed is ignored
	return -1;
}


void JointTrajectoryCache::selectJoints(const JointState& in_sorted, JointState& out) const
{
	for(int joint = 0; joint < this->index_fullpose.size(); joint++) {
		if (in_sorted.position.size() != 0) out.position[joint] = in_sorted.position[index_fullpose[joint]];
		if (in_sorted.velocity.size() != 0) out.velocity[joint] = in_sorted.velocity[index_fullpose[joint]];
	}
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

