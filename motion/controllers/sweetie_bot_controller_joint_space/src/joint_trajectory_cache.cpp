#include "joint_trajectory_cache.hpp"

#include <limits>
#include <cstddef>
#include <bitset>

#include <sweetie_bot_robot_model/robot_model.hpp>

namespace sweetie_bot {
namespace motion {
namespace controller {

JointTrajectoryCache::JointTrajectoryCache(const control_msgs::FollowJointTrajectoryGoal& goal, RobotModel * robot_model)
{

	if (!robot_model || !robot_model->ready()) throw std::invalid_argument("JointTrajectoryCache: RobotModel is not ready");

	// get list of kinematic chains
	const unsigned int max_chains = 32;
	const std::vector<std::string> chains_list = robot_model->listChains();
	if (chains_list.size() >= max_chains) throw std::invalid_argument("JointTrajectoryCache: too many kineamtic chains (more then 32)");

	// split received joints in two groups: actual robot joint and support points names, prepended with substring "support".
	std::vector<bool> support_flags(goal.trajectory.joint_names.size());	// support points are market by true 
	std::bitset<max_chains> support_chains_flags; // support chains set

	for(int i = 0; i < goal.trajectory.joint_names.size(); i++) {
		const std::string& name = goal.trajectory.joint_names[i];
		if (0 == name.compare(0, 8, "support/")) {
			// this joint contains support state information
			int index = robot_model->getChainIndex(name.substr(8));
			if (index < 0) throw std::invalid_argument("JointTrajectoryCache: unknown chain in support state: " + name);
			support_chains_flags.set(index, true);
			support_names.push_back(name.substr(8));
			support_flags[i] = true;
		}
		else {
			// ordinary joint
			int index = robot_model->getJointIndex(name);
			if (index < 0) throw std::invalid_argument("JointTrajectoryCache: Unknown joint: " + name);
			names.push_back(name);
			index_fullpose.push_back(index);
			support_flags[i] = false;
		}
	}
	// get chains list for subsequent resource requests.
	{ 
		 // get controlled chains from joint list
		this->chains = robot_model->getJointsChains(this->names);
		std::bitset<max_chains> controlled_chains_flags; // controlled chains market by true
		for ( const string& name : this->chains) {
			// std::cout << "name: " << name << " index: " << robot_model->getChainIndex(name) << std::endl;
			controlled_chains_flags.set(robot_model->getChainIndex(name), true);
		}
		// std::cout << "controlled_chains_flags: " << controlled_chains_flags << " support_chains_flags: " << support_chains_flags << std::endl;

		std::bitset<max_chains> common_chains_flags = controlled_chains_flags & support_chains_flags;
		// Calculate the set of support chains that should be added to required resourses list.
		support_chains_flags ^= common_chains_flags;
		// Calculate the set of controlled chains that is not mentioned in supports.
		// They are added to support list and assumed to be free.
		controlled_chains_flags ^= common_chains_flags;
		// std::cout << "controlled_chains_flags: " << controlled_chains_flags << " support_chains_flags: " << support_chains_flags << " common_chains_flags: " << common_chains_flags << std::endl;
		//
		for(int i = 0; i < chains_list.size(); i++) {
			if (support_chains_flags[i]) this->chains.push_back(chains_list[i]);
			if (controlled_chains_flags[i]) this->support_names.push_back(chains_list[i]);
		}
	}
	// now extract and interpolate trajectory
	loadTrajectory(goal.trajectory, support_flags, robot_model);
	// get tolerances from message
	getJointTolerance(goal);
}


/**
 * interpolate trajectory and cache support state buffer
 */
void JointTrajectoryCache::loadTrajectory(const trajectory_msgs::JointTrajectory& trajectory, const std::vector<bool>& support_flags, RobotModel * robot_model) 
{
	int n_joints = names.size();
	int n_supports = support_names.size();
	int n_samples = trajectory.points.size();
	if (n_samples < 2) throw::std::invalid_argument("JointTrajectoryCache: trajectory must contains at least 2 samples.");
	if (trajectory.points[0].time_from_start.toSec() != 0) throw::std::invalid_argument("JointTrajectoryCache: trajectory start time is not equal to zero.");
	// std::cout << "n_supports = " << n_supports << " n_joints = " << n_joints << std::endl;

	// get contacts names and default contacts
	std::vector<std::string> contacts_list = robot_model->listContacts();
	std::vector<std::string> default_contacts_list;
	default_contacts_list.reserve(n_supports);
	for (int i = 0; i < n_supports; i++) default_contacts_list.push_back( robot_model->getChainDefaultContact(support_names[i]) );

	alglib::real_1d_array t;
	std::vector<alglib::real_1d_array> joint_trajectory(n_joints);
	//allocate memory
	t.setlength(n_samples);
	for(int joint = 0; joint < n_joints; joint++) joint_trajectory[joint].setlength(n_samples);
	this->support_points.resize(n_samples);
	//copy trajectory data
	const double t_fix_step = 0.001; // default step if incorrect time sequence is supplied
	double t_prev = -t_fix_step;
	for(int k = 0; k < n_samples; k++) {
		//check joint number
		if (trajectory.points[k].positions.size() != trajectory.joint_names.size()) throw::std::invalid_argument("JointTrajectoryCache: malformed trajectory_msgs::JointTrajectory.");
		//copy time
		t[k] = trajectory.points[k].time_from_start.toSec();
		// fix incorrect time
		//TODO is there any better way to handle trajectories with zero execution time?
		if (t_prev >= t[k]) t[k] = t_prev + t_fix_step;
		t_prev = t[k];
		support_points[k].t = t[k];
		// copy joint postions
		int joint = 0;
		int support = 0;
		//TODO reserve
		support_points[k].support.resize(n_supports);
		support_points[k].contact.resize(n_supports);
		for(int i = 0; i < trajectory.joint_names.size(); i++) {
			if (support_flags[i]) {
				double value = trajectory.points[k].positions[i];
				if (value < 0.0) value = 0.0;
				int index = std::floor(value);

				if (value <= 1.0 || index >= contacts_list.size()) {
					// default contact 
					support_points[k].support[support] = value;
					support_points[k].contact[support] = default_contacts_list[support];
				}
				else {
					// get contact form index
					support_points[k].support[support] = value - index;
					support_points[k].contact[support] = contacts_list[index];
				}
				support++;
			}
			else {
				joint_trajectory[joint][k] = trajectory.points[k].positions[i];
				joint++;
			}
		}
		// pad supports with zeros
		for(; support < n_supports; support++) support_points[k].support[support] = 0.0;
	}
	//perform interpolation
	this->joint_splines.resize(n_joints);
	for(int joint = 0; joint < n_joints; joint++) {
		// alglib::spline1dbuildcubic(t, joint_trajectory[joint], n_samples, 1, 0.0, 1, 0.0, this->joint_splines[joint]);
		alglib::spline1dbuildakima(t, joint_trajectory[joint], this->joint_splines[joint]);
		// velocity an acceleration is ignored
	}
	this->goal_time = trajectory.points[n_samples-1].time_from_start.toSec();
}

/**
 * get trajectory tolerance
 */
void JointTrajectoryCache::getJointTolerance(const FollowJointTrajectoryGoal& goal)
{
	int n_joints = names.size();
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

void JointTrajectoryCache::prepareSupportStateBuffer(SupportState& support) const
{
	support.name = support_names;
	support.support.resize(support_names.size());
	support.contact.resize(support_names.size());
}

void JointTrajectoryCache::getSupportState(double t, SupportState& support) const
{
	// find index of the last element which is less then t
	int index;
	auto it = upper_bound(support_points.begin(), support_points.end(), t, [](double t, const SupportPoint& p) { return t < p.t; });
	if (it == support_points.begin()) index = 0;
	else index = it - support_points.begin() - 1;
	// copy support state
	copy(support_points[index].support.begin(), support_points[index].support.end(), support.support.begin());
	copy(support_points[index].contact.begin(), support_points[index].contact.end(), support.contact.begin());
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

