#ifndef  JOINT_TRAJECTORY_CACHE_HPP
#define  JOINT_TRAJECTORY_CACHE_HPP

#include <interpolation.h>

#include <sensor_msgs/typekit/JointState.h>
#include <control_msgs/typekit/FollowJointTrajectoryGoal.h>
#include <sweetie_bot_kinematics_msgs/typekit/SupportState.h>


namespace sweetie_bot {
namespace motion {

class RobotModel;

namespace controller {


/**
 * @brief FollowJointTrajectoryGoal cache.
 * Approximate trajectory with 2-nd order spline, cache up joint names,  tolerance and joint indexes.
 * Provide methods for easy tolerance check and intermedia points generation.
 **/
class JointTrajectoryCache 
{
	public:
		typedef sensor_msgs::JointState JointState;
		typedef control_msgs::FollowJointTrajectoryGoal FollowJointTrajectoryGoal;
		typedef control_msgs::JointTolerance JointTolerance;
		typedef sweetie_bot_kinematics_msgs::SupportState SupportState;
		typedef alglib::spline1dinterpolant JointSpline;
		
	protected:
		struct SupportPoint {
			double t;
			std::vector<double> support;
			std::vector<std::string> contact;
		};

	protected:

		std::vector<std::string> names;
		std::vector<std::string> chains;
		std::vector<int> index_fullpose;
		double goal_time;

		std::vector<JointSpline> joint_splines;

		std::vector<std::string> support_names;
		std::vector<SupportPoint> support_points;

		std::vector<double> path_tolerance; 
		std::vector<double> goal_tolerance;
		double goal_time_tolerance;
		
	protected:
		void loadTrajectory(const trajectory_msgs::JointTrajectory& trajectory, const std::vector<bool>& support_flags, RobotModel * robot_model, double threshold);
		void getJointTolerance(const FollowJointTrajectoryGoal& msg);

	public:
		/**
		 * @brief Make joint space trajectory cache
		 * Approximate trajectory with 2-nd order spline, cache up joint names,  tolerance and joint indexes.
		 * @param _trajectory ROS message with goal trajectory.
		 * @param robot_mode RobotModel to interprete joints names.
		 **/
		JointTrajectoryCache(const FollowJointTrajectoryGoal& _trajectory, RobotModel * robot_model, double threshold);

		/**
		 * @brief Prepare joint state buffer
		 * Set joints names, resize arrays.
		 * @param joints Joint state buffer.
		 **/
		void prepareJointStateBuffer(JointState& joints) const;

		/**
		 * @brief Return trajectory duration.
		 * @return Movement duration in seconds.
		 **/
		double getGoalTime() const { 
			return this->goal_time; 
		}

		/**
		 * @brief Return lists of required kinematics chains (according to RobotModel).
		 * @return List of kinematics chains.
		 **/
		const std::vector<std::string>& getRequiredChains() const {
			return this->chains;
		}

		/**
		 * @brief Get intermedia point of trajectory to buffer of proper size.
		 * Method does not check joint state size. 
		 * @param joints Joint state buffer receiving new state
		 * @param t time in seconds from begining of movment.
		 * @return true if time @a t in range of trajectory.
		 **/
		bool getJointState(double t, JointState& joints) const;

		/*
		 * @brief Simplifies check of start conditions: check path trajectory for specified time.
		 * @param joints_real_sorted  Full pose in joint space sorted according to RobotModel.
		 * @return full pose index of joint which violates tolerance, -1 otherwise
		 **/
		int checkPathToleranceFullpose(const JointState& joints_real_sorted, double t = 0.0) const;

		/*
		 * @brief Check if given pose is within defined path tolerance.
		 * Check path_tolerance. Both poses must have the same layout as trajectory.
		 * @return index of joint which violates tolerance, -1 otherwise
		 **/
		int checkPathTolerance(const JointState& joints_real, const JointState& joints_desired) const;

		/*
		 * @brief Check if given pose is within defined goal tolerance.
		 * Check goal_tolerance. Both poses must have the same layout as trajectory.
		 * @return index of joint which violates tolerance, -1 otherwise
		 **/
		int checkGoalTolerance(const JointState& joints_real, const JointState& joints_desired) const;

		/*
		 * @brief Check if goal time tolearnce is violated.
		 * If time constraints is violated return positive value, zero or negative otherwise.
		 * @return duration on which current time exceeds time limit.
		 **/
		double checkGoalTimeTolerance(double t) const {
			return t - (this->goal_time + this->goal_time_tolerance);
		}

		/*
		 * @brief Select trajectory joints from full robot pose. Use joint index to extract joints.
		 * Method does  not perform size checks and does not set joints names. Use @c prepareJointStateBuffer() to prepare it.
		 * @param in_sorted Full pose in joint space sorted according to RobotModel.
		 * @param out State of trajectory joints.k
		 **/
		void selectJoints(const JointState& in_sorted, JointState& out) const;

		/**
		 * @brief Prepare SupportState message.
		 * Resize arrays, set names.
		 * @param support message buffer,
		 **/
		void prepareSupportStateBuffer(SupportState& support) const;

		/**
		 * @brief Get supports' state to buffer of proper size.
		 * Method does not check message
		 * @param joints Support state buffer receiving new state
		 * @param t time in seconds from begining of movement.
		 **/
		void getSupportState(double t, SupportState& support) const;
};


} // namespace controller
} // namespace motion
} // namespace sweetie_bot


#endif  /*JOINT_TRAJECTORY_CACHE_HPP*/

