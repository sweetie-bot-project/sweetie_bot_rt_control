#ifndef  JOINT_TRAJECTORY_CACHE_HPP
#define  JOINT_TRAJECTORY_CACHE_HPP

#include <sensor_msgs/typekit/JointState.h>
#include <sweetie_bot_kinematics_msgs/typekit/SupportState.h>
#include <sweetie_bot_kinematics_msgs/typekit/RigidBodyState.h>
#include <sweetie_bot_control_msgs/typekit/FollowStepSequenceAction.h>

namespace sweetie_bot {
namespace motion {

class RobotModel;

namespace controller {


/**
 * @brief CartesianRobotState cache. 
 * Store supplied messages "as is" using shared pointer. Object includes time marker.
 *
 * TODO In future versions should also support:
 * * messages appending
 * * hermite interpolation if period does not match
 * * restore velocity by spline 
 *
 * TODO base link processing, cahin index access, limb name vector bufferizaion
 **/
class CartesianTrajectoryCache 
{
	public:
		typedef sweetie_bot_kinematics_msgs::RigidBodyState RigidBodyState;
		typedef sweetie_bot_control_msgs::FollowStepSequenceGoal FollowStepSequenceGoal;
		typedef sweetie_bot_kinematics_msgs::SupportState SupportState;
		typedef boost::shared_ptr<const FollowStepSequenceGoal> FollowStepSequenceGoal_const_shared_ptr;
		typedef std::list<FollowStepSequenceGoal_const_shared_ptr> StepSequenceList;

	protected:
		StepSequenceList step_sequence_list; /**< Contains trajectory chunks. */
		int time_step; /**< Contains number of point in step_sequence_list.front(). Always valid. */
		std::vector<int> limb_index_fullpose; /**< index of coresponding chain in fullpose */
		std::vector<std::string> contact_name; /**< buffers default contact names from robot model */

	public:
		/**
		 * @brief Create new trajectory from sweetie_bot_control_msgs::FollowStepSequenceGoal message.
		 * Check message consistency and throw std::invalid_argument if check failed.
		 * @param trajectory Pointer to goal message. Shared pointer is used to minimize number of copy operations.
		 * @param robot_model Pointer to RobotModel.
		 * @param period Discretizaion period of trajectory.
		 **/
		CartesianTrajectoryCache(FollowStepSequenceGoal_const_shared_ptr trajectory, RobotModel * robot_model, double period);

		
		/**
		 * @brief Move time marker one period in future.
		 * Move time marker one period ahead.
		 * @return false if trajectory is finished
		 **/
		bool step();

		/**
		 * @brief Prepare message buffers.
		 * Set names, resize arrays.
		 * @param limbs Buffer to be prepared.
		 **/
		void prepareEndEffectorStateBuffer(RigidBodyState& limbs) const;

		/**
		 * @brief Return lists of required kinematics chains (according to RobotModel).
		 * @return List of kinematics chains.
		 **/
		std::vector<std::string> getRequiredChains() const;

		/**
		 * @brief Return lists of required kinematics chains (according to RobotModel).
		 * @return List of kinematics chains.
		 **/
		double getTime() const {
			return step_sequence_list.front()->time_from_start[time_step];
		}

		/**
		 * @brief Copy desired base state in buffer of proper size.
		 * Method does not check buffer size. 
		 * @param base RigidBodyState buffer receiving new base. It size must be at least one.
		 * @return true if trajectory contains information about base position, false is returned otherwise.
		 **/
		bool getBaseState(RigidBodyState& base) const;

		/**
		 * @brief Copy desired end effector locations in buffer of proper size.
		 * Method does not check buffer size. 
		 * @param limbs RigidBodyState buffer receiving new state represented in base link frame.
		 **/
		void getEndEffectorState(RigidBodyState& limbs) const;

		/*
		 * @brief Simplifies check of start conditions: check path tolerance at current time.
		 * @param joints_real_sorted  Full pose in joint space sorted according to RobotModel.
		 * @return index of kinematic chain which violates tolerance, -1 otherwise
		 **/
		int checkPathToleranceFullpose(const RigidBodyState& limbs_real_sorted) const;

		/*
		 * @brief Check if given pose is within defined path tolerance.
		 * Check path_tolerance. Both poses must have the same layout as trajectory.
		 * @return index of kinematic chain which violates tolerance, -1 otherwise
		 **/
		int checkPathTolerance(const RigidBodyState& joints_real, const RigidBodyState& joints_desired) const;

		/*
		 * @brief Select kinematic chains from full robot pose. 
		 * Method does  not perform size checks and does not set kinematic chains' names. Use @c prepareEndEffectorStateBuffer() to prepare it.
		 * @param in_sorted Full pose  sorted according to RobotModel.
		 * @param out Selected kinematic chains.
		 **/
		void selectEndEffector(const RigidBodyState& in_sorted, RigidBodyState& out) const;

		/**
		 * @brief Prepare SupportState message.
		 * Resize arrays, set names.
		 * @param support message buffer,
		 **/
		void prepareSupportStateBuffer(SupportState& support) const;

		/**
		 * @brief Get supports' state to buffer of proper size.
		 * Method does not check message
		 * @param supports Support state buffer receiving new state
		 **/
		void getSupportState(SupportState& support) const;
};


} // namespace controller
} // namespace motion
} // namespace sweetie_bot


#endif  /*JOINT_TRAJECTORY_CACHE_HPP*/

