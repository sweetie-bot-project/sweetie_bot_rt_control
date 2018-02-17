#ifndef  ANIMATION_JOINT_TRAJECTORY_COMPONENT_HPP
#define  ANIMATION_JOINT_TRAJECTORY_COMPONENT_HPP

#include "animation_joint_trajectory-base.hpp"

#include <sweetie_bot_orocos_misc/simple_action_server.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/typekit/FollowJointTrajectoryAction.h>

namespace sweetie_bot {
namespace motion {
namespace controller {

class AnimJointTrajectory : public AnimJointTrajectoryBase
{
	protected:
		// Goal, Feedback, Result typedefs
		ACTION_DEFINITION(control_msgs::FollowJointTrajectoryAction);	
	protected:
		// PROPERTIES
		double setling_time;

	protected:
		// ACTIONLIB:
		// Simple action server
		OrocosSimpleActionServer<control_msgs::FollowJointTrajectoryAction> action_server;
		// new pending goal is received
		void newGoalHook(const Goal& goal);
		// active goal is being canceled
		void cancelGoalHook();
		// buffers (also can be created dynamically)
		Result goal_result;
		Feedback goal_feedback;

	protected:
		// COMPONENT STATE
	public:
		AnimJointTrajectory(std::string const& name);

		bool resourceChangedHook();
		void stopOperationalHook();

		void operationalHook(bool on_target);

		bool configureHook();
		bool startHook();
		void operationalHook();
		void stopHook();
		void cleanupHook();
};

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

#endif  /*ANIMATION_JOINT_TRAJECTORY-COMPONENT_HPP*/
