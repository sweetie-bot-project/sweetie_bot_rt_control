#ifndef  SWEETIE_BOT_CONTROLLER_CARTESIAN_FILTER_RIGID_BODY_STATE_HPP
#define  SWEETIE_BOT_CONTROLLER_CARTESIAN_FILTER_RIGID_BODY_STATE_HPP

#include <sweetie_bot_kinematics_msgs/RigidBodyState.h>

namespace sweetie_bot {
namespace motion {
namespace filter {

class FilterRigidBodyStateInterface 
{
	public:
		typedef sweetie_bot_kinematics_msgs::RigidBodyState RigidBodyState;

	public:
		virtual bool reset(const RigidBodyState& state0, double period) = 0;
		virtual bool update(const RigidBodyState& state, const RigidBodyState& ref, RigidBodyState& new_state) = 0;
};

} // namespace filter
} // namespace sweetie_bot 
} // namespace motion 


#endif  /*SWEETIE_BOT_CONTROLLER_CARTESIAN_FILTER_RIGID_BODY_STATE_HPP*/
