#ifndef  TRANSIENT_JOINT_STATE_HPP
#define  TRANSIENT_JOINT_STATE_HPP

#include <orocos/sensor_msgs/typekit/JointState.h>

namespace sweetie_bot {
namespace motion {
namespace filter {

class TransientJointStateInterface 
{
	public:
		typedef sensor_msgs::JointState JointState;

	public:
		virtual bool reset(const JointState& state0, double period) = 0;
		virtual bool update(const JointState& state, const JointState & ref, JointState& new_state) = 0;
};

} // namespace filter
} // namespace sweetie_bot 
} // namespace motion 


#endif  /*TRANSIENT_JOINT_STATE_HPP*/
