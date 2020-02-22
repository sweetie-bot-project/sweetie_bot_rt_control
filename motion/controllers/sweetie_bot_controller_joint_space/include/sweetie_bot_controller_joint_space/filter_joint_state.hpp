#ifndef  FILTER_JOINT_STATE_HPP
#define  FILTER_JOINT_STATE_HPP

#include <orocos/sensor_msgs/typekit/JointState.h>

namespace sweetie_bot {
namespace motion {
namespace filter {

/**
 * @brief Filter interface for JointState message.
 * Filters can be used to smooth a joint trajectory, to concat to trajectories or to bring robot to desired pose.
 **/
class FilterJointStateInterface 
{
	public:
		typedef sensor_msgs::JointState JointState;

	public:
		/**
		 * @brief Reset filter. 
		 * Set internal state of the filter if it presents. Note that this method also performs filter initilization. 
		 * So you must call @a reset() before any @a update() call or if message size has changed.
		 * @param state0 new filter state
		 * @return true on success
		 **/
		virtual bool reset(const JointState& state0, double period) = 0;

		/**
		 * @brief Filtration step. 
		 * Calculate filter output and update filter internal state it presents.
		 * @param state current pose
		 * @param ref desired pose
		 * @param new_state pose calculated by filter. You can use @a state or @a ref to store @a new_state.
		 * @return true on success
		 **/
		virtual bool update(const JointState& state, const JointState & ref, JointState& new_state) = 0;
};

} // namespace filter
} // namespace sweetie_bot 
} // namespace motion 


#endif  /*FILTER_JOINT_STATE_HPP*/
