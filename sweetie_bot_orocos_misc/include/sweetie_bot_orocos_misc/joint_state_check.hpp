#ifndef  JOINT_STATE_CHECK_HPP
#define  JOINT_STATE_CHECK_HPP

#include <sensor_msgs/JointState.h>

namespace sweetie_bot {

/**
 * Checks if all nonzero length fields of JointState have equal sizes. Does not check @a effort.
 * If @a sz is provided additinally check if their size is equal to @a sz.
 * @param sz expected number of joints
 */
bool isValidJointState(const sensor_msgs::JointState& msg, int sz = -1);

/**
 * Checks if all nonzero length fields of JointState have equal sizes. Field @a position must present. Does not check @a effort.
 * If @a sz is provided additinally check if their size is equal to @a sz.
 * @param sz expected number of joints
 */
bool isValidJointStatePos(const sensor_msgs::JointState& msg, int sz = -1);

/**
 * Checks if all nonzero length fields of JointState have equal sizes. Fields @a position and @a velocity must present. Does not check @a effort.
 * If @a sz is provided additinally check if their size is equal to @a sz.
 * @param sz expected number of joints
 */
bool isValidJointStatePosVel(const sensor_msgs::JointState& msg, int sz = -1);

/**
 * Checks if all nonzero length fields of JointState have equal sizes. Fields @a name and @a position must present. Does not check @a effort.
 * If @a sz is provided additinally check if their size is equal to @a sz.
 * @param sz expected number of joints
 */
bool isValidJointStateNamePos(const sensor_msgs::JointState& msg, int sz = -1);

/**
 * Checks if all nonzero length fields of JointState have equal sizes. Fields @a name, @a position and @a velocity must present. Does not check @a effort.
 * If @a sz is provided additinally check if their size is equal to @a sz.
 * @param sz expected number of joints
 */
bool isValidJointStateNamePosVel(const sensor_msgs::JointState& msg, int sz = -1);

} // namespace sweetie_bot

#ifndef MESSAGE_CHECKS_INLINE
#define MESSAGE_CHECKS_INLINE 1
#endif

#if MESSAGE_CHECKS_INLINE
#include "joint_state_check_impl.hpp"
#endif 

#endif  /*JOINT_STATE_CHECK_HPP*/
