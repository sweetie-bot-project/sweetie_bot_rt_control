#ifndef  MESSAGE_CHECKS_HPP
#define  MESSAGE_CHECKS_HPP

#include <sweetie_bot_kinematics_msgs/SupportState.h>
#include <sweetie_bot_kinematics_msgs/RigidBodyState.h>

namespace sweetie_bot {

/** RIGID BODY STATE */

/**
 * Checks if all nonzero length fields of RigidBodyState have equal sizes. Fields @a name and @a frame must present. 
 * If @a sz is provided additinally check if their size is equal to @a sz.
 * @param sz expected number of elements.
 */
inline bool isValidRigidBodyStateNameFrame(const sweetie_bot_kinematics_msgs::RigidBodyState& msg, int sz = -1);
/**
 * Checks if all nonzero length fields of RigidBodyState have equal sizes. Fields @a name, @a frame and @a twist must present. 
 * If @a sz is provided additinally check if their size is equal to @a sz.
 * @param sz expected number of elements.
 */
inline bool isValidRigidBodyStateNameFrameTwist(const sweetie_bot_kinematics_msgs::RigidBodyState& msg, int sz = -1);
/**
 * Checks if all nonzero length fields of RigidBodyState have equal sizes. Fields @a name, @a frame and @a twist must present. 
 * If @a sz is provided additinally check if their size is equal to @a sz.
 * @param sz expected number of elements.
 */
inline bool isValidRigidBodyStateFrameTwist(const sweetie_bot_kinematics_msgs::RigidBodyState& msg, int sz = -1);

/** SUPPORT STATE */

/**
 * Checks if all nonzero length fields of SupportState have equal sizes. Fields @a name, @a support must present. 
 * If @a sz is provided additinally check if their size is equal to @a sz.
 * @param sz expected number of elements.
 */
inline bool isValidSupportStateNameSupp(const sweetie_bot_kinematics_msgs::SupportState& msg, int sz = -1); 
/**
 * Checks if all nonzero length fields of SupportState have equal sizes. Fields @a name, @a support and @a contact must present. 
 * If @a sz is provided additinally check if their size is equal to @a sz.
 * @param sz expected number of elements.
 */
inline bool isValidSupportStateNameSuppCont(const sweetie_bot_kinematics_msgs::SupportState& msg, int sz = -1);

}

#ifndef MESSAGE_CHECKS_INLINE
#define MESSAGE_CHECKS_INLINE 1
#endif

#if MESSAGE_CHECKS_INLINE
#include "message_checks_impl.hpp"
#endif 

#endif  /*MESSAGE_CHECKS_HPP*/
