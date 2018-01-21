#include <sweetie_bot_orocos_misc/joint_state_check.hpp>
#include <sweetie_bot_orocos_misc/message_checks.hpp>

#if (!MESSAGE_CHECKS_INLINE)
#include <sweetie_bot_orocos_misc/message_checks_impl.hpp>
#include <sweetie_bot_orocos_misc/joint_state_check_impl.hpp>
#endif

namespace sweetie_bot {

bool isValidJointState(const sensor_msgs::JointState& msg, int sz)
{
	if (msg.name.size() != 0) {
		if (sz < 0) sz = msg.name.size();
		else if (sz != msg.name.size()) return false;
	}
	if (msg.position.size() != 0) {
		if (sz < 0) sz = msg.position.size();
		else if (sz != msg.position.size()) return false;
	}
	if (msg.velocity.size() != 0) {
		if (sz < 0) sz = msg.velocity.size();
		else if (sz != msg.velocity.size()) return false;
	}
	if (msg.effort.size() != 0) {
		if (sz < 0) sz = msg.effort.size();
		else if (sz != msg.effort.size()) return false;
	}
	return true;
}

} // namespace sweetie_bot
