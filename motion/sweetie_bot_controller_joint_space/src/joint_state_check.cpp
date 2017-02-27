#include "joint_state_check.hpp"

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

bool isValidJointStateNamePos(const sensor_msgs::JointState& msg, int sz)
{
	if (sz < 0) sz = msg.name.size();
	else if (sz != msg.name.size()) return false;
	if (sz != msg.position.size()) return false;
	if (sz != msg.position.size() && msg.position.size() != 0) return false;
	if (sz != msg.velocity.size() && msg.velocity.size() != 0) return false;
	return true;
}

bool isValidJointStatePos(const sensor_msgs::JointState& msg, int sz)
{
	if (sz < 0) sz = msg.position.size();
	else if (sz != msg.position.size()) return false;
	if (sz != msg.name.size() && msg.name.size() != 0) return false;
	if (sz != msg.position.size() && msg.position.size() != 0) return false;
	if (sz != msg.velocity.size() && msg.velocity.size() != 0) return false;
	return true;
}
