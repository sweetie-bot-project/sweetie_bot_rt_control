namespace sweetie_bot {

/*
 * JOINT STATE
 */

inline bool isValidJointStateNamePos(const sensor_msgs::JointState& msg, int sz)
{
	if (sz < 0) sz = msg.name.size();
	else if (sz != msg.name.size()) return false;
	if (sz != msg.position.size()) return false;
	if (sz != msg.velocity.size() && msg.velocity.size() != 0) return false;
	return true;
}

inline bool isValidJointStateNamePosVel(const sensor_msgs::JointState& msg, int sz)
{
	if (sz < 0) sz = msg.name.size();
	else if (sz != msg.name.size()) return false;
	if (sz != msg.position.size()) return false;
	if (sz != msg.velocity.size()) return false;
	return true;
}

inline bool isValidJointStatePos(const sensor_msgs::JointState& msg, int sz)
{
	if (sz < 0) sz = msg.position.size();
	else if (sz != msg.position.size()) return false;
	if (sz != msg.name.size() && msg.name.size() != 0) return false;
	if (sz != msg.velocity.size() && msg.velocity.size() != 0) return false;
	return true;
}

inline bool isValidJointStatePosVel(const sensor_msgs::JointState& msg, int sz)
{
	if (sz < 0) sz = msg.position.size();
	else if (sz != msg.position.size()) return false;
	if (sz != msg.name.size() && msg.name.size() != 0) return false;
	if (sz != msg.velocity.size()) return false;
	return true;
}

} // namespace sweetie_bot

