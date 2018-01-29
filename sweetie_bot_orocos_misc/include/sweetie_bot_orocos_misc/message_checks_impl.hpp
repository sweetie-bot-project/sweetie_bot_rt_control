
namespace sweetie_bot {


/*
 * RIGID BODY STATE 
 */

inline bool isValidRigidBodyStateNameFrame(const sweetie_bot_kinematics_msgs::RigidBodyState& msg, int sz)
{
	if (sz < 0) sz = msg.name.size();
	else if (sz != msg.name.size()) return false;
	if (sz != msg.frame.size()) return false;
	if (sz != msg.twist.size() && msg.twist.size() != 0) return false;
	return true;
}

inline bool isValidRigidBodyStateNameFrameTwist(const sweetie_bot_kinematics_msgs::RigidBodyState& msg, int sz)
{
	if (sz < 0) sz = msg.name.size();
	else if (sz != msg.name.size()) return false;
	if (sz != msg.frame.size()) return false;
	if (sz != msg.twist.size()) return false;
	return true;
}

inline bool isValidRigidBodyStateFrameTwist(const sweetie_bot_kinematics_msgs::RigidBodyState& msg, int sz)
{
	if (sz < 0) sz = msg.frame.size();
	else if (sz != msg.frame.size()) return false;
	if (sz != msg.name.size()) return false;
	if (sz != msg.twist.size()) return false;
	return true;
}


/*
 * SUPPORT STATE
 */

inline bool isValidSupportStateNameSupp(const sweetie_bot_kinematics_msgs::SupportState& msg, int sz) 
{
	if (sz < 0) sz = msg.name.size();
	else if (sz != msg.name.size()) return false;
	if (sz != msg.support.size()) return false;
	if (sz != msg.contact.size() && msg.contact.size() != 0) return false;
	return true;
}

inline bool isValidSupportStateNameSuppCont(const sweetie_bot_kinematics_msgs::SupportState& msg, int sz) 
{
	if (sz < 0) sz = msg.name.size();
	else if (sz != msg.name.size()) return false;
	if (sz != msg.support.size()) return false;
	if (sz != msg.contact.size()) return false;
	return true;
}

/*
 * JOINT STATE ACCEL
 */

inline bool isValidJointStateAccelNamePosVelAccelEffort(const sweetie_bot_kinematics_msgs::JointStateAccel& msg, int sz)
{
	if (sz < 0) sz = msg.name.size();
	else if (sz != msg.name.size()) return false;
	if (sz != msg.position.size()) return false;
	if (sz != msg.velocity.size()) return false;
	if (sz != msg.acceleration.size()) return false;
	if (sz != msg.effort.size()) return false;
	return true;
}

inline bool isValidJointStateAccelPosVelAccel(const sweetie_bot_kinematics_msgs::JointStateAccel& msg, int sz) 
{
	if (sz < 0) sz = msg.position.size();
	else if (sz != msg.position.size()) return false;
	if (sz != msg.name.size() && msg.name.size() > 0) return false;
	if (sz != msg.velocity.size()) return false;
	if (sz != msg.acceleration.size()) return false;
	if (sz != msg.effort.size()) return false;
	return true;

}


} // namespace sweetie_bot

