# State vectors of the group of kinematics chains. It is assumed that all chains are linked to robot frame.
#  * name --- the name of kinematic chain.
#  * position --- the last segment pose in coordinate system of kinematic chain base (robot frame).
#  * speed --- the last segment speed.
#  * force --- external force applied to the last segment in robot frame coordinates (maybe world coordinates?)
#
# All arrays sould have the same size. Presece of limb, position and speed fields is necessary.
Header header
string[] name
geometry_msgs/Pose[] pose
geometry_msgs/Twist[] speed
# Wrench[] force
