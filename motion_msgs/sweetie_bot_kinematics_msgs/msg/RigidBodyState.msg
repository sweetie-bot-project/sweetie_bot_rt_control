# State vectors of the group of kinematics chains. It is assumed that all chains are linked to robot frame.
#  * name --- the name of kinematic chain (multiDOF joint).
#  * frame --- the last segment pose in coordinate system of kinematic chain base (robot frame).
#  * twist --- the last segment speed.
#  * wrench --- force applied to the last segment in robot frame coordinates (maybe world coordinates?)
#
# All arrays sould have the same size. Presece of limb, position and speed fields is necessary.
Header header
string[] name
kdl_msgs/Frame[] frame
kdl_msgs/Twist[] twist
#kdl_msgs/Wrench[] wrench
