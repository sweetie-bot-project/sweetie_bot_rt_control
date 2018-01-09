# Robot weight distributon between legs.
#  * name --- the name of the chain.
#  * support --- leg support coefficients or z-component of reaction force. Support coefficient is the ratio of 
# the weight corresponding to the leg to the total weight of robot.
#  * point --- contact point in coordinates of end effector.
#
# All arrays sould have the same size or contain no elemnts.
#
Header header
string[] name
float64[] support
#kdl_msgs/Vector point
