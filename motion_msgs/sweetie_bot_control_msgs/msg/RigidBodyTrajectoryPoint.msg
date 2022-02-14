#
# Point of rigid body trajectory. 
# 
# * pose --- pose specification
# * twist --- velocity specification (screw twist)
# * accel --- acceleraton 
# * contact --- for end effectors means that it contacts support surface.
#

geometry_msgs/Pose pose
kdl_msgs/Twist twist
kdl_msgs/Twist accel
bool contact

