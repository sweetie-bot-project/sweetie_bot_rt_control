# Information of robot balance (in world frame).
#  * mass --- mass of the robot.
#  * CoM --- center of mass.
#  * CoP --- center of pressure (all contacts are assumed to be on z=0 level).
#  * ZMP --- zero moment point, may be different from CoP if contacts are unable to produce necessary wrench.
#  * support_points --- contact points (vertrices of support polygone).
#
Header header
float64 mass 
kdl_msgs/Vector CoM
kdl_msgs/Vector CoP
kdl_msgs/Vector ZMP
kdl_msgs/Vector[] support_points
