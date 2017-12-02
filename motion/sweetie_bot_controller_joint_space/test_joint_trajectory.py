#!/usr/bin/env python

#
# Test node sweetie_bot::motion::AnimJointTrajectoryBase controller.
# Repeatedly send simple trajectory to execute.
# 
# Deploy joint_space_control infrastructure before running test.
#


import rospy
from rospy.rostime import Duration
import std_msgs.msg
import control_msgs.msg
import trajectory_msgs.msg
import actionlib

def deg2rad(angle):
    return angle*3.1415/180

# make FollowJointTrajectoryGoal message
def make_msg(joints, points_deg, tolerance_deg, time_tolerance):
    goal = control_msgs.msg.FollowJointTrajectoryGoal
    goal.goal_time_tolerance = Duration(time_tolerance)
    goal.path_tolerance = [ control_msgs.msg.JointTolerance(j, deg2rad(tolerance_deg), 0, 0) for j in joints ]
    goal.goal_tolerance = [ control_msgs.msg.JointTolerance(j, deg2rad(tolerance_deg), 0, 0) for j in joints ]
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    points = [ trajectory_msgs.msg.JointTrajectoryPoint([deg2rad(p) for p in pos], [], [], [], Duration(time)) for time, pos in points_deg ]
    goal.trajectory = trajectory_msgs.msg.JointTrajectory(header, joints, points)
    return goal

if __name__ == '__main__':
    rospy.init_node('test_joint_trajectory')

    client = actionlib.SimpleActionClient('/sweetie_bot/motion/controller/joint_trajectory', 
            control_msgs.msg.FollowJointTrajectoryAction)
    client.wait_for_server()

    for k in range(0,10):
        joints = ['joint12', 'joint13', 'joint14', 'joint42', 'joint43', 'joint44',]
        points = [ 
            [ 0.0, [ 0, 0, 0,  0, 0, 0] ],
            [ 1.0, [ 45, -90, 45, 45, -90, 45] ],
            [ 2.0, [ 0, 0, 0,  0, 0, 0] ],
        ]

        print("\nSending a goal: leg2, leg3.")
        client.send_goal(make_msg(joints, points, 30, 0.2))
        print("Wait for goal...")
        client.wait_for_result()
        print("Result: " + str(client.get_result()))

        joints = ['joint22', 'joint23', 'joint24', 'joint32', 'joint33', 'joint34',]
        points = [ 
            [ 0.0, [ 0, 0, 0,  0, 0, 0] ],
            [ 1.0, [ 45, -90, 45, 45, -90, 45] ],
            [ 2.0, [ 0, 0, 0,  0, 0, 0] ],
        ]

        print("\nSending a goal: leg1, leg4.")
        client.send_goal(make_msg(joints, points, 30, 0.2))
        print("Wait for goal...")
        client.wait_for_result()
        print("Result: " + str(client.get_result()))
