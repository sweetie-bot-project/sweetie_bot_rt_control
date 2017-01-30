#!/usr/bin/env python

import rospy
import sweetie_bot_resource_control_msgs.msg
import actionlib

if __name__ == '__main__':
    rospy.init_node('controller_selector')

    client = actionlib.SimpleActionClient('/motion/cont_actionlib', 
            sweetie_bot_resource_control_msgs.msg.MoveManuallyAction)
    client.wait_for_server()

    print("\nSending a goal: speed = 1.0 ")
    goal = sweetie_bot_resource_control_msgs.msg.MoveManuallyGoal(1.0, 5.0)
    client.send_goal(goal)
    print("Wait for goal...")
    client.wait_for_result()
    print("Result: " + str(client.get_result()))

    print("\nSending an incorrect goal: speed = 2.0 ")
    goal = sweetie_bot_resource_control_msgs.msg.MoveManuallyGoal(2.0, 2000.0)
    client.send_goal(goal)
    print("Wait for goal...")
    client.wait_for_result()
    print("Result: " + str(client.get_result()))

    print("\nSending a goal: speed = 3.0 ")
    goal = sweetie_bot_resource_control_msgs.msg.MoveManuallyGoal(3.0, 5.0)
    client.send_goal(goal)
    print("Wait 3 seconds...")
    rospy.sleep(3)
    print("Cancel goal.")
    client.cancel_goal()
    client.wait_for_result()
    print("Result: " + str(client.get_result()))

    print("\nSending a goal: speed = 4.0 ")
    goal = sweetie_bot_resource_control_msgs.msg.MoveManuallyGoal(4.0, 5.0)
    client.send_goal(goal)
    print("Cancel goal.")
    client.cancel_goal()
    client.wait_for_result()
    print("Result: " + str(client.get_result()))

    print("\nSending a goal: speed = 5.0 ")
    goal = sweetie_bot_resource_control_msgs.msg.MoveManuallyGoal(5.0, 5.0)
    client.send_goal(goal)
    print("Wait 3 seconds...")
    rospy.sleep(3)
    print("Preemt by a new goal: speed = 6.0 ")
    goal = sweetie_bot_resource_control_msgs.msg.MoveManuallyGoal(6.0, 5.0)
    client.send_goal(goal)
    client.wait_for_result()
    print("Result: " + str(client.get_result()))

    print("\nSending a goal: speed = 7.0 ")
    goal = sweetie_bot_resource_control_msgs.msg.MoveManuallyGoal(7.0, 5.0)
    client.send_goal(goal)
    print("Preemt by a new goal: speed = 8.0 ")
    goal = sweetie_bot_resource_control_msgs.msg.MoveManuallyGoal(8.0, 5.0)
    client.send_goal(goal)
    client.wait_for_result()
    print("Result: " + str(client.get_result()))
