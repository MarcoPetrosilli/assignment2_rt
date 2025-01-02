#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
from actionlib_msgs.msg import GoalStatus
import assignment2_rt_part1.msg
from tf import transformations
from std_srvs.srv import *
import time

# 0 - go to point
# 1 - wall following
# 2 - done
# 3 - canceled
# callbacks

pose = PoseStamped()

def goal_planning_client(t_pose):
    client = actionlib.SimpleActionClient('/reaching_goal', assignment2_rt_part1.msg.PlanningAction)
    client.wait_for_server()
    goal = assignment2_rt_part1.msg.PlanningGoal(target_pose=t_pose)
    client.send_goal(goal, done_cb=done_callback, feedback_cb=feedback_callback)
    
    #client.wait_for_result()
    
    while not client.get_result():
        cancel_var = input("Type 'c' to cancel the target ")
        if cancel_var == 'c':
            client.cancel_goal()
            print("Goal canceled")
            return 0
        client.wait_for_result()
        
    return client.get_result()
    
def feedback_callback(feedback):
    
    actual_position = feedback.actual_pose.position
   
    target_position = pose.pose.position
    
    distance = math.sqrt(
        (actual_position.x - target_position.x)**2 +
        (actual_position.y - target_position.y)**2 +
        (actual_position.z - target_position.z)**2
    )
    
    rospy.loginfo("[FEEDBACK] Position: (%f, %f, %f); Distance: %f", actual_position.x, actual_position.y, actual_position.z, distance)

        
def done_callback(state, result):
    
    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("[STATE] Goal reached!")
        
    
def main():
    global pose
    
    rospy.init_node('action_client')
    
    while not rospy.is_shutdown():
    	
        pose.pose.position.x = float(input("Insert x coordinate: "))
        pose.pose.position.y = float(input("Insert y coordinate: "))
        pose.pose.position.z = 0.0
        #pose.theta = input("Insert the theta-orientation: ")
    
        goal_planning_client(pose)
    
if __name__ == "__main__":
    main()

