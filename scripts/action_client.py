#!/usr/bin/env python
## @package assignment2_rt_part1
# \file action_client.py
# \brief Sends a goal to the action server, consisting of the x and y coordinates, which are read from the launch file, that the mobile robot must reach.
# \author Marco Petrosilli
# \version 1.0
# \date 15/03/2025
#
# \details
# Subscribes to: <BR>
#     /odom
#
# Publishes to: <BR>
#     /pos_target
#     /vel_last
#
# Description: <BR>
#	This node implements an action client that sends position goals to the mobile robot.
#     	It reads initial target coordinates (x,y) from the launch file parameters and sends them
#     	to the action server. The node allows users to input new target coordinates during execution,
#     	monitors the robot's progress toward the goal, and provides the option to cancel the goal
#    	by typing 'c' in the terminal. It continuously publishes the robot's current position and
#     	velocity while tracking the goal. The node also handles goal completion callbacks and feedback
#     	processing to provide status updates during execution.

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
from actionlib_msgs.msg import GoalStatus
import assignment2_rt_part1.msg
from assignment2_rt_part1.msg import pos_vel
from tf import transformations
from std_srvs.srv import *
import time
import sys
import select

# Global variables
pose = PoseStamped()  ## Global variable for the target pose
pose_ = Pose()  ## Global variable for the robot's pose
twist_ = Twist()  ## Global variable for the robot's twist (velocity)
msg = pos_vel()  ## Global message for velocity
pub = rospy.Publisher('/pos_vel', pos_vel, queue_size=10)  ## Publisher for velocity messages
targ_pub = rospy.Publisher('/last_target', PoseStamped, queue_size=10)  ## Publisher for target pose

## \brief
# Sends a planning goal to an action server and allows monitoring the progress towards the goal. 
# It also provides the ability to cancel the goal during execution.
# \param t_pose The target position (pose) that the robot should reach. It is a geometry_msgs/Pose object containing the desired position and orientation.
# \return The result of the action after it completes, which can be obtained via the client.get_result() method. The result depends on the behavior of the action as defined in the action server.
#
# This function communicates with an action server to send a goal specifying the robot's target position.
# It waits for the server to be available, sends the goal, and allows real-time monitoring of its progress.
# Users can cancel the goal by typing 'c' in the terminal. The function continuously publishes the robot?s
# current position and velocity until the goal is reached or canceled.

def goal_planning_client(t_pose):
    global pose_
    global twist_
    global targ_pub
    global pub
    
    client = actionlib.SimpleActionClient('/reaching_goal', assignment2_rt_part1.msg.PlanningAction)
    client.wait_for_server()
    goal = assignment2_rt_part1.msg.PlanningGoal(target_pose=t_pose)
    client.send_goal(goal, done_cb=done_callback, feedback_cb=feedback_callback)
    
    print("Type 'c' to cancel the target ")
    
    while not client.get_result():
    
        msg.x = pose_.position.x
        msg.y = pose_.position.y
        msg.vel_x = twist_.linear.x
        msg.vel_z = twist_.angular.z
    
        i, o, e = select.select([sys.stdin], [], [], 1.0)
        if(i):
            cancel = sys.stdin.readline().strip()
            if cancel == 'c':
               client.cancel_goal()
               rospy.loginfo("Goal deletion: confirmed")
               break

        pub.publish(msg)
        targ_pub.publish(t_pose)

    return client.get_result()

## \brief
# The clbk_odom function is a callback that updates the global pose and twist based on incoming odometry data from the robot.
# \param msg The odometry message containing the robot's current pose and twist (linear and angular velocity). It is of type nav_msgs/Odometry.
# \return This function does not return any value. It updates the global variables pose_ and twist_ with the current robot pose and twist data.
#
# This function acts as a callback for the /odom topic, updating global variables pose_ and twist_
# with the current position and velocity of the robot. These values are essential for monitoring the
# robot's movement and ensuring it follows the planned trajectory.
def clbk_odom(msg):
    global pose_
    pose_ = msg.pose.pose
    global twist_
    twist_ = msg.twist.twist

## \brief Processes feedback from the action server and computes the remaining distance.
# \param feedback The feedback message from the action server. It contains the current pose of the robot, specifically the actual position (feedback.actual_pose.position), 
# which is used to compute the distance to the target position.
# \return This function does not return any value.
#
# This function calculates the Euclidean distance between the robot's current position and the target.
# It logs the robot's position and the remaining distance to help monitor progress.
def feedback_callback(feedback):
    actual_position = feedback.actual_pose.position
    target_position = pose.pose.position
    distance = math.sqrt(
        (actual_position.x - target_position.x)**2 +
        (actual_position.y - target_position.y)**2 +
        (actual_position.z - target_position.z)**2
    )
    rospy.loginfo("[FEEDBACK] Position: (%f, %f, %f); Distance: %f", actual_position.x, actual_position.y, actual_position.z, distance)

## \brief Logs when the goal has been reached.
# \param result The result of the goal execution, which contains the outcome of the action. This parameter can hold any relevant data returned by the action server (not used in this function).
# \return This function does not return any value.
#
# This function is called when the goal execution is completed. It verifies whether the goal was successfully reached and logs the outcome accordingly.
def done_callback(state, result):
    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("[STATE] Goal reached!")

## \brief Initializes the ROS node and continuously updates the target position.
# \param None
# \return This function does not return any value.
#
# The main function initializes the ROS node, subscribes to the /odom topic, and sets the robot's target
# position using parameters retrieved from the launch file. It allows users to input new target coordinates
# dynamically and sends them to the goal planning function for execution.
def main():
    global pose
    global pose_
    global twist_
    global targ_pub

    rospy.init_node('action_client')
    rospy.Subscriber('/odom', Odometry, clbk_odom)
    rate = rospy.Rate(10)
    
    pose.pose.position.x = rospy.get_param('des_pos_x')  
    pose.pose.position.y = rospy.get_param('des_pos_y')
    pose.pose.position.z = 0.0
    
    goal_planning_client(pose)
    rate.sleep()
    
    while not rospy.is_shutdown():
        pose.pose.position.x = float(input("Insert x coordinate: "))
        pose.pose.position.y = float(input("Insert y coordinate: "))
        
        goal_planning_client(pose)
        rate.sleep()

if __name__ == "__main__":
    main()

