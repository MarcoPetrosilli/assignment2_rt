#!/usr/bin/env python
## @package assignment2_rt_part1
# \file last_target_node.py
# \brief Node that outputs the last target position set by the user
# \author Marco Petrosilli
# \version 1.0
# \date 15/03/2025
#
# \details
# Subscribes to: <BR>
# 	 /last_target
#
# Services : <BR>
# 	target_coord
#
# Description :
#
# 	This node provides a service to retrieve the last target position.
# 	It subscribes to the '/last_target' topic to receive position updates,
# 	then stores these coordinates to be retrieved later through the 'target_coord' service.
# 	This functionality allows other nodes to query the most recent target position 
# 	without needing to subscribe to the position topic themselves.
#
 

import rospy
from geometry_msgs.msg import PoseStamped
from assignment2_rt_part1.srv import target_coord, target_coordResponse


x = 0.0 ## Global variable to store the x coordinate
y = 0.0 ## Global variable to store the y coordinate

## \brief Callback function for the target position subscription
# \param msg The PoseStamped message containing the target position
# \return None
# 
# This function is called whenever a new target position is published.
# It extracts the x and y coordinates from the received message and
# updates the global variables to store the most recent position data.
# The function handles coordinate transformation from the message format
# to simple x,y values that can be easily provided through the service.

def last_target_clbk(msg):
    global x, y
    x = msg.pose.position.x
    y = msg.pose.position.y

## \brief Service handler that returns the stored target coordinates
# \param req The service request (empty in this case)
# \return target_coordResponse containing the x and y coordinates
#
# This function handles requests to the 'target_coord' service.
# When called, it creates and returns a response object containing
# the current values of the global x and y coordinate variables.
# This service allows other nodes to query the last known target position
# without needing to manage their own subscription to position updates.

def return_values(req):
    global x, y
    return target_coordResponse(x, y)
    

## \brief Main function that initializes the node and sets up communication
# \param None
# \return None
#
# This function initializes the ROS node, sets up the subscriber to
# receive position updates, and advertises the service to provide
# coordinate information to other nodes. It then enters the main loop
# that keeps the node running until shutdown is requested.
# The node operates at a frequency of 10Hz, which balances responsiveness
# with system resource usage.

def main():
    global x, y
    
    
    rospy.init_node('last_target_node')
    
    
    rospy.Subscriber('/last_target', PoseStamped, last_target_clbk)
    
   
    rospy.Service('target_coord', target_coord, return_values)
    
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    main()

