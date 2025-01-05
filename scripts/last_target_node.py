#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from assignment2_rt_part1.srv import target_coord, target_coordResponse

# Variabili globali per memorizzare le coordinate
x = 0.0
y = 0.0

# Callback per il subscriber
def last_target_clbk(msg):
    global x, y
    x = msg.pose.position.x
    y = msg.pose.position.y

# Callback per il servizio
def return_values(req):
    global x, y
    return target_coordResponse(x, y)

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

