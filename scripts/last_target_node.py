#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from assignment2_rt_part1.srv import target_coord, target_coordResponse

x = 0
y = 0

def last_target_clbk(msg):
	global x
	global y
	x = msg.pose.position.x
	y = msg.pose.position.y
	
def return_values(req):
    global x
    global y

    return SentCoordsResponse(x, y)
	
def main():
	global x
	global y
	
	rospy.init_node('last_target_node')
	rospy.Subscriber('/last_target', PoseStamped, last_target_clbk)
	rospy.Service('target_coord', target_coord, return_values) 
    	rospy.spin()
		


if __name__ == "__main__":
    main()
