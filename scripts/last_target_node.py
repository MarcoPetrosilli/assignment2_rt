#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

x = 0
y = 0

def last_target_clbk(msg):
	global x
	global y
	x = msg.pose.position.x
	y = msg.pose.position.y
	
def main():
	global x
	global y
	
	rospy.init_node('last_target_node')
	rospy.Subscriber('/last_target', PoseStamped, last_target_clbk)
	rate = rospy.Rate(10)
	
	
	while not rospy.is_shutdown():
		print("x: %f, y: %f" % (x, y))
		rate.sleep()
	

if __name__ == "__main__":
	main()
