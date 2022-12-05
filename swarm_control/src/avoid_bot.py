#!/usr/bin/python3

import rospy
import random
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
import sys

id = sys.argv[1]

x_vel = 0

other_x = 0
other_y = 0

x = 0
y = 0

def coord_update_callback(odom):
	global x
	global y
	x = odom.pose.pose.position.x
	y = odom.pose.pose.position.y

def turn_callback(odom):
	global x_vel
	global x
	global y
	global other_x
	global other_y

	x_vel = odom.twist.twist.linear.x
	other_x = odom.pose.pose.position.x
	other_y = odom.pose.pose.position.y

	if ((other_x - x)**2 + (other_y - y)**2)**0.5 <= 0.25:
		x_vel = -x_vel

def main():
	global x_vel
	rospy.init_node('avoid_bots_controller', anonymous=True)

	pub = rospy.Publisher('bot'+str(id)+'/cmd_vel',Twist,queue_size=10) 

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		for i in range(10):
			if i != id:
				sub_0 = rospy.Subscriber('bot'+str(id)+'/odom',Odometry,coord_update_callback)
				sub_1 = rospy.Subscriber('bot'+str(i)+'/odom',Odometry,turn_callback)
				pub.publish(x_vel)
		rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass