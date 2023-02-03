#!/usr/bin/python3

import rospy
import random
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
import sys
import math

x_vel = 0
y_vel = 0
x = 0
y = 0
id = sys.argv[1]

def callback(odom):
	global x_vel
	global y_vel
	global x
	global y

	x = odom.pose.pose.position.x
	y = odom.pose.pose.position.y
	x_vel = odom.twist.twist.linear.x
	y_vel = odom.twist.twist.linear.y
	# print(x_vel)
	# print(y_vel)

def main():
	global x_vel
	global y_vel
	global x
	global y

	rospy.init_node('random_motion_controller_'+str(id))

	pub = rospy.Publisher('bot'+str(id)+'/cmd_vel',Twist,queue_size=10) 
	rate = rospy.Rate(10)

	vel = Twist()
	count = 0
	while not rospy.is_shutdown():
		sub = rospy.Subscriber('bot'+str(id)+'/odom',Odometry,callback) 
		vel.linear.x = random.random()
		if count % 5 == 0:
			vel.angular.z = random.uniform(-1,1) * 3.14
		else:
			vel.angular.z = 0
		if math.abs(x-2.5) <= 0.2:
			vel.linear.x = -vel.linear.x
		if math.abs(y-2.5) <= 0.2:
			vel.linear.x = -vel.linear.x
		count += 1	
		pub.publish(vel)
		rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
