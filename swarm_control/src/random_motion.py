#!/usr/bin/python3

import rospy
import random
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
import sys

x_vel = 0
y_vel = 0
id = sys.argv[1]

def callback(odom):
	global x_vel
	global y_vel

	x_vel = odom.twist.twist.linear.x
	y_vel = odom.twist.twist.linear.y
	# print(x_vel)
	# print(y_vel)

def main():
	global x_vel
	global y_vel
	rospy.init_node('random_motion_controller', anonymous=True)

	pub = rospy.Publisher('bot'+str(id)+'/cmd_vel',Twist,queue_size=10) 
	rate = rospy.Rate(10)

	vel = Twist()
	count = 0
	while not rospy.is_shutdown():
		sub = rospy.Subscriber('bot'+str(id)+'/odom',Odometry,callback) 
		vel.linear.x = random.random()
		vel.linear.y = random.random()
		if count % 5 ==0:
			vel.angular.z = random.uniform(-1,1) * 3.14
		else:
			vel.angular.z = 0
		count += 1	
		pub.publish(vel)
		rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass