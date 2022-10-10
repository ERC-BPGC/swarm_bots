#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from math import atan2, cos, sin, pi

class PID:

	def __init__(self,kp,ki,kd,final_coord,goals):
		self.ed_dot, self.ed_old, self.Ed, self.ed, self.et_dot, self.et_old,self.Et,self.et, self.v, self.x, self.y, self.theta, self.w = 0,0,0,0,0,0,0,0,0,0,0,0,0
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.goals = goals
		self.x_final, self.y_final = final_coord
		self.theta_final = 0 #atan2(self.y_final,self.x_final)
		self.motion = Twist()
		self.odom_sub = rospy.Subscriber('bot0/odom', Odometry, self.bot_pos)
		self.cmd_vel_pub = rospy.Publisher('bot0/cmd_vel', Twist, queue_size=1)
		self.count = 0
	
	def bot_pos(self,msg):
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		rot_q = msg.pose.pose.orientation
		(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
		self.theta = theta #msg.pose.pose.orientation.z
		self.theta_final = atan2(self.y_final-self.y, self.x_final-self.x)
		self.vel_update()
	
	def reset(self):
		self.ed_dot, self.ed_old, self.Ed, self.ed, self.et_dot, self.et_old,self.Et,self.et, self.v, self.x, self.y, self.theta, self.w = 0,0,0,0,0,0,0,0,0,0,0,0,0
		self.theta_final = 0
		self.count = 0
		
	def goal_change(self):
		self.x_final, self.y_final = self.goals[0]
		self.goals.pop(0)
		
	def distance(self,p1,p2):
		x1,y1 = p1
		x2,y2 = p2
		return ((x1-x2)**2 + (y1-y2)**2)**0.5
	
	def vel_update(self):
		self.ed  = self.distance((self.x_final, self.y_final),(self.x,self.y))
		self.et = self.theta_final - self.theta
		self.et = atan2(sin(self.et),cos(self.et))

		if self.ed <= 0.1:
			self.motion.linear.x = 0
			self.motion.linear.y = 0
			self.motion.angular.z = 0
			self.cmd_vel_pub.publish(self.motion)
			if len(self.goals) > 0:
				self.reset()
				self.goal_change()
			
		else:
			if self.count == 0:
				self.ed_dot = 0
				self.et_dot = 0
			else:
				self.ed_dot = self.ed - self.ed_old
				self.et_dot = self.et - self.et_old
				
			self.Ed += self.ed
			self.Et += self.et
			self.v = self.kp*self.ed + self.kd*self.ed_dot + self.ki*self.Ed
			self.w = self.kp*self.et + 0*self.kd*self.et_dot + self.ki*self.Et
			self.ed_old = self.ed
			self.et_old = self.et
			if self.v > 0:
				self.motion.linear.x = min(self.v, 0.2)
			else:
				self.motion.linear.x = max(self.v, -0.2)
			if self.w > 1:
				self.w = 1
			if self.w < -1:
				self.w = -1
			self.motion.angular.z = self.w
			
			if abs(self.et) > 0.1:
				self.motion.linear.x = 0
			else:
				self.motion.angular.z = 0
			
			self.cmd_vel_pub.publish(self.motion)
			print(self.v)
			self.count+=1

rospy.init_node('pid_controller')	

if __name__ == '__main__':
	PID(1, 0,0.5,(-1,0),[(-2,0),(0,0),(0,1)])
	rospy.spin()	
