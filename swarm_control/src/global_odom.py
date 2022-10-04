#usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

class GlobalOdom:
	def __init__(self):
		rospy.init_node("global_odom")
		self.pub = rospy.Publisher("/global_odom", Pose, queue_size=10)
		self.sub = rospy.Subscriber("/odom", Pose, self.callback)
		self.global_pose = []
		self.pose = rospy.get_param("bot_spawn_coordinates")
		
	def global_odom_subscriber(self):
		self.bot_index = rospy.get_param("/bot_index")
		for self.i in self.bot_index:
			rospy.Subscriber("/bot"+str(self.i)+"/odom", Odometry, self.global_odom_callback)

	# Global Callback
	def global_odom_callback(self, data):
		self.global_pose.append(data.pose.pose)
		pass		