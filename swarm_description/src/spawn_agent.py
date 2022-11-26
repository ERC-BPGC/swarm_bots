#!/usr/bin/env python3

from gazebo_msgs.srv import SpawnModel
import os
import rospy
import rospkg
from geometry_msgs.msg import Pose, Quaternion
import numpy as np
import math
import random
from tf.transformations import quaternion_from_euler
SAFE_DIST = 0.15 # Collision free distance between 2 robots
coords = rospy.get_param("bot_spawn_coordinates")



def check_collision(x,y):
	coords = rospy.get_param("bot_spawn_coordinates")
	for coord in coords:
		if abs(coord[0] - x) < SAFE_DIST and abs(coord[1] - y) < SAFE_DIST:
			return True
	return False

def spawn_agent(qty):

	# Service Calls
	rospy.loginfo("Waiting for Service: gazebo/spawn_urdf_model ....")
	rospy.wait_for_service("gazebo/spawn_urdf_model")
	rospy.loginfo("Service: gazebo/spawn_urdf_model is available!!")

	# Get URDF file
	rospack = rospkg.RosPack()

	print(coords)

	# Parameters have been initialized
	# ! bot_index = []
	pose = Pose()
	count = 0

	for i in range(qty):
		urdf_dir = os.path.join(rospack.get_path("swarm_description"), f"models/agent{i}", "bot.urdf.xacro")	
		client = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
		# ! bot_index.append(count)
		
		rospy.loginfo("Assigning random coordinates")
		while True:
			# Assign random coordinates
			pose.position.x = np.random.uniform(-2.3,2.3)
			pose.position.y = np.random.uniform(-2.3,2.3)
			if check_collision(pose.position.x, pose.position.y):
				print("Collision detected. Trying Again")
				continue
			elif not check_collision(pose.position.x, pose.position.y):
				break
				
		# half_angle = (float)(random.randint() % 360 - 180)/2 * np.pi/180.0
		q = quaternion_from_euler(0, 0, random.uniform(-np.pi, np.pi))
		pose.orientation.x = q[0]
		pose.orientation.y = q[1]
		pose.orientation.z = q[2]
		pose.orientation.w = q[3]


		# Set coordinates on the parameter server
		coords.append([pose.position.x,pose.position.y])
		rospy.set_param("bot_spawn_coordinates", coords)
		# ! rospy.set_param("bot_index", bot_index)

		namespace = f"/bot{count}"
		client(model_name = f"robot{count}", model_xml = open(urdf_dir, 'r').read(), robot_namespace = namespace, initial_pose = pose)
		rospy.loginfo(f"bot{count} spawned")

		# ! bot_index.append(count)  
		count += 1

# cooordinates = rospy.get_param("bot_spawn_coordinates")

if __name__=="__main__":
	rospy.init_node("spawn_agent")
	bots = rospy.get_param("bots")
	rospy.loginfo(f"Initialising Simulation with {bots} bots ")
	spawn_agent(bots)
