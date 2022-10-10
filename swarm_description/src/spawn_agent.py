#usr/bin/env python3

from pickletools import read_stringnl_noescape_pair
from gazebo_msgs.srv import SpawnModel
import os
import rospy
import rospkg
from geometry_msgs.msg import Pose
import numpy as np


def spawn_agent(qty):

	# Service Calls
	rospy.loginfo("Waiting for Service: gazebo/spawn_urdf_model ....")
	rospy.wait_for_service("gazebo/spawn_urdf_model")
	rospy.loginfo("Service: gazebo/spawn_urdf_model is available!!")

	# Get URDF file
	rospack = rospkg.RosPack()
	

	coords = rospy.get_param("bot_spawn_coordinates")
	print(coords)


	# Parameters have been initialized
	# ! bot_index = []
	pose = Pose()
	count = 0

	for i in range(qty):
		urdf_dir = os.path.join(rospack.get_path("swarm_description"), f"models/agent{i}", "bot.urdf.xacro")	
		client = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
		# ! bot_index.append(count)
		
		# Assign random coordinates
		rospy.loginfo("Assigning random coordinates")
		pose.position.x = np.random.uniform(-2.3,2.3)
		pose.position.y = np.random.uniform(-2.3,2.3)

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
