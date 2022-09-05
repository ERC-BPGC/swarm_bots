#!/usr/bin/env python3
from gazebo_msgs.srv import SpawnModel
import os
import rospy
import rospkg
from geometry_msgs.msg import Pose

rospack = rospkg.RosPack()
rospy.init_node('spawn_bot',log_level=rospy.INFO)

initial_pose = Pose()
initial_pose.position.x = 0
initial_pose.position.y = 0
initial_pose.position.z = 0

reference_frame='world'
spawn_bot_client = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

directory = rospack.get_path('swarm_description')
full_path = os.path.join(directory,'models/agent/urdf','bot.urdf.xacro')

def spawn_bots(coordinate_list, client=spawn_bot_client, pose=initial_pose, urdf_dir=full_path):
	count = 0
	for coord in coordinate_list:
		pose.position.x = coord[0]
		pose.position.y = coord[1]
		namespace = f"/bot{count}"
		client(model_name=f"robot{count}", model_xml = open(urdf_dir
, 'r').read(),robot_namespace=namespace, initial_pose = pose)
		count+=1

spawn_bots([(0,0),(1,1),(0,2),(3,4)])