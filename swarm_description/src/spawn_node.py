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
	rospy.set_param('bot_spawn_coordinates', coordinate_list)
	rospy.loginfo("bot_spawn_coordinates parameter set")
	bot_index = []
	for coord in coordinate_list:
		bot_index.append(count)
		pose.position.x = coord[0]
		pose.position.y = coord[1]
		namespace = f"/bot{count}"
		client(model_name=f"robot{count}", model_xml = open(urdf_dir
, 'r').read(),robot_namespace=namespace, initial_pose = pose)
		rospy.loginfo(f"bot{count} coordinates set and spawned")
		count+=1
	rospy.set_param('bot_index', bot_index)
	rospy.loginfo("bot_index parameter set")
	print(rospy.get_param('bot_index'))

coordinates = rospy.get_param('bot_spawn_coordinates')
spawn_bots(coordinates)