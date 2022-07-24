#!usr/bin/env python3

# Program to spawn a number of agents in the simulation
# =======================================================================================================================================
# TODO 
# ! - Should SPAWNING a new Agent create a new object of class Agent
# ! - Should CREATING a new object of type Agent spawn it into the world
# =======================================================================================================================================

import rospy
from gazebo_msgs.srv import SpawnModel
from tf.transformations import quaternion_from_euler
import random
import rospkg
import math

class Agent:
    """Class to describe the property of each agent 

    Variables:  

    - agent_id: id of the model corresponds to the AruCo Tag-ID 

    - pose_x: x position of the agent  

    - pose_y: y position of the agent  

    - angle: rotation around the Z-Axis (Heading of the robot)  

    Methods:
        
        - update_parameters: updates the parameters of the agent when a new agent is created
    """

    def __init__(self, agent_id, pose_x, pose_y, angle):

        self.agent_id = agent_id

        # Spawn Position Parameters
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.angle = angle

        self.agent_id_list = rospy.get_param("/agent_id")
  
        if self.agent_id_list[0] == 6969: # Parameter list hasnt been initialised yet
            # Initialise Parameter Server
            # Add first agent to the parameter server
            rospy.set_param("/agent_id", [self.agent_id])  
            rospy.set_param("/agent_start_position", [[self.pose_x, self.pose_y, self.angle]])
            return
        
        # Add new agent to the parameter server
        self.update_parameters()
        
    def update_parameters(self):
        
        # Append new parameters to the parameters
        rospy.set_param("/agent_id", self.agent_id_list.append(self.agent_id))
        rospy.set_param("/agent_start_position",self.agent_id_list.append([self.pose_x, self.pose_y, self.angle]))

def spawn_agent(agent_id: int, random_pos: bool, pose_x: float, pose_y: float, angle: float) -> bool:
    """Spawns an agent and adds the agent info to the Parameter Server

    Input:

    - agent_id: Contains the marker ID of the current Agent

    - random_pos: Specifies if the agent is spawned at a random position or not. If True,
    the agent is spawned at a random position and the position parameters are ignored

    - pose_x: X Coordinate of the Model (Ignored if random_pos = True)

    - pose_y: Y Coordinate of the Model (Ignored if random_pos = True)

    - pose_z: Z Coordinate of the Model (Ignored if random_pos = True)

    Output:

    - success: Boolean to indicate if the agent was spawned successfully or not
    """

    rospack = rospkg.RosPack()
    # Add path for the models
    path = rospack.get_path("swarm_description")
    print(path)
    # Model ID of the Agent
    # Tentative change when URDF is ready
    sdf_model = open(path + '/models/aruco_marker_{}.sdf'.format(agent_id), 'r').read()

    # Request the spawn service
    spawn_service = rospy.ServiceProxy('gazebo/Spawn_Model', SpawnModel)
    rospy.loginfo("Waiting for Spawn Model Service...")
    spawn_service.wait_for_service()
    rospy.loginfo("Connected to Spawn Model Service!")

    request = SpawnModel._request_class()
    response = SpawnModel._response_class()

    ARENA_LENGTH = rospy.get_param("arena_length")
    ARENA_WIDTH = rospy.get_param("arena_width")

    position_occupied = False

    if random_pos:  # Spawn an Agent at a Random Position within the arena

        request.model_name = "agent_" + str(agent_id)
        request.model_xml = sdf_model

        while True: # Keep generating new positions until a non-occupied position is found
            request.initial_pose.position.x = random.uniform(-ARENA_LENGTH, ARENA_LENGTH)
            request.initial_pose.position.y = random.uniform(-ARENA_WIDTH, ARENA_WIDTH)
            request.initial_pose.position.z = 0

            # TODO Add angle to the request
            # angle = random.uniform(0, math.pi)

            position_occupied = check_position(request.initial_pose.position.x, request.initial_pose.position.y)
            
            if position_occupied:
                # Generate a new random position
                continue

            elif not position_occupied:
                rospy.loginfo("Spawning Agent {} at Pose ({}, {},{})".format(agent_id, pose_x, pose_y, angle))
                # Call the service to spawn the agent
                spawn_service.call(request)

                rospy.loginfo("Model Spawned!!")
                break

    elif not random_pos:  # Spawn an Agent at a Specific Position within the arena

        request.model_name = "agent_" + str(agent_id)
        request.model_xml = sdf_model

        while True:
            request.initial_pose.position.x = random.uniform(-ARENA_LENGTH, ARENA_LENGTH)
            request.initial_pose.position.y = random.uniform(-ARENA_WIDTH, ARENA_WIDTH)
            request.initial_pose.position.z = 0

            # TODO Add angle to the request
            # angle = random.uniform(0, math.pi)

            position_occupied = check_position(request.initial_pose.position.x, request.initial_pose.position.y)

            if position_occupied or math.abs(request.initial_pose.position.x) > ARENA_LENGTH or math.abs(request.initial_pose.position.y) > ARENA_WIDTH:
                rospy.logerr("Position Invalid please call the service again with a different position")

                # Send Failure Response
                response.success = False
                return response.success

            elif not position_occupied:
                rospy.loginfo("Spawning Agent {} at Pose ({}, {},{})".format(agent_id, pose_x, pose_y, angle))

                # Call the service to spawn the agent
                spawn_service.call(request)

                rospy.loginfo("Model Spawned!!")
                break

    response.success = True
    # response.status_message = "Spawned Model"
    rospy.loginfo("Spawned Model")
    return response.success


def check_position(x: float,y: float) -> bool:
    """
    Take data from Paramter Server and check if the position is occupied

    - Return False if position is not occupied
    
    - Return True if position is occupied
    """

    agent_position_list = rospy.get_param("agent_start_position")
    count = 0
    for x2,y2,a2 in agent_position_list:
        if euclidiean_dist(x,y, x2,y2) > 0.5:
            count = count + 1
    
    if count == len(agent_position_list) - 1: # Position is not occupied
        return False
    else:
        return True
    
def euclidiean_dist(x1,y1,x2,y2) -> float:
    """
    Calculate Euclidiean Distance between 2 points in a 2D Space"""

    return math.sqrt((x1-x2)**2 + (y1-y2)**2)
    
