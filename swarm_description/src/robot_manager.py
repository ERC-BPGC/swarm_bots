#usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from swarm_msgs.msg import pose_list
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
import math
import random

postions = [] # List of positions of all robots







