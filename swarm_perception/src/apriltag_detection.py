import cv2
import rospy
import numpy as np
from apriltag import apriltag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
ros_img = Image()

# Coordinates of the bounding box
bound_top = (0,0)
bound_bottom = (0,0)
arena_center = (0,0)
# Image frame cordinates, not world frame cordinates
# Initialize the pose array with empty elements for each robot

poses = PoseArray()
for i in range(0, rospy.get_param('bots')):
	poses.poses.append(Pose())

world_poses = PoseArray()
for i in range(0, rospy.get_param('bots')):
	world_poses.poses.append(Pose())

def img_callback(msg):
	if msg == None:
		return
	global ros_img

	# rospy.loginfo("Image Received")
	ros_img = msg
	cv_img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
	
	gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
	detector = apriltag("tag36h11")
	results = detector.detect(gray)
	if len(results) > 0:
		rospy.loginfo(f"{len(results)} Tags Detected, Drawing Bounding Boxes")


	for i in range(0,len(results)):
		# Corners of the tag	
		lb = int(results[i]['lb-rb-rt-lt'][0][0]),int(results[i]['lb-rb-rt-lt'][0][1])
		rb = int(results[i]['lb-rb-rt-lt'][1][0]),int(results[i]['lb-rb-rt-lt'][1][1])
		rt = int(results[i]['lb-rb-rt-lt'][2][0]),int(results[i]['lb-rb-rt-lt'][2][1])
		lt = int(results[i]['lb-rb-rt-lt'][3][0]),int(results[i]['lb-rb-rt-lt'][3][1])
		center = int(results[i]['center'][0]),int(results[i]['center'][1])
		id = results[i]['id']
		top_center = int((lt[0]+rt[0])/2),int((lt[1]+rt[1])/2)
		# Draw the bounding boxes
		# cv2.line(cv_img,lb,rb,(0,255,0),2)
		# cv2.line(cv_img,rb,rt,(0,255,0),2)
		# cv2.line(cv_img,rt,lt,(0,255,0),2)
		# cv2.line(cv_img,lt,lb,(0,255,0),2)

		# Draw arrow from center of tag to center of top edge
		cv2.arrowedLine(cv_img,center,top_center, (0,0,255),3,tipLength=0.08, line_type=cv2.LINE_AA)

		# Draw Dots at the corners
		cv2.circle(cv_img,lb,2,(255, 255, 0),-1) # Yellow
		cv2.circle(cv_img,rb,2,(255, 0, 255),-1) # Purple *
		cv2.circle(cv_img,rt,2,(0, 225, 255),-1) # Cyan
		cv2.circle(cv_img,lt,2,(255, 0, 0),-1) # Blue *
		if id == 15:
			global bound_top
			bound_top = rb
			print("Top Bound in image: ", bound_top)
			continue
			
		elif id == 14:
			global bound_bottom
			bound_bottom = lt
			print("Bottom Bound in image: ", bound_bottom)
			continue

		# Show Coords of Origin of Arena
		global arena_center
		arena_center = int((bound_top[0]+bound_bottom[0])/2),int((bound_top[1]+bound_bottom[1])/2)
		cv2.circle(cv_img,arena_center,5,(0, 225, 255),-1) # Cyan


		# rate= rospy.Rate(5)
		# rate.sleep()
		if id == 3:
			pose_img_pub(id, int(results[i]['center'][0]),int(results[i]['center'][1]))
			pose_global_pub(id, int(results[i]['center'][0]),int(results[i]['center'][1]))

	# rospy.loginfo("Publishing Image")
	img_pub.publish(bridge.cv2_to_imgmsg(cv_img, "bgr8"))
	# cv2.imshow('Tags',cv_img)
	# cv2.waitKey(0)

def pose_img_pub(id, x, y):
	'''Publishes poses of tags in Image Frame (pixels)'''
	# Update Pose at the ID of the robot
	poses.poses[id-1].position.x = x
	poses.poses[id-1].position.y = y

	# Publish the PoseArray
	# print(poses)
	image_poses_pub.publish(poses)
	rate = rospy.Rate(50)
	rate.sleep()

def pose_global_pub(id, x, y):
	'''Publishes poses of tags in Global Frame (meters)'''
	global world_poses
	x_global,y_global = img_to_global_transform(x,y)
	world_poses.poses[id].position.x = x_global
	world_poses.poses[id].position.y = y_global
	global_pose_pub.publish(world_poses)
	rate = rospy.Rate(50)
	rate.sleep()

def img_to_global_transform(x,y):
	'''
	Transforms poses from Image Frame (pixels) to Global Frame (meters)  
	
	We define 3 Frames
	1. Image Frame: Origin is top left corner of image
	2. Arena Frame: Origin is center of arena in the image
	3. Global Frame: Origin is center of arena in the world

	We recieve data in the Image Frame. Which is then transformed to the Arena Frame as follows:
	1. The center of the arena is found in the image frame
	2. Origin is shifted to the center of the arena

	Now we have the data in the Arena Frame. Which is then transformed to the Global Frame as follows:
	- Since the arena frame and the global frame have the same origin, we just need to multiply all the values by the scaling factor.
	'''
	global bound_top, bound_bottom
	rospy.set_param("image_frame_bounds", [bound_top, bound_bottom])

	# rospy.loginfo("Transforming Image Coordinates to Global Coordinates")

	# Length of diagonal of the arena in Image Frame (pixels)
	# bound_diagonal_image = np.linalg.norm(np.array(bound_top)-np.array(bound_bottom))
	bound_diagonal_image = np.sqrt((bound_top[0]-bound_bottom[0])**2 + (bound_top[1]-bound_bottom[1])**2)
	print(bound_diagonal_image)
	# print(bound_top, bound_bottom)
	# print("Diagonal of Arena in Image Frame: ", bound_diagonal_image)
	dimension  = rospy.get_param("dimension")

	# Diagonal of arena is sqrt(2)*dimension (Assuming a square arena)
	bound_diagonal_global = dimension * np.sqrt(2)
	scale = 0
	if bound_diagonal_image != 0:
		# Calculate the conversion factor for image to global coordinates (unit: meters/pixels)
		scale = bound_diagonal_global/bound_diagonal_image
		print('Scale:',scale)
		# rospy.set_param("scale", scale)

	# Accounting for rotation of camera and arena
	y_global,x_global =  (-scale * (x-arena_center[0])) - 0.1,(-scale * (y-arena_center[1])) - 0.26
	print(x_global,y_global)
	return x_global,y_global
	

if __name__ == "__main__":
	bridge = CvBridge()
	rospy.init_node('apriltag_detection')
	rospy.loginfo("AprilTag Node Started")
	sub = rospy.Subscriber('/camera/rgb/image_raw', Image, img_callback)
	img_pub = rospy.Publisher('/detection_image', Image, queue_size=10) 
	image_poses_pub = rospy.Publisher('/agent_poses_image', PoseArray, queue_size=10)
	global_pose_pub = rospy.Publisher('/agent_poses_global', PoseArray, queue_size=10)
	rospy.spin()



