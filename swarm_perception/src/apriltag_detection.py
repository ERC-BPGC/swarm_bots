import cv2
import numpy as np
import rospy
from apriltag import apriltag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

ros_img = Image()

def img_callback(msg):
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
		
		top_center = int((lt[0]+rt[0])/2),int((lt[1]+rt[1])/2)
		# Draw the bounding boxes
		cv2.line(cv_img,lb,rb,(0,255,0),2)
		cv2.line(cv_img,rb,rt,(0,255,0),2)
		cv2.line(cv_img,rt,lt,(0,255,0),2)
		cv2.line(cv_img,lt,lb,(0,255,0),2)

		# Draw arrow from center of tag to center of top edge
		cv2.arrowedLine(cv_img,center,top_center, (0,0,255),3,tipLength=0.08, line_type=cv2.LINE_AA)

		# Draw Dots at the corners
		cv2.circle(cv_img,lb,2,(255, 255, 0),-1)
		cv2.circle(cv_img,rb,2,(255, 0, 255),-1)
		cv2.circle(cv_img,rt,2,(0, 225, 255),-1)
		cv2.circle(cv_img,lt,2,(255, 0, 0),-1)
		rate= rospy.Rate(5)
		rate.sleep()


	# rospy.loginfo("Publishing Image")
	img_pub.publish(bridge.cv2_to_imgmsg(cv_img, "bgr8"))
	# cv2.imshow('Tags',cv_img)
	# cv2.waitKey(0)


if __name__ == "__main__":
	bridge = CvBridge()
	rospy.init_node('apriltag_detection')
	rospy.loginfo("AprilTag Node Started")
	sub = rospy.Subscriber('/camera/rgb/image_raw', Image, img_callback)
	img_pub = rospy.Publisher('/detection_image', Image, queue_size=10) 
	rospy.spin()



