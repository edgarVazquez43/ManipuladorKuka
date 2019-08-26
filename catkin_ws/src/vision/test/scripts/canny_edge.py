#!/usr/bin/env python
from __future__ import print_function

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color",Image,self.callback)

  def callback(self,data):
    try:
	cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      
	# converting BGR to HSV 
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

	# define range of red color in HSV 
	lower_red = np.array([30,150,50]) 
	upper_red = np.array([255,255,180])	
      
    	# create a red HSV colour boundary and  
    	# threshold HSV image 
    	mask = cv2.inRange(hsv, lower_red, upper_red)

	# Bitwise-AND mask and original image 
	res = cv2.bitwise_and(cv_image,cv_image, mask= mask)

    	# finds edges in the input image image and 
    	# marks them in the output map edges 
    	edges = cv2.Canny(cv_image,100,200) 

    except CvBridgeError as e:
      print(e)

    cv2.imshow("Image window",edges)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


