#!/usr/bin/env python
from __future__ import print_function, division
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

#from __future__ import division
import argparse
from math import atan2, cos, sin, sqrt, pi

class image_converter:


	def __init__(self):
    		self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)

    		self.bridge = CvBridge()
    		self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color",Image,self.callback)

	def callback(self,data):
    		try:
			#Load video as BGR8
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			
			if cv_image is None:
    				print('Could not open or find the image: ', args.input)
    				exit(0)

			# Convert image to grayscale
			gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
			# Convert image to binary
			_, bw = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
			# Find all the contours in the thresholded image
			_, contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

			for i, c in enumerate(contours):
				# Calculate the area of each contour
				area = cv2.contourArea(c);
				# Ignore contours that are too small or too large
				if area < 1e2 or 1e3 < area:
					continue

				# Draw each contour only for visualisation purposes
				cv2.drawContours(cv_image, contours, i, (0, 0, 255), 2);
				# Find the orientation of each shape
				getOrientation(c, cv_image)
				
		except CvBridgeError as e:
			print(e)

		cv2.imshow("PCA",cv_image)
		cv2.waitKey(3)		

		cap.release()
		cv2.destroyAllWindows()
    
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

def drawAxis(img, p_, q_, colour, scale):
	p = list(p_)
	q = list(q_)
	## [visualization1]
	angle = atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
	hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))

	# Here we lengthen the arrow by a factor of scale
	q[0] = p[0] - scale * hypotenuse * cos(angle)
	q[1] = p[1] - scale * hypotenuse * sin(angle)
	cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)

	# create the arrow hooks
	p[0] = q[0] + 9 * cos(angle + pi / 4)
	p[1] = q[1] + 9 * sin(angle + pi / 4)
	cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)

	p[0] = q[0] + 9 * cos(angle - pi / 4)
	p[1] = q[1] + 9 * sin(angle - pi / 4)
	cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
	## [visualization1]

def getOrientation(pts, img):
	## [pca]
	# Construct a buffer used by the pca analysis
	sz = len(pts)
	data_pts = np.empty((sz, 2), dtype=np.float64)
	for i in range(data_pts.shape[0]):
		data_pts[i,0] = pts[i,0,0]
		data_pts[i,1] = pts[i,0,1]

		# Perform PCA analysis
		mean = np.empty((0))
		mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)

		# Store the center of the object
		cntr = (int(mean[0,0]), int(mean[0,1]))
		## [pca]

		## [visualization]
		# Draw the principal components
		cv2.circle(img, cntr, 3, (255, 0, 255), 2)
		p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
		p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
		drawAxis(img, cntr, p1, (0, 255, 0), 1)
		drawAxis(img, cntr, p2, (255, 255, 0), 5)

		angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
		## [visualization]

		return angle


if __name__ == '__main__':
    	  main(sys.argv)

