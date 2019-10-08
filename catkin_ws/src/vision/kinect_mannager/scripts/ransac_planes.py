#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError


def callbackSubscriberPC(data):
  global point_cloud
  point_cloud = data


## When service is called return the lastest PC recived
def callbackServerPC(req):
  global point_cloud
  return point_cloud

  
      
def main(args):
  print "INITIALIZING  KINECT  MANNAGER  NODE  [by Edd]"
  rospy.init_node('kinect_mannager')

  ### Declare subscriber
  sub_pointCloud = rospy.Subscriber("/kinect2/sd/points", PointCloud2, callbackSubscriberPC)

  ### Declare service server
  serv_pointClod = rospy.Server("/hardware/kinect/get_rgbd", PointCloud2, callbackServerPC)

  global point_cloud
  point_cloud = PointCloud2()

  
  loop = rospy.Rate(10)

  print " Kinect mannager is ready:"
  print "   - Waiting for service call... "
  while not rospy.is_shutdown():
    loop.sleep()


#################################
##           MAIN            ####
if __name__ == '__main__':
    main(sys.argv)
