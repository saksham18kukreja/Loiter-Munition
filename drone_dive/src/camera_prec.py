#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import time
import cv2
import numpy as np
import math
import threading
import queue
import argparse
from sensor_msgs.msg import Image
from mavros_msgs.msg import AttitudeTarget
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_from_euler

class camera_detect:
  def __init__(self):
    self.camera_fovx = 68.0
    self.camera_fovy = 68.0
    self.camera_cx= 90
    self.camera_cy= 90
    self.tracker = cv2.TrackerKCF_create()
    self.height = 180
    self.width = 180
    self.bbox = None
    self.bridge = CvBridge()
    self.pitch_yaw_correct = AttitudeTarget()
    self.camera_pub = rospy.Publisher("/plane_cam/usb_cam/att_correction",AttitudeTarget,queue_size=10)
    self.camera_sub = rospy.Subscriber("/plane_cam/usb_cam/image_raw",Image,self.image_callback)


  def image_callback(self,data):

    video = self.bridge.imgmsg_to_cv2(data,"bgr8")
    
    if self.bbox == None:
      cv2.imshow("Tracking",video)
      self.correction_publisher(0,0,0)
      k = cv2.waitKey(30) & 0xff
      if k == 27:      
        self.bbox = cv2.selectROI(video, False)
        font = cv2.FONT_HERSHEY_SIMPLEX 
        ok = self.tracker.init(video, self.bbox)
        cv2.destroyWindow("ROI selector")


    else:
      ok, self.bbox = self.tracker.update(video)

      if ok:
          p1 = (int(self.bbox[0]), int(self.bbox[1]))
          p2 = (int(self.bbox[0] + self.bbox[2]),
                int(self.bbox[1] + self.bbox[3]))
          cv2.rectangle(video, p1, p2, (0,0,255), 2, 2)
          u = int(self.bbox[0] + self.bbox[2]/2)
          v = int(self.bbox[1] + self.bbox[3]/2)
          
      # values for pitch and yaw correction based on direction vector of the target
          yaw = np.radians(((u-self.camera_cx))*(self.camera_fovx/self.width))
          pitch = np.radians(((v-self.camera_cy))*(-self.camera_fovy/self.height))
          self.correction_publisher(0,-pitch,0)      
          print(yaw,pitch)
        
      cv2.imshow("Tracking", video)
      cv2.waitKey(1)
      

  def correction_publisher(self,roll,pitch,yaw):
    q = quaternion_from_euler(roll,pitch,yaw)
     
    self.pitch_yaw_correct.orientation.x = q[0]
    self.pitch_yaw_correct.orientation.y = q[1]
    self.pitch_yaw_correct.orientation.z = q[2]
    self.pitch_yaw_correct.orientation.w = q[3]
    self.camera_pub.publish(self.pitch_yaw_correct)          

def main():
  rospy.init_node("camera_precision",anonymous=True)
  camera = camera_detect()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main()