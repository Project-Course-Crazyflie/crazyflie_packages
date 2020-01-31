#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/myresult", Image, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback)

  def callback(self,data):
    # Convert the image from ROS to OpenCV format?
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      gray = self.bridge.imgmsg_to_cv2(data, "mono8")
      #gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    except CvBridgeError as e:
      print(e)

    gray = np.float32(gray)
    dst = cv2.cornerHarris(gray,2,3,0.04)
    #result is dilated for marking the corners, not important
    dst = cv2.dilate(dst,None)
    # Threshold for an optimal value, it may vary depending on the image.
    cv_image[dst>0.01*dst.max()]=[0,0,255]
    res = cv_image

    # Otsu's thresholding after Gaussian filtering
    #blur = cv2.GaussianBlur(cv_image,(5,5),0)
    #ret3,th3 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #res = th3

    

    # Publish the image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('colorseg', anonymous=True)

  ic = image_converter()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
