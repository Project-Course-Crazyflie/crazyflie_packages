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

    sift = cv2.SURF()
    kp = sift.detect(gray,None)

    img=cv2.drawKeypoints(gray,kp)
    res = img
    #cv2.imwrite('sift_keypoints.jpg',img)

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
