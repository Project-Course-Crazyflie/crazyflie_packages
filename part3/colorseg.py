#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import imutils
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
    except CvBridgeError as e:
      print(e)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of the color we look for in the HSV space
    lower = np.array([5, 50, 50])
    upper = np.array([15, 255, 255])

    # Threshold the HSV image to get only the pixels in ranage
    mask = cv2.inRange(hsv, lower, upper)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(gray, gray, mask= mask)
    
    # Copy the thresholded image.
    im_floodfill = res.copy()
    # Mask used to flood filling.
    # Notice the size needs to be 2 pixels than the image.
    h, w = res.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)
    # Floodfill from point (0, 0)
    cv2.floodFill(im_floodfill, mask, (0,0), 255);
    # Invert floodfilled image
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)
    # Combine the two images to get the foreground.
    res = res | im_floodfill_inv

    im2,contours,hierarchy = cv2.findContours(res, 1, 2)
    #cv2.drawContours(cv_image, contours, -1, (255,0,0), 3)
    for c in contours:
      M = cv2.moments(c, binaryImage=True)
      try:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
      except ZeroDivisionError:
        pass
      else:
        cv2.circle(cv_image, (cx,cy), 50, 255)

    # Publish the image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, "mono8"))
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
