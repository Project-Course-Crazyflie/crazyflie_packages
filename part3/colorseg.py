#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import math
from tf.transformations import quaternion_from_matrix, euler_matrix, quaternion_from_euler
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/myresult", Image, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback)
    self.cam_info_sub = rospy.Subscriber("/cf1/camera/camera_info", CameraInfo, self.cam_info_cb)
    self.cam_info = None

    self.wall_pos_pub = rospy.Publisher("image/walls", PoseArray, queue_size=2)

  def cam_info_cb(self, msg):
    self.cam_info = msg

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
    cv2.floodFill(im_floodfill, mask, (0,0), 255)
    # Invert floodfilled image
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)
    # Combine the two images to get the foreground.
    res = res | im_floodfill_inv

    im2,contours,hierarchy = cv2.findContours(res, 1, 2)
    
    wall_contours = []
    wall_poses = PoseArray()
    wall_poses.header.frame_id = "cf1/camera_link"
    wall_poses.header.stamp = rospy.Time.now()
    for c in contours:
      l = cv2.arcLength(c, closed=True)
      #rospy.loginfo("Arc-Length: " + str(l))
      if l < 200:
        # not a wall. or wall far away
        continue

      wall_contours.append(c)
      
      M = cv2.moments(c, binaryImage=True)
      try:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        
        ###
        K = self.cam_info.K
        K_inv = np.linalg.inv(np.array(K).reshape([3, 3]))
        v = np.array([cx, cy, 1])
        v_cam = np.matmul(K_inv, v)
        ###
        #x = (cx - 320.5)/245.54354678
        #y = (cy - 240.5)/245.54354678
        #z = 1 #100/245.54354678
        x,y,z = v_cam
        
        
        #z = z/area*dist_scale
        #rospy.loginfo("K_inv: " + str(K_inv) )
        #rospy.loginfo("V_true: " + str([x, y]) )
        #rospy.loginfo("V_cam: " + str(v_cam))
        
        
        """
        def skew(v):
            return np.array([[0, -v[2], v[1]],[v[2], 0, -v[0]],[-v[1], v[0], 0]])

        a = np.array([1, 1, 1])
        b = np.array([x, y, z])
        b = b/np.linalg.norm(b)
        c = a * b
        v = np.cross(a, b)
        R = np.eye(3) + skew(v) + np.square(skew(v))*1/(1 + c)
        R = np.array([list(R[0,:]) + [0], list(R[1,:]) + [0], list(R[2,:]) + [0], [0,0,0,1]])
        rospy.loginfo(R)
        
        v = np.array([x, y, z])
        u = np.array([x, y, 0])
        theta = np.dot(v, u)/(np.linalg.norm(v)*np.linalg.norm(u))
        """

        
        xr = np.arctan2(z, y)
        yr = -np.arctan2(z, x)
        zr = np.arctan2(y, x)
        #xy_norm = np.linalg.norm([x,y])
        #theta = -np.arctan2(z, xy_norm)
        #theta = 0
        q = quaternion_from_euler(0, yr, 0, 'rzyx') 
        
        area = cv2.contourArea(c)
        rospy.loginfo("Area: " + str(area))
        dist_scale = 12000/(area)
        p = Pose()
        p.position.x = x*dist_scale
        p.position.y = y*dist_scale
        p.position.z = z*dist_scale
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        #p.position.z = 1/245.54354678

        wall_poses.poses.append(p)
      except ZeroDivisionError:
        pass
      else:
        cv2.circle(cv_image, (cx,cy), 50, 255)

    cv2.drawContours(cv_image, wall_contours, -1, (255,0,0), 3)

    # Publish the image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      self.wall_pos_pub.publish(wall_poses)
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
