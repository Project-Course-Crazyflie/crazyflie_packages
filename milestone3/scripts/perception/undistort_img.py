#!/usr/bin/env python
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import roslib
import rospy
from sensor_msgs.msg import Image
import os.path
import glob


#saves images from video feed
rospy.init_node('undistort_img')

image_msg = None
bridge = CvBridge()
mtx  = np.load(rospy.get_param(rospy.get_name() + "/mtx"))
dist = np.load(rospy.get_param(rospy.get_name() + "/dist"))
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(640,480),1,(640,480))

def callback(msg):
    global image_msg
    image_msg = msg
    return

def publish(msg):
    pub_img.publish(msg)
    return

sub_pose = rospy.Subscriber('cf1/camera/image_raw', Image, callback)
pub_img = rospy.Publisher('cf1/camera/image_undist', Image, queue_size=1)


def main():
    global image_msg
    rate = rospy.Rate(40) #Hz

    while not rospy.is_shutdown():
        if image_msg:
            try:
                img = bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            except CvBridgeError as e:
                print(e)
            image_msg = None
            #get undistorted image
            dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
            x,y,w,h = roi
            dst = dst[y:y+h, x:x+w]
            # h, w = 446, 332

            #publish it 
            publish(bridge.cv2_to_imgmsg(dst, 'bgr8'))

        rate.sleep()

if __name__ == '__main__':
	main()