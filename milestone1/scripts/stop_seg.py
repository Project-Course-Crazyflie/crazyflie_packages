#!/usr/bin/env python

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import roslib
from std_msgs.msg import String
import numpy as np
import rospy

mtx  = np.load('mtx.npy')
dist = np.load('dist.npy')
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(640,480),1,(640,480))

def image_callback(img_msg):
    #convert image from ROS format to CV
    bridge = CvBridge()
    try:
    # verify the undistorted video stream
        cv_image = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        #dst = cv2.undistort(cv_image, mtx, dist, None, newcameramtx)
        #x,y,w,h = roi
        #cv_image = dst[y:y+h, x:x+w]

    except CvBridgeError as e:
        print(e)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of the color we look for in the HSV space, trial and error kind of (HEX to HSV)
    lower0 = np.array([170, 10, 30])
    upper0 = np.array([179, 255, 255])

    lower1 = np.array([0, 10, 30])
    upper1 = np.array([5, 255, 255])

    # Threshold the HSV image to get only the pixels in ranage
    gray0 = cv2.inRange(hsv, lower0, upper0)
    gray1 = cv2.inRange(hsv, lower1, upper1)

    gray = cv2.bitwise_or(gray0, gray1)

    #Blur with Gaussian Filtering, NO BLUR SINCE IT DETECTS THE SIGNS, but you need it in irl because of much noise
    gray = cv2.GaussianBlur(gray, (5,5),  2)

    kernel = np.ones((5,5),np.uint8)

    #Erosion, the kernel slides through the image (2D convolution). 1 or zeros. A pixel in the original image considered 1 if all pixels under the kernel is 1.
    gray = cv2.erode(gray, kernel, iterations = 2)
    #Dilate, , a pixel element is 1 if atleast one pixel under the kernel is 1
    gray = cv2.dilate(gray, kernel, iterations = 4)


    _, contours, hierarchy = cv2.findContours(gray,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)


    try:
        #Get the actual inner list of the hierarchy
        hierarchy = hierarchy[0]

        #(Empty) List with walls and their center of mass coordinates
        center_mass = []

        #Calculating center of mass and drawing contours
        for i, c in enumerate(contours):
            current_hierarchy = hierarchy[i]
            print(cv2.contourArea(c))
            if current_hierarchy[3] < 0 and cv2.contourArea(c) >= 650 :
                #Outermost parent components
                M = cv2.moments(c)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                center_mass.append((cx, cy))
                cv_image = cv2.drawContours(cv_image, c, -1, (255,0,0), 2)
                cv_image = cv2.circle(cv_image,(cx,cy), 4, (255,0,0), -1 )


            """
            elif current_hierarchy[2] < 0:
                #Innermost child components
                cv_image = cv2.drawContours(cv_image, c, -1, (0,255,0), 2 )
            """
        font = cv2.FONT_HERSHEY_SIMPLEX
        color = (0, 0 ,0)
        text_placement = (10,50)
        cv2.putText(cv_image,'STOP sign detected', text_placement, font, 2,color,1,cv2.LINE_AA)
        print(center_mass)

    except TypeError:
        font = cv2.FONT_HERSHEY_SIMPLEX
        color = (0, 0 ,0)
        text_placement = (10,50)
        cv2.putText(cv_image,'No STOP sign detected', text_placement, font, 2,color,1,cv2.LINE_AA)


    try:
        #Convert back to ROS format from CV and publish to the topic /my
        image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
    except CvBridgeError as e:
        print(e)

rospy.init_node('fc3')
image_pub = rospy.Publisher('/myresult', Image, queue_size=2)
image_sub = rospy.Subscriber('/cf1/camera/image_raw', Image, image_callback)

def main():
    rate = rospy.Rate(10) #Hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
