#!/usr/bin/env python

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import roslib
from std_msgs.msg import String
import numpy as np
import rospy

def image_callback(img_msg):
    #convert image from ROS format to CV
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
    except CvBridgeError as e:
        print(e)
    
    # Convert BGR to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of the color we look for in the HSV space, trial and error kind of (HEX to HSV)
    lower= np.array([170, 100, 100])
    upper = np.array([180, 255, 255])

    # Threshold the HSV image to get only the pixels in ranage
    gray = cv2.inRange(hsv, lower, upper)

    #Blur with Gaussian Filtering, NO BLUR SINCE IT DETECTS THE SIGNS, but you need it in irl because of much noise
    #gray = cv2.GaussianBlur(gray, (5,5),  0)

    kernel = np.ones((5,5),np.uint8)
    
    #Erosion
    gray = cv2.erode(gray, kernel, iterations = 1)
    #Dilate
    gray = cv2.dilate(gray, kernel, iterations = 1)

    
    _, contours, hierarchy = cv2.findContours(gray,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)

    
    try:
        #Get the actual inner list of the hierarchy
        hierarchy = hierarchy[0]
        
        #(Empty) List with walls and their center of mass coordinates
        center_mass = []

        #Calculating center of mass and drawing contours
        for i, c in enumerate(contours):
            current_hierarchy = hierarchy[i]
            
            if current_hierarchy[3] < 0:
                #Outermost parent components
                M = cv2.moments(c)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                center_mass.append((cx, cy))
                cv_image = cv2.drawContours(cv_image, c, -1, (255,0,0), 2)
                cv_image = cv2.circle(cv_image,(cx,cy), 2, (255,0,0), -1 )
                
            elif current_hierarchy[2] < 0:
                #Innermost child components
                cv_image = cv2.drawContours(cv_image, c, -1, (0,255,0), 2 )
            
        print("Center of mass: " + str(center_mass))
    
    except TypeError:
        print("No walls detected")

    try:
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
