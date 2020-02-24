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

image_msg = None
bridge = CvBridge()


def callback(msg):
    global image_msg
    image_msg = msg
    return



sub_pose = rospy.Subscriber('cf1/camera/image_undist', Image, callback)
rospy.init_node('traning_data')

def main():
    global image_msg
    rate = rospy.Rate(10) #Hz
    files = glob.glob("/home/robot/Traning_set/*.png")
    files.sort()
    nb = 0      #nb of images saved
    i = 0       #where to start the image names
    
    if len(files) != 0 :
        string = files[-1].split('/')[-1]
        i = (int(string[5:-4]))+1
    


    while not rospy.is_shutdown():
        k = raw_input("Press any key to capture image, q to quit: ")
        if k == 'q':
            print(str(nb)+' images saved!')
            rospy.signal_shutdown('')
            continue

        if image_msg:
            try:
                img = bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            except CvBridgeError as e:
                print(e)
            image_msg = None

            n_0 = 4 - len(str(i))
            n = '0' * n_0

            filename = 'image' + n + str(i) + '.png'
            path = os.path.join(os.path.expanduser('~'),'Traning_set',filename)
            i += 1
            if not cv2.imwrite(path, img):
                raise Exception("Could not write image")
            nb += 1
        rate.sleep()

if __name__ == '__main__':
	main()