#!/usr/bin/env python
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import roslib
import rospy
from sensor_msgs.msg import Image
import os.path
import glob


#publishe the camera_raw images at a slower rate

image_msg = None
bridge = CvBridge()
mtx  = np.load('mtx.npy')
dist = np.load('dist.npy')
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(640,480),1,(640,480))

def callback(msg):
    global image_msg
    image_msg = msg
    return

def publish(msg):
    image_pub.publish(msg)
    return


sub_pose = rospy.Subscriber('cf1/camera/image_raw', Image, callback)
image_pub = rospy.Publisher('/training_img', Image, queue_size=5)
rospy.init_node('traning_data')

def main():
    global image_msg
    rate = rospy.Rate(10) #Hz
    files = glob.glob("/home/robot/Traning_set/*.png")
    files.sort()
    i = 0
    print(files)
    if len(files) != 0 :
        string = files[-1].split('/')[-1]
        i = (int(string[5:-4]))+1
    print(i)
    



    while not rospy.is_shutdown():
        raw_input("Press any key to capture image ")
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

            n_0 = 4 - len(str(i))
            n = '0' * n_0

            filename = 'image' + n + str(i) + '.png'
            path = os.path.join(os.path.expanduser('~'),'Traning_set',filename)
            i+=1
            if not cv2.imwrite(path, dst):
                raise Exception("Could not write image")
        rate.sleep()

if __name__ == '__main__':
	main()