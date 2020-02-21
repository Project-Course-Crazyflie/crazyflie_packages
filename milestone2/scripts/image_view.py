#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge, CvBridgeError
import roslib
import rospy
from sensor_msgs.msg import Image

#publishe the camera_raw images at a slower rate

image_msg = None
bridge = CvBridge()

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
    rate = rospy.Rate(1) #Hz
    i=1

    while not rospy.is_shutdown():
        k = input("Press qny key to capture image")
        if image_msg:
            try:
                img = bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            except CvBridgeError as e:
                print(e)
            image_msg = None
            path = "~/Traning_set/image"+str(i)+".png"
            i+=1
            cv2.imwrite(path, img)
            print("image_pub saved!")
        rate.sleep()

if __name__ == '__main__':
	main()