#!/usr/bin/env python
#This file originates from https://github.com/ultralytics/yolov3.git (detect.py)
#However it has been significantly changed
from sys import platform


import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import roslib
import rospy
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import MultiArrayDimension, Int32MultiArray


from models import *  # set ONNX_EXPORT in models.py
from datasets import *
from utils import *


def main():

	global image_msg

	with torch.no_grad():
		#Load the model and everything else that should only be done once.
		img_size,out, source, weights, half, view_img, save_txt = opt.img_size, opt.output, opt.source, opt.weights, opt.half, opt.view_img, opt.save_txt
		device = torch_utils.select_device(opt.device)

		model = Darknet(opt.cfg, img_size)
		if weights.endswith('.pt'):  # pytorch format
			model.load_state_dict(torch.load(weights, map_location=device)['model'])
		else:  # darknet format
			load_darknet_weights(model, weights)

		# set model to Eval mode (rather than training mode)
		model.to(device).eval()

		# Get names and colors
		names = load_classes(opt.names)
		colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]


		"""
		-----------------------------------------------------------------------------
		"""
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			rate.sleep()

			if image_msg:
				t1 = torch_utils.time_synchronized()
				boxes = []

				#Run the model and send result-message
				#Variable explanations:::::::::::::::
				#im0 is the original image
				#img is the scaled image to size opt.img_size
				#pred is the yolo network output
				#

				# Read image
				try:
					im0 = bridge.imgmsg_to_cv2(image_msg, 'bgr8')
				except CvBridgeError as e:
					rospy.logwarn(e)
				image_msg = None

				# resize by 0-padding
				if opt.img_size != tuple(im0.shape[0:2]):
					img = letterbox(im0, new_shape=opt.img_size*2)[0]
				else:
					img = im0

				# Convert from BGR to RGB
				img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
				img = np.ascontiguousarray(img)




				#Normalize pixel values
				img = torch.from_numpy(img).to(device)
				img = img.float() / 255.0  # uint8 to fp16/32 # 0 - 255 to 0.0 - 1.0
				if img.ndimension() == 3:
					img = img.unsqueeze(0)

				# Inference
				#
				pred = model(img)[0]
				#

				# Apply NMS
				pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)

				cboxes = [] #same as boxes but retaining the centered coordinates

				for detection in pred:
					if not detection:
						continue
					# extract the class ID and confidence (i.e., probability) of
					# the current object detection
					scores = detection[5:]
					classID = np.argmax(scores)
					conf = scores[classID]#detection[4]#
					# filter out weak predictions by ensuring the detected
					# probability is greater than the minimum probability
					if conf > confidence:# and detection[classID] > 0.5:
						# scale the bounding box coordinates back relative to the
						# size of the image, keeping in mind that YOLO actually
						# returns the center (x, y)-coordinates of the bounding
						# box followed by the boxes' width and height
						box = detection[0:4] * np.array([W, H, W, H])
						(centerX, centerY, width, height) = box.astype(np.int32)

						cboxes.extend([centerX, centerY, classID, height, width, stamp, stamp_ns])

				detected = False
				# Process detections
				for i, det in enumerate(pred):  # detections per image
					print('det: ', det)

					s = ''

					s += '%gx%g ' % img.shape[2:]  # print string
					if det is not None and len(det):
						# Rescale boxes from img_size to im0 size
						det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()


						# Print results
						for c in det[:, -1].unique():
							n = (det[:, -1] == c).sum()  # detections per class
							s += '%g %ss, ' % (n, names[int(c)])  # add to string

						# Write results
						#for *xyxy, conf, cls in det:

						for A in det:
							conf = A[0]
							cls = A[1]
							xyxy = A[2:]
							if view_img:  # Add bbox to image and send to /boxed_image
								detected = True
								label = '%s %.2f' % (names[int(cls)], conf)
								im0 = plot_one_box(xyxy, im0, label=label, color=colors[int(cls)])


					# Print time (inference + NMS)
					t2 = torch_utils.time_synchronized()
					print('%sDone. (%.3fs)' % (s, t2 - t1))
					#print('possible fps: ', 1/(t2 - t1))
					# Stream results
					if detected: # Only publish results if there was a detected sign
						print('sign detected')
						if view_img:
							img_publish(im0)

					# Save results (image with detections)
					# Send message
					box_publish(boxes) # [centerX, centerY, classID, height, width, stamp, stamp_ns]

					print('time from raw image to box sent is: ', t2 - t1)




class options:
	def __init__(self):
		self.cfg = 'yolov3-tiny.cfg'
		self.weights = 'best-tiny-all.pt'
		self.classes = 1
		self.names = 'three_signs.names'
		self.source = '////////////////////////////////'
		self.output = '//////////////////////////////'
		self.img_size = [640]
		self.conf_thres = 0.3
		self.iou_thres = 0.6
		self.fourcc = 'mp4v'
		self.half = False
		self.device = 'cpu'
		self.view_img = True
		self.save_txt = False
		self.agnostic_nms = False


image_msg = None
bridge = CvBridge()

def callback(msg):
	global image_msg
	image_msg = msg
	return

def img_publish(img):
	try:
		#Convert back to ROS format from CV and publish to the topic /myresult
		image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
		#print('published')
	except CvBridgeError as e:
		print(e)
	return

def box_publish(boxes):
	msg = Int32MultiArray()
	msg.layout.dim = [MultiArrayDimension(),MultiArrayDimension()]
	n = 6
	msg.layout.dim[0].label  = "n_boxes"
	msg.layout.dim[0].size   = len(boxes)/n
	msg.layout.dim[0].stride = len(boxes)
	msg.layout.dim[1].label  = "nums"
	msg.layout.dim[1].size   = n
	msg.layout.dim[1].stride = n
	msg.layout.data_offset = 0

	msg.data = boxes
	box_pub.publish(msg)
	return

rospy.init_node('vision')
sub_pose = rospy.Subscriber('cf1/camera/image_raw', ImageMsg, callback)
image_pub = rospy.Publisher('/boxed_image', ImageMsg, queue_size=1)
box_pub = rospy.Publisher('/sign_box', Int32MultiArray, queue_size=1)

if __name__ == '__main__':
	opt = options()
	print('options:\n',opt,'\n\n')

	main()
