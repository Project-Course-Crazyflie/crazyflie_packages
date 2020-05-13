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
				cboxes = []

				#Run the model and send result-message
				#Variable explanations:::::::::::::::
				#im0 is the original image
				#img is the scaled image to size opt.img_size
				#pred is the yolo network output
				#

				# Read image
				try:
					im0 = bridge.imgmsg_to_cv2(image_msg, 'rgb8')
					#print('im0:', im0.shape)
				except CvBridgeError as e:
					rospy.logwarn(e)
				stamp = image_msg.header.stamp.secs
				stamp_ns = image_msg.header.stamp.nsecs
				image_msg = None

				# resize by 0-padding
				if opt.img_size != tuple(im0.shape[0:2]):
					img = letterbox(im0, new_shape=opt.img_size*2)[0]
				else:
					img = im0
				#print('img',img.shape)

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
				#print(pred[0][0])
				pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
				#remove boxes that overlap, keep only the one with highest conf
				#make sure that boxes for different classes do not affect each other

				detected = False

				# Process detections
				for i, det in enumerate(pred):  # detections per image

					s = str(img.shape[2]) + 'x' + str(img.shape[3])#'%gx%g ' % img.shape[2:]  # print string
					if det is not None and len(det):

						detected = True
						# Rescale boxes from img_size to im0 size
						det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()


						# Print results
						for c in det[:, -1].unique():
							n = (det[:, -1] == c).sum()  # detections per class
							s += ' ' + str(int(n)) + ' ' + names[int(c)] + 's |'#'%g %ss, ' % (n, names[int(c)])  # add number of 'name's to string

						for A in det:
							conf = A[4]
							cls = A[-1]
							xyxy = A[:4]
							#print('xyxy: ', xyxy)
							if any(xyxy[2:] <= 1):
								if len(det) == 1:
									detected = False
									break
								continue
							if view_img:  # Add bbox to image and send to /boxed_image
								label = names[int(c)] + str(conf)#'%s %.2f' % (names[int(cls)], conf)
								im0 = plot_one_box(xyxy, im0, label=label, color=colors[int(cls)])

							conf = A[4]
							tmp = xyxy2xywh(xyxy.unsqueeze(0))
							(centerX, centerY, width, height) = (tmp[0,0],tmp[0,1],tmp[0,2],tmp[0,3])
							print('cls: ' + str(cls))
							cboxes.extend([int(centerX), int(centerY), int(cls), int(height), int(width), stamp, stamp_ns])

				if detected:

					# Send message
					box_publish(cboxes) # [centerX, centerY, classID, height, width, stamp, stamp_ns]
					#print('time from raw image to box sent is: ', t2 - t1)

					if view_img:
						t2 = torch_utils.time_synchronized()
						print(s + 'Done in ' + str(t2 - t1) + ' seconds')
						img_publish(im0)

					# Save results (image with detections)







class options:
	def __init__(self):
		self.cfg = rospy.get_param(rospy.get_name() + '/cfg')
		self.weights = rospy.get_param(rospy.get_name() + '/weights')
		self.classes = [0,1,2]
		self.names = rospy.get_param(rospy.get_name() + '/names')
		self.source = '////////////////////////////////'
		self.output = '//////////////////////////////'
		self.img_size = [640]
		self.conf_thres = 0.3
		self.iou_thres = 0.3
		self.fourcc = 'mp4v'
		self.half = False
		self.device = ''
		self.view_img = rospy.get_param(rospy.get_name() + '/view_img')
		self.save_txt = False
		self.agnostic_nms = True


image_msg = None
bridge = CvBridge()

def callback(msg):
	global image_msg
	image_msg = msg
	return

def img_publish(img):
	try:
		#Convert back to ROS format from CV and publish to the topic /myresult
		image_pub.publish(bridge.cv2_to_imgmsg(img, 'rgb8'))
		#print('published')
	except CvBridgeError as e:
		print(e)
	return

def box_publish(boxes):
	msg = Int32MultiArray()
	msg.layout.dim = [MultiArrayDimension(),MultiArrayDimension()]
	n = 7 #Gandalfs number
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
sub_pose = rospy.Subscriber('cf1/camera/image_undist', ImageMsg, callback)
image_pub = rospy.Publisher('/boxed_image', ImageMsg, queue_size=1)
box_pub = rospy.Publisher('/sign_box', Int32MultiArray, queue_size=1)

if __name__ == '__main__':
	opt = options()
	print('options:\n',opt,'\n\n')

	main()
