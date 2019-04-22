#!/usr/bin/env python

import pyrealsense2 as rs
import rospy
import numpy as np
import sys
import cv2
import queue
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CollisionDetector(object):
	def __init__(self, queue_size=5):
		self.image_queue = queue.Queue(queue_size)
		self.image_queue_size = queue_size
		self.bridge = CvBridge()

		rospy.init_node("collision_detector")

		self.pub = rospy.Publisher('/collision', Bool, queue_size=30)
		rospy.Subscriber('rgb_frame', Image, self.receive_image)


	def receive_image(self, msg):
		curr_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

		if self.image_queue.qsize() >= self.image_queue_size:
			self.image_queue.get() # pop the old image

		cv2.imwrite("/tmp/test.jpg", curr_image)
		self.image_queue.put(curr_image)


	def run(self):
		r = rospy.Rate(10)
		
        while not rospy.is_shutdown():
        	self.pub.publish(False)
        	r.sleep()


if __name__ == '__main__':
	try:
		detector = CollisionDetector()
		detector.run()
	except rospy.ROSInterruptException:
		pass
