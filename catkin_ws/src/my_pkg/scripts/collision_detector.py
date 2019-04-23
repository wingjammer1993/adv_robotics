#!/usr/bin/env python

import rospy
import numpy as np
import sys
import cv2
import queue
import time
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# from skimage.measure import compare_ssim as ssim
from skimage.measure import compare_mse


class CollisionDetector(object):
	def __init__(self, queue_size=5, delay=5, coll_th=5):
		self.image_queue = queue.Queue(queue_size)
		self.image_queue_size = queue_size
		self.bridge = CvBridge()

		# Delay the detection to overcome initial setup.
		self.start_time = time.time()
		self.induced_delay = delay

		# Image MSE threshold
		self.img_mse_th = coll_th

		rospy.init_node("collision_detector", anonymous=True)

		self.pub = rospy.Publisher('/collision', Int8, queue_size=10, latch=True)
		rospy.Subscriber('rgb_frame', Image, self.receive_image, queue_size=1)

	def receive_image(self, msg):
		curr_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

		if not self.wait_for_delay():
			if self.image_queue.qsize() >= self.image_queue_size:
				self.image_queue.get() # pop the old image

			self.image_queue.put(curr_image)

	def wait_for_delay(self):
		if int(time.time() - self.start_time) < self.induced_delay:
			rospy.loginfo("Waiting for initial delay")
			return True
		else:
			return False

	def detect_collision(self):
		if self.image_queue.qsize() == self.image_queue_size:
			images = np.array(self.image_queue.queue)
			curr_image = cv2.cvtColor(images[-1, :, :, :], cv2.COLOR_BGR2GRAY)

			for index in range(images.shape[0] - 1):
				img = cv2.cvtColor(images[index, :, :, :], cv2.COLOR_BGR2GRAY)

				# sim_score = ssim(curr_image, img)

				# difference = cv2.subtract(curr_image, img)
				# mean_variation = np.sum(difference) / np.prod(img.shape)
				# rospy.loginfo("Mean Variation: {}".format(mean_variation))

				mse = compare_mse(curr_image, img)

				if mse > self.img_mse_th:
					return False

			return True



def run(det):
	r = rospy.Rate(10)

	while not rospy.is_shutdown():
		msg = Int8(0)

		if det.detect_collision():
			msg = Int8(1)

		det.pub.publish(msg)
		r.sleep()


if __name__ == '__main__':
	try:
		detector = CollisionDetector()
		run(detector)
	except rospy.ROSInterruptException:
		pass
