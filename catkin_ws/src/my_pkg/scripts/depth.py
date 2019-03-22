#!/usr/bin/env python

import pyrealsense2 as rs
import rospy
import numpy as np
import sys
import cv2
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist


def grid():
	pub = rospy.Publisher('depth_frame', Twist, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	try:
		pipeline = rs.pipeline()

		config = rs.config()
		config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
		config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
		# Start streaming
		pipeline.start(config)

		# filters
		hole_filling = rs.hole_filling_filter()

		# get camera intrinsics
		profile = pipeline.get_active_profile()
		depth_sensor = profile.get_device().first_depth_sensor()
		depth_scale = depth_sensor.get_depth_scale()

		col_length = int(640*(1.0/8.0))
		row_length = int(480*(1.0/5.0))

		while not rospy.is_shutdown():
			# This call waits until a new coherent set of frames is available on a devicepip3 install opencv-python
			# Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable
			print('Getting frame data now')
			frames = pipeline.wait_for_frames()
			depth_frame = frames.get_depth_frame()
			color_frame = frames.get_color_frame()

			# depth_frame = hole_filling.process(depth_frame)
			if not depth_frame or not color_frame:
			    continue

			depth_image = np.asanyarray(depth_frame.get_data())

			left_image = depth_image[ 2*row_length:3*row_length  , 0:col_length ]
			left_distances = depth_scale*left_image
			left_distances_filtered = left_distances[left_distances > 0]

			center_image = depth_image[ 2*row_length: 3*row_length, 3*col_length:5*col_length]
			center_distances = depth_scale*center_image
			center_distances_filtered = center_distances[center_distances > 0]

			right_image = depth_image[  2*row_length:3*row_length, 7*col_length:8*col_length ]
			right_distances = depth_scale*right_image
			right_distances_filtered = right_distances[right_distances > 0]
			# right_distances_projected = right_distances_filtered*math.sin(rad(42.6))

			mean = Twist()
			mean.linear.x = np.mean(left_distances_filtered)
			mean.linear.y = np.mean(center_distances_filtered)
			mean.linear.z = np.mean(right_distances_filtered)
			"""
			if DRAW_GRID:
			    color_image = np.asanyarray(color_frame.get_data())
			    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
			    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
			    # Stack both images horizontally
			    grid(color_image, w_portion, h_portion, 255, 255, 2)
			    images = np.hstack((color_image, depth_colormap))
			    cv2.imshow('RealSense', images)
			    cv2.waitKey(1)
			"""
			pub.publish(mean)
			rate.sleep()
		pipeline.stop()


	except Exception as e:
		print('except : ', e)
		pass

if __name__ == '__main__':
	try:
		grid()
	except rospy.ROSInterruptException:
		pass
