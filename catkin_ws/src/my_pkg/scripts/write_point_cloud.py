#!/usr/bin/env python

import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

bag = rosbag.Bag('cloud.bag', 'w')
rospy.init_node('hello')

try:
	pcloud = PointCloud2()
	cloud = [[33,22,11],[55,33,22],[33,22,11]]
	pcloud = pc2.create_cloud_xyz32(pcloud.header, cloud)
	print(pcloud)
	bag.write('pclouds', pcloud)
	bag.write('pclouds', pcloud)
finally:
    bag.close()