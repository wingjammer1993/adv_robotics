#!/usr/bin/env python

import rosbag
import rospy
from std_msgs.msg import Int32MultiArray, String
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import ros_numpy

bag = rosbag.Bag('test.bag', 'w')
try:
	pcloud = PointCloud2()
	cloud = [[33,22,11],[55,33,22],[33,22,11]]
	pcloud = pc2.create_cloud_xyz32(pcloud.header, cloud)
	bag.write('pclouds', pcloud)
	bag.write('pclouds', pcloud)
finally:
    bag.close()


