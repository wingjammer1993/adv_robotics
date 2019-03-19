#!/usr/bin/env python

import rosbag
bag = rosbag.Bag('cloud.bag', 'w')

for topic, msg, t in bag.read_messages(topics=['pclouds']):
	print(topic)
	print(t)
	print(msg)
bag.close()