#!/usr/bin/env python

import rosbag


bag = rosbag.Bag('test.bag')
for topic, msg, t in bag.read_messages(topics=['pclouds']):
    print(msg)
bag.close()