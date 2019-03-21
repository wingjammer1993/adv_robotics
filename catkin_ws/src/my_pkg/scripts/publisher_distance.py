#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Float32

import numpy as np

def talker():
    pub = rospy.Publisher('depth_frame', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        distance = np.random.rand() 
        pub.publish(distance)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass