#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import numpy as np

class Echo(object):
    def __init__(self):
        self.value = 0

        rospy.init_node('echoer')

        self.pub = rospy.Publisher('/cmd_vel', Twist, latch=True)
        rospy.Subscriber('depth_frame', Float32, self.update_value)

    def update_value(self, msg):
        self.value = msg.data
        rospy.loginfo(self.value)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            message = Twist()
            message.angular.z = self.value*10
            self.pub.publish(message)
            r.sleep()


if __name__ == '__main__':
    try:
        #Testing our function
        echo = Echo()
        echo.run()
    except rospy.ROSInterruptException: pass