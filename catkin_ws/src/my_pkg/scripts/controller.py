#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import numpy as np
import signal
import sys


class Echo(object):
    def __init__(self, threshold_large=3, threshold_small=2, delta=1):
        self.value = 0
        self.threshold_large = threshold_large
        self.threshold_small = threshold_small
        self.l_threshold = 4
        self.center = (self.threshold_large + self.threshold_small)*0.5
        self.delta = delta
        self.curr_position = Twist()
        
        self.p = 1
        self.d = 100
        self.i = 1000
        self.e_1 = 0
        self.e = 0
        
        rospy.init_node('echoer')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1, latch=True)
        rospy.Subscriber('depth_frame', Twist, self.update_value)

    def Pcontrol_steer(self):
        self.e_1 = self.e
        left = self.curr_position.linear.x
        center = self.curr_position.linear.y
        right = self.curr_position.linear.z
        if center > 3:
            return 5
        else:
            self.e = left - right
            return 5 + self.p*(self.e) + self.d*(self.e_1 - self.e) + self.i*(self.e_1 + self.e)

    def signal_handler(self, sig, frame):
        velocity_control = Twist()
        velocity_control.linear.x = 0
        velocity_control.angular.z = 5
        print("sigint called")
        self.pub.publish(velocity_control)
        sys.exit(0)

    def update_value(self, msg):
        self.curr_position = msg
        rospy.loginfo(self.curr_position)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            velocity_control = Twist()
            velocity_control.linear.x = 2
            velocity_control.angular.z = self.Pcontrol_steer()
            self.pub.publish(velocity_control)
            r.sleep()


if __name__ == '__main__':
    try:
        #Testing our function
        echo = Echo()
        signal.signal(signal.SIGINT, echo.signal_handler)
        echo.run()
    except rospy.ROSInterruptException: pass
