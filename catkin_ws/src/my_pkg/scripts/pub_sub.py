#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import numpy as np

class Echo(object):
    def __init__(self, threshold_large=0.5, threshold_small=0.2, delta=1):
        self.value = 0
        self.threshold_large = threshold_large
        self.threshold_small = threshold_small
        self.center = (self.threshold_large + self.threshold_small)*0.5
        self.delta = delta
        rospy.init_node('echoer')

        self.pub = rospy.Publisher('/cmd_vel', Twist, latch=True)
        rospy.Subscriber('depth_frame', Float32, self.update_value)

    def Pcontrol_steer():
    	p_control = 0
    	if self.threshold_large > self.curr_position & self.curr_position < self.threshold_small:
    		p_control = self.curr_position - self.center
    	elif self.threshold_large < self.curr_position
    		p_control = self.curr_position - self.center

    	p_scaled = p_control/self.center
    	return 5 + self.delta*p_scaled

    def Pcontrol_xvel():
    	return 4

    def update_value(self, msg):
        self.curr_position = msg.data
        rospy.loginfo(self.curr_position)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            velocity_control = Twist()
            velocity_control.linear.x = Pcontrol_xvel()
            velocity_control.angular.z = Pcontrol_steer()
            self.pub.publish(velocity_control)
            r.sleep()


if __name__ == '__main__':
    try:
        #Testing our function
        echo = Echo()
        echo.run()
    except rospy.ROSInterruptException: pass