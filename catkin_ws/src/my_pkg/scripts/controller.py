#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import numpy as np
import signal



signal.signal(signal.SIGINT, Echo.signal_handler)


class Echo(object):
    def __init__(self, threshold_large=2, threshold_small=1, delta=1):
        self.value = 0
        self.threshold_large = threshold_large
        self.threshold_small = threshold_small
        self.center = (self.threshold_large + self.threshold_small)*0.5
        self.delta = delta
        self.curr_position = 0
        rospy.init_node('echoer')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5, latch=True)
        rospy.Subscriber('depth_frame', Float32, self.update_value)

    def Pcontrol_steer(self):
    	p_control = 0
    	if self.threshold_large > self.curr_position and self.curr_position < self.threshold_small:
    		p_control = self.curr_position - self.center
    	elif self.threshold_large < self.curr_position:
    		p_control = self.curr_position - self.center
    	return 5 + self.delta*p_control
    
    @staticmethod
    def signal_handler(sig, frame):
        velocity_control = Twist()
        velocity_control.linear.x = 0
        velocity_control.angular.z = 5
        self.pub.publish(velocity_control)

    def update_value(self, msg):
        self.curr_position = msg.data
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
        echo.run()
    except rospy.ROSInterruptException: pass
