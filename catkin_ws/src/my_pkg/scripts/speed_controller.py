#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32, Int8, Bool
from geometry_msgs.msg import Twist
import numpy as np
import signal
import sys
import time
import queue


class Echo(object):
    def __init__(self, threshold_large=3, threshold_small=2, delta=1):
        self.value = 0
        self.threshold_large = threshold_large
        self.threshold_small = threshold_small
      

        # Angular Controls
        self.l_threshold = 4
        self.center = (self.threshold_large + self.threshold_small)*0.5
        self.delta = delta
        self.curr_position = Twist()
        
        # PID control values
        self.p = -1.2
        self.d = 0
        self.i = 0
        self.e_1 = 0
        self.e = 0
        
        # Initial speed value. The current speed is between the current value of speed and min_speed
        self.speed = 1.75
        self.min_speed = 1.75       
        rospy.init_node('echoer')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1, latch=True)
        rospy.Subscriber('depth_frame', Twist, self.update_value,queue_size=1)

    def Pcontrol_steer(self):
        
        left = self.curr_position.linear.x
        center = self.curr_position.linear.y
        right = self.curr_position.linear.z

        if center > 4 and center > right:
            return 6
        elif center - 1.5 < right
            return 6 + 2
        else:
            self.e = (left - right)
            n_steer = 6 + self.p*(self.e) + self.d*(self.e_1 - self.e) + self.i*(self.e_1 + self.e)


    def signal_handler(self, sig, frame):
        velocity_control = Twist()
        velocity_control.linear.x = 0
        velocity_control.angular.z = 4
        print("sigint called")
        self.pub.publish(velocity_control)
        sys.exit(0)

    def update_value(self, msg):
        self.curr_position = msg
        rospy.loginfo(self.curr_position)

    def control_linear_speed(self):
         return max(self.min_speed, self.speed)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Degrading speed. Done for midterm.
            self.speed -= 0.05
            velocity_control = Twist()
            velocity_control.linear.x = self.control_linear_speed()
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
