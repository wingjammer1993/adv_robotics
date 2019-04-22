#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32, Int8, Bool
from geometry_msgs.msg import Twist
import numpy as np
import signal
import sys
import time


class Echo(object):
    def __init__(self, threshold_large=3, threshold_small=2, delta=1):
        self.value = 0
        self.threshold_large = threshold_large
        self.threshold_small = threshold_small

        # Collision controls
        self.max_back_speed = -2.0
        self.collision_occr = False
        self.max_reverse_time = 2
        self.collision_time = None

        # Stop sign controls
        self.stop_sign_detected = False
        self.brake_value = -0.5

        # Angular Controls
        self.l_threshold = 4
        self.center = (self.threshold_large + self.threshold_small)*0.5
        self.delta = delta
        self.curr_position = Twist()
        
        # PID control values
        self.p = -0.8
        self.d = 0
        self.i = 0
        self.e_1 = 0
        self.e = 0
        
        # Initial speed value. The current speed is between the current value of speed and min_speed
        self.speed = 3.1
        self.min_speed = 2.0
        
        rospy.init_node('echoer')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1, latch=True)
        rospy.Subscriber('depth_frame', Twist, self.update_value)
        rospy.Subscriber('collision', Int8, self.save_collision)
        rospy.Subscriber('stop_sign/out', Bool, self.save_stop_sign)


    def Pcontrol_steer(self):
        if self.collision_occr or self.stop_sign_detected:
            return 0.0

        self.e_1 = self.e
        left = self.curr_position.linear.x
        center = self.curr_position.linear.y
        right = self.curr_position.linear.z
        
        if center > 4 and center > right:
            return 5 + 1
        elif center - 1.5 < right:
            return 5 + 3
        else:
            self.e = (left - right)
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
        # rospy.loginfo(self.curr_position)

    def save_stop_sign(self, msg):
        self.stop_sign_detected = msg.data
        rospy.loginfo("Stop sign: {}".format(self.stop_sign_detected))

    def save_collision(self, msg):
        if msg.data == 1 and not self.collision_occr:
            rospy.loginfo("Collision Detected. Backing out")
            self.collision_occr = True
            self.collision_time = time.time()

    def check_collision(self):
        # Control the max reverse distance after collision
        if self.collision_occr and int(time.time() - self.collision_time) < self.max_reverse_time:
            return True
        else:
            self.collision_occr = False

            return False

    def control_linear_speed(self):
        if self.stop_sign_detected:
            return self.brake_value
        elif self.check_collision():
            return self.max_back_speed
        else:
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
