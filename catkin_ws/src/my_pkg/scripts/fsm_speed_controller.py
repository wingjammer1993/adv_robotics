#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32, Int8, Bool
from geometry_msgs.msg import Twist
import numpy as np
import signal
import sys
import time
import math
import queue


class Echo(object):
    def __init__(self, threshold_large=3, threshold_small=2, delta=1):
        self.value = 0
        self.threshold_large = threshold_large
        self.threshold_small = threshold_small

        # Angular Controls
        self.right_turn = None
        self.right_timer = None
        self.turn_taken = False
        self.stgt_timer = None
        self.max_q_size = 6
        self.max_c_q_size = 3
        self.l_queue = queue.Queue(self.max_q_size)
        self.c_queue = queue.Queue(self.max_c_q_size)
        self.r_queue = queue.Queue(self.max_q_size)
        self.delta = delta
        self.curr_position = Twist()
        self.right_th = 5

        # State machine
        self.state = "orientation_1"
        
        # PID control values
        self.p = -1.2
        self.d = 0
        self.i = 0
        self.e_1 = 0
        self.e = 0
        
        # Initial speed value. The current speed is between the current value of speed and min_speed
        self.max_speed = 3.5
        rospy.init_node('echoer')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1, latch=True)
        rospy.Subscriber('depth_frame', Twist, self.update_value,queue_size=1)

    def state_controller(self):
        left = self.curr_position.linear.x
        center = self.curr_position.linear.y
        right = self.curr_position.linear.z

        l_q, r_q, c_q = [], [], []

        if self.stgt_timer is None:
            self.stgt_timer = time.time()

        if self.l_queue.qsize() >= self.max_q_size:
            l_q = np.array(self.l_queue.queue)
            r_q = np.array(self.r_queue.queue)

        if self.c_queue.qsize() >= self.max_c_q_size:
            c_q = np.array(self.c_queue.queue)

        rospy.loginfo("Current State: {}".format(self.state))
        steer, vel = 6, self.max_speed

        if self.state == "orientation_1":
            steer, vel = self.orientation_1(c_q, l_q, r_q)
        elif self.state == "right_turn_1":
            steer, vel = self.right_turn_timer(0.8, 4, self.orientation_2)
        elif self.state == "orientation_2":
            steer, vel = self.orientation_2(c_q, l_q, r_q)
        elif self.state == "right_turn_2":
            steer, vel = self.right_turn_timer(0.8, 6, self.orientation_3)
        elif self.state == "orientation_3":
            steer, vel = self.orientation_3(c_q, l_q, r_q)
        else:
            steer, vel = self.accelerate(c_q)

        return steer, vel

    def accelerate(self, c_q):
        if np.mean(c_q) < 12:
            return 6, max(1.5, 0.25 * np.mean(c_q))
        else:
            return 6, self.max_speed

    def orientation_3(self, c_q, l_q, r_q):
        if len(l_q) != 0 and np.all(np.diff(l_q) <= 0) and np.mean(l_q) < np.mean(r_q):
            return self.left_orientation()
        elif len(r_q) != 0 and np.all(np.diff(r_q) <= 0) and np.mean(r_q) < np.mean(l_q):
            return self.right_orientation()
        else:
            return self.accelerate(c_q)

    def orientation_2(self, c_q, l_q, r_q):
        if len(l_q) != 0 and np.all(np.diff(l_q) <= 0) and np.mean(l_q) < np.mean(r_q):
            return self.left_orientation()
        elif len(r_q) != 0 and np.all(np.diff(r_q) <= 0) and np.mean(r_q) < np.mean(l_q):
            return self.right_orientation()
        elif len(c_q) != 0 and np.all(np.diff(c_q) <= 0) and c_q[-1] < self.right_th and time.time() - self.stgt_timer > 7:
            self.state = "right_turn_2"
            return self.right_turn_2(c_q, c_q[-1])
        else:
            return self.accelerate(c_q)

    def orientation_1(self, c_q, l_q, r_q):
        if len(c_q) != 0 and np.all(np.diff(c_q) <= 0) and c_q[-1] < self.right_th  and time.time() - self.stgt_timer > 5:
            self.state = "right_turn_1"
            return self.right_turn_1(c_q, c_q[-1])
        elif len(l_q) != 0 and np.all(np.diff(l_q) <= 0):
            return self.left_orientation()
        elif len(r_q) != 0 and np.all(np.diff(r_q) <= 0):
            return self.right_orientation()
        else:
            return self.accelerate(c_q)

    def left_orientation(self):
        rospy.loginfo("Orientation left")

        return 7, self.max_speed

    def right_orientation(self):
        rospy.loginfo("Orientation right")

        return 4.5, self.max_speed

    def right_turn_timer(self, time_th, next_right_th, next_state):
        if self.right_turn is not None and self.right_timer is not None and (time.time() - self.right_timer) < time_th:
            rospy.loginfo("In timer")

            return 10, 1.2
        elif self.right_turn is True:
            self.right_turn = False
            self.right_timer = None
            self.turn_taken = True
            self.right_th = next_right_th

            self.l_queue = queue.Queue(self.max_q_size)
            self.c_queue = queue.Queue(self.max_c_q_size)
            self.r_queue = queue.Queue(self.max_q_size)
            self.state = next_state.__name__
            self.stgt_timer = time.time()
            return next_state([], [], [])

    def right_turn_1(self, c_q, center):
        self.right_turn = True
        self.right_timer = time.time()

        return 10, 0.5

    def right_turn_2(self, c_q, center):
        self.right_turn = True
        self.right_timer = time.time()

        return 10, 0.5

    def signal_handler(self, sig, frame):
        velocity_control = Twist()
        velocity_control.linear.x = 0
        velocity_control.angular.z = 6
        print("sigint called")
        self.pub.publish(velocity_control)
        sys.exit(0)

    def update_value(self, msg):
        self.curr_position = msg
        if self.l_queue.qsize() >= self.max_q_size:
            self.l_queue.get()
        if self.c_queue.qsize() >= self.max_c_q_size:
            self.c_queue.get()
        if self.r_queue.qsize() >= self.max_q_size:
            self.r_queue.get()

        if self.curr_position.linear.x > 7:
            self.curr_position.linear.x = 7
        elif self.curr_position.linear.y > 15:
            self.curr_position.linear.y = 15
        elif self.curr_position.linear.z > 7:
            self.curr_position.linear.z = 7

        self.l_queue.put(self.curr_position.linear.x)
        self.c_queue.put(self.curr_position.linear.y)
        self.r_queue.put(self.curr_position.linear.z)

        rospy.loginfo(self.curr_position)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Degrading speed. Done for midterm.
            velocity_control = Twist()
            velocity_control.angular.z, velocity_control.linear.x = self.state_controller()
            self.pub.publish(velocity_control)
            r.sleep()


if __name__ == '__main__':
    try:
        #Testing our function
        echo = Echo()
        signal.signal(signal.SIGINT, echo.signal_handler)
        echo.run()
    except rospy.ROSInterruptException: pass

