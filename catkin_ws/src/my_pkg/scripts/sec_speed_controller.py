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
        self.max_q_size = 5
        self.max_c_q_size = 4
        self.l_queue = queue.Queue(self.max_q_size)
        self.c_queue = queue.Queue(self.max_c_q_size)
        self.r_queue = queue.Queue(self.max_q_size)
        self.delta = delta
        self.curr_position = Twist()
        self.right_th = 6.5
        
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

        l_q, r_q, c_q = [], [], []

        if self.right_turn is not None and self.right_timer is not None and (time.time() - self.right_timer) < 0.8:
            rospy.loginfo("In timer")
            return 10, 1.2
        elif self.right_turn is True:
            self.right_turn = False
            self.right_timer = None
            self.turn_taken = True
            self.right_th = 4.5

            self.l_queue = queue.Queue(self.max_q_size)
            self.c_queue = queue.Queue(self.max_c_q_size)
            self.r_queue = queue.Queue(self.max_q_size)

            #self.stgt_timer = time.time()
            # return 6.5, 2.5

        if self.l_queue.qsize() >= self.max_q_size:
            l_q = np.array(self.l_queue.queue)
            r_q = np.array(self.r_queue.queue)
        if self.c_queue.qsize() >= self.max_c_q_size:
            c_q = np.array(self.c_queue.queue)

        rospy.loginfo("Center mean: {}".format(np.mean(c_q)))
        if self.turn_taken and np.mean(c_q) > 10:
            self.turn_taken = 1

        if len(c_q) != 0 and np.all(np.diff(c_q) <= 0) and center < self.right_th and not self.turn_taken:
            self.right_turn = True
            self.right_timer = time.time()
            rospy.loginfo("Right turn 1")
            return 10, 0.5
        elif len(l_q) != 0 and np.all(np.diff(l_q) <= 0):
            rospy.loginfo("Orientation left")
            return 6.5, 3.5
        elif len(r_q) != 0 and np.all(np.diff(r_q) <= 0):
            rospy.loginfo("Orientation right")
            return 4.5, 3.5
        elif len(c_q) != 0 and np.all(np.diff(c_q) <= 0) and center < self.right_th and self.turn_taken == 1:
            self.right_turn = True
            self.turn_taken = True
            self.right_timer = time.time()
            rospy.loginfo("Right turn 2")
            return 10, 0.5
        #elif len(c_q) != 0 and np.all(np.diff(c_q) <= 0) and center < 4:
        #    return 8, 1.75
        else:
            if np.mean(c_q) < 12:
                rospy.loginfo("vachan")
                return 6, max(1.5, 0.25 * np.mean(c_q))
            else:
                if self.right_turn is True:
                    return 10, 1.2
                return 6, 3.5

    def Pcontrol_steer2(self):
        rospy.loginfo("PID 2")
       # self.e_1 = self.e
       # left = self.curr_position.linear.x
       # center = self.curr_position.linear.y
       # right = self.curr_position.linear.z


       # if center > 4 and center > right:
       #     if center > 10:
       #         return 6, 3.7
       #     else:            
       #         return 6, min(1.5, 0.25*center)
       # elif center < right:
       #     return 5 + 5, 1.2
       # else:
       #     self.e = (left - right)
       #     n_steer = 6 + self.p*(self.e) + self.d*(self.e_1 - self.e) + self.i*(self.e_1 + self.e)

        #    if n_steer < 3:
        #        return 3, 3.7
        #    elif n_steer > 7:
        #        return 8, 3.7
        #    else:
        #        return n_steer, 3.7
        
        left = self.curr_position.linear.x
        center = self.curr_position.linear.y
        right = self.curr_position.linear.z

        l_q, r_q, c_q = [], [], []

        if self.right_turn is not None and self.right_timer is not None and (time.time() - self.right_timer) < 0.8:
            rospy.loginfo("In timer")
            return 10, 1.2
        elif self.right_turn is True:
            self.right_turn = False
            self.right_timer = None
            self.turn_taken = True
            self.right_th = 4.5

            self.l_queue = queue.Queue(self.max_q_size)
            self.c_queue = queue.Queue(self.max_c_q_size)
            self.r_queue = queue.Queue(self.max_q_size)

            #self.stgt_timer = time.time()
            # return 6.5, 2.5

        if self.stgt_timer is not None:
            if center < 4 and time.time() - self.stgt_timer < 0.2:
                rospy.loginfo("Stablizing")
                self.max_c_q_size = 5
                return 6, 2.5
            else:
                self.l_queue = queue.Queue(self.max_q_size)
                self.c_queue = queue.Queue(self.max_c_q_size)
                self.r_queue = queue.Queue(self.max_q_size)
                self.stgt_timer = None

        if self.l_queue.qsize() >= self.max_q_size:
            l_q = np.array(self.l_queue.queue)
            r_q = np.array(self.r_queue.queue)
        if self.c_queue.qsize() >= self.max_c_q_size:
            c_q = np.array(self.c_queue.queue)

        rospy.loginfo("Center mean: {}".format(np.mean(c_q)))
        if self.turn_taken and np.mean(c_q) > 10:
            self.turn_taken = 1

        if len(c_q) != 0 and np.all(np.diff(c_q) <= 0) and center < self.right_th and not self.turn_taken:
            self.right_turn = True
            self.right_timer = time.time()
            rospy.loginfo("Right turn 1")
            return 10, 0.5
        elif len(l_q) != 0 and np.all(np.diff(l_q) <= 0):
            rospy.loginfo("Orientation left")
            return 6.5, 3.7
        elif len(r_q) != 0 and np.all(np.diff(r_q) <= 0):
            rospy.loginfo("Orientation right")
            return 5, 3.7
        elif len(c_q) != 0 and np.all(np.diff(c_q) <= 0) and center < 6 and self.turn_taken == 1:
            self.right_turn = True
            self.turn_taken = True
            self.right_timer = time.time()
            rospy.loginfo("Right turn 2")
            return 10, 0.5
        #elif len(c_q) != 0 and np.all(np.diff(c_q) <= 0) and center < 4:
        #    return 8, 1.75
        else:
            if np.mean(c_q) < 12:
                rospy.loginfo("vachan")
                return 6, max(1.5, 0.25 * np.mean(c_q))
            else:
                if self.right_turn is True:
                    return 10, 1.2
                return 6, 3.7


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
            self.speed -= 0.05
            velocity_control = Twist()
            if self.turn_taken:
                velocity_control.angular.z, velocity_control.linear.x = self.Pcontrol_steer()
            else:
                velocity_control.angular.z, velocity_control.linear.x = self.Pcontrol_steer()
            self.pub.publish(velocity_control)
            r.sleep()


if __name__ == '__main__':
    try:
        #Testing our function
        echo = Echo()
        signal.signal(signal.SIGINT, echo.signal_handler)
        echo.run()
    except rospy.ROSInterruptException: pass

