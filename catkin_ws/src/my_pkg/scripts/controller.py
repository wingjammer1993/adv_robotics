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
        
        # Ball detection controls
        self.move_right = False
        self.move_left = False
        self.prev_steer = None
        self.steer_acc = queue.Queue()
        self.correct_timer = None

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
        rospy.Subscriber('collision', Int8, self.save_collision, queue_size=1)
        rospy.Subscriber('stop_sign/out', Bool, self.save_stop_sign, queue_size=1)
        rospy.Subscriber('ball_detect/out', Int8, self.ball_control, queue_size=1)

    def Pcontrol_steer(self):
        if self.collision_occr or self.stop_sign_detected:
            return 5.0            

        self.e_1 = self.e
        left = self.curr_position.linear.x
        center = self.curr_position.linear.y
        right = self.curr_position.linear.z

        #if self.correct_timer is not None:
        #    rospy.loginfo(time.time() - self.correct_timer)

        #if self.correct_timer is not None and time.time() - self.correct_timer > 0.5:
        #    rospy.loginfo("In the timer condn.")
        #    if self.prev_steer is not None:
        #        n_steer = 4.5
        #        self.prev_steer = None

        #        return n_steer
        #else:
        #    if self.prev_steer is not None:
        #        return self.prev_steer

        b_steer = None
        
        if self.prev_steer is None or time.time() - self.correct_timer < 0.8:
            if self.prev_steer is not None:
                return self.prev_steer

            if self.move_right == True:
                b_steer = 8
            elif self.move_left == True:
                b_steer = 2

            if b_steer is not None:
                self.prev_steer = b_steer
                self.correct_timer = time.time()

                return b_steer
        elif self.prev_steer is not None and self.prev_steer != -1:
            self.prev_steer = -1

            return 6

        rospy.loginfo("in normal PID loop")

        if center > 4 and center > right:
            return 6
        #elif center - 1.5 < right:
        #    return 5 + 3           
        else:
            self.e = (left - right)
            n_steer = 6 + self.p*(self.e) + self.d*(self.e_1 - self.e) + self.i*(self.e_1 + self.e)

            if n_steer < 3:
                return 3
            elif n_steer > 7:
                return 8
            else:
                return 6

    def signal_handler(self, sig, frame):
        velocity_control = Twist()
        velocity_control.linear.x = 0
        velocity_control.angular.z = 4
        print("sigint called")
        self.pub.publish(velocity_control)
        sys.exit(0)

    def update_value(self, msg):
        self.curr_position = msg
        #rospy.loginfo(self.curr_position)

    def save_stop_sign(self, msg):
        self.stop_sign_detected = msg.data
        rospy.loginfo("Stop sign: {}".format(self.stop_sign_detected))

    def save_collision(self, msg):
        if msg.data == 1 and not self.collision_occr:
            rospy.loginfo("Collision Detected. Backing out")
            self.collision_occr = True
            self.collision_time = time.time()

    def ball_control(self,msg):
        #if msg.data == 1:
        #    self.move_right = True
        #elif msg.data == -1:
        #    self.move_left = True
        #else:
        #    self.move_right = False
        #    self.move_left = False

        if self.steer_acc.qsize() >= 8:
            self.steer_acc.get()

        self.steer_acc.put(msg.data)

        rospy.loginfo(self.steer_acc.queue)

        if self.steer_acc.qsize() < 8:
            self.move_right = False
            self.move_left = False

            return

        a = np.array(self.steer_acc.queue)

        if np.all(a == 0):
            return

        if a[-1] != 0 and (np.all(a == 1) or np.all(np.diff(a) >= 0)):
            self.move_left = False
            self.move_right = True
        elif a[-1] != 0 and (np.all(a == -1) or np.all(np.diff(a) <= 0)):
            self.move_left = True
            self.move_right = False
        else:
            self.move_right = False
            self.move_left = False

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
