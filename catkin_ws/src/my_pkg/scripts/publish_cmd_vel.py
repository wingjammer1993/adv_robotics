#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import numpy as np

def move():
    # Starts a new node
    rospy.init_node('publish_velocity', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    rate = rospy.Rate(0.05) # 10hz
    flag = True
    while not rospy.is_shutdown():
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        if flag:
            vel_msg.angular.z = 0
        else:
            vel_msg.angular.z = 10
        flag = not(flag)
        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass