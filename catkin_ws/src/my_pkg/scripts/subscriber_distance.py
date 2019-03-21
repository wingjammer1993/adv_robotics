#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32

threshold_large = 0.75
threshold_small = 0.25
def callback(data):
    rospy.loginfo(data.data)
    rospy.loginfo(threshold_large)
    rospy.loginfo(threshold_small)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("depth_frame", Float32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()