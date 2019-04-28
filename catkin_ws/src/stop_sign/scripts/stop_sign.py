#!/usr/bin/env python
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from stop import Stop
from time import time

from detector import Detector

from sensor_msgs.msg import Image
from std_msgs.msg import Bool


from threading import Event

class StopSign(object):
    def __init__(self, detector):
        #self.event = Event()
        #self.event.set()
        self.bridge = CvBridge()
        self.contains = False
        self.img, self.prev_img = None, None
        self.detector = detector

        # Publishes object recognition prediction
        self.stop_sign_pub = rospy.Publisher(
            'stop_sign/out',
             Bool,
             queue_size=10
         )

        # Subscribes to rs_camera color
        self.image_sub = rospy.Subscriber(
            "rgb_frame",
            Image,
            self.image_callback,
            queue_size=1
        )

    def image_callback(self, msg):
        # Perform prediction per the set frequency
        #if self.event.isSet():
    	#    self.event.clear()
            # Convert ros image to cv image
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            return
        self.prev_img = self.img 
        self.img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
    def get_latest_status(self):
        if self.img is not None:
            # self.contains = self.detector.contains_ss(self.img.copy())
            self.contains = self.detector.detect_haar(self.img.copy())
        else:
            self.contains = False

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.get_latest_status()
            if not np.array_equal(self.prev_img, self.img):
                if self.contains:
                    print "Stop sign detected."
                    self.stop_sign_pub.publish(True)
                else:
                    print "No stop sign detected."
                    self.stop_sign_pub.publish(False)
                    ##self.event.set()
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('stop_sign', anonymous=True)
    r = rospkg.RosPack()
    
    #try:
        #r = rospkg.RosPack()
        #target = r.get_path("stop_sign")+"/data/stop_sign.png"
        #print(target)
    #except IOError, e:
        #print e
    haar_classifier_path = r.get_path("stop_sign")+"/data/stopsign_classifier.xml"

    # detector = Detector(cv2.imread(target), (320, 240), debug=False)
    detector = Stop(haar_classifier_path)
    ss = StopSign(detector)
    
    ss.run()
