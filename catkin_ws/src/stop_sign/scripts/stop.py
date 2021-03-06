import time

import cv2
import imutils
import numpy as np
import rospy
from skimage.transform import pyramid_gaussian


class Stop(object):
    def __init__(self, w_xml):
        # self.image = img
        self.classifier = cv2.CascadeClassifier(w_xml)


    def detect_haar(self, img, example=False):
        """
            Determines whether the input image contains a stop sign using the trained Haar classifier.
            If <example> is set, also draws a rectangle around the detected sign and displays the image.
        """
        # Detect any stop signs in the image using the classifier at various scales.
        stop_signs = self.classifier.detectMultiScale(img, 1.02, 10)

        return len(stop_signs) > 0
