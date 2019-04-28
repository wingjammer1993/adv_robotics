from skimage.transform import pyramid_gaussian
import argparse
import cv2
import time
import numpy as np
import glob
import rospy
from numba import jit
#from matplotlib import pyplot as plt

class Detector(object):
    def __init__(self, target, image_size, debug=False, win_size=16, threshold = 8e-5, step_size=2):
        self.win_size = win_size
        self.target = cv2.resize(target, (win_size, win_size))
        self.threshold = threshold
        self.step_size = step_size
        self.image_size = image_size
        self.debug = debug
        
    @staticmethod
    @jit(nopython=True)
    def sliding_window(image, step_size, window_size):
        for y in range(0, image.shape[0], step_size):
            for x in range(0, image.shape[1], step_size):
                yield (x, y, image[y:y+window_size[1], x:x+window_size[1]])

    def meanSquareError(self, target, image):
        assert image.shape == target.shape, "Images must be the same shape."
        error = np.sum((image.astype("float") - target.astype("float")) ** 2)
        error = error/float(image.shape[0] * image.shape[1] * image.shape[2])
        return error

    def get_ssim(self, image):
        start = time.time()

        max_ssim = -1
        max_img = None
        max_bbox = []

        while image.shape[0] > 16 and  image.shape[1] > 16:
            image = cv2.pyrDown(image)
            for (x, y, window) in self.sliding_window(
                    image, step_size=self.step_size,
                    window_size=(self.win_size, self.win_size)
                ):
                if window.shape[0] != 16 or window.shape[1] != 16:
                    continue

                ssim = 1./self.meanSquareError(self.target, image[y:y+16, x:x+16])
                if ssim > max_ssim:
                    max_ssim = ssim
                    max_img = image.copy()
                    max_bbox = [(x, y), (x + 16, y + 16)]

                if self.debug:
                    clone = image.copy()
                    cv2.rectangle(clone, (x, y), (x + 16, y + 16), (255, 0, 0), 2)
                    cv2.imshow("Window",clone)
                    cv2.waitKey(1)
                    time.sleep(0.025)

        if self.debug:
            if max_ssim < self.threshold:
                cv2.rectangle(max_img, max_bbox[0], max_bbox[1], (0, 0, 255), 2)
            else:
                cv2.rectangle(max_img, max_bbox[0], max_bbox[1], (0, 255, 0), 2)

            cv2.imwrite("{}.png".format(name), max_img)
            cv2.destroyAllWindows()

        end = time.time()
        return max_ssim
## extra function added
    def contains_mse(self, image):
        mse = self.meanSquareError(self.target, image)
        return mse


    def contains_ss(self, image, debug=False):
        curr_time = time.time()
        if image.shape != self.image_size:
            image = cv2.resize(image, self.image_size)
        max_ssim = self.get_ssim(image)
        rospy.loginfo("Stop Sign SSIM: {}".format(max_ssim))
        rospy.loginfo("DET Time: {}".format(time.time() - curr_time))
        return max_ssim > self.threshold

    @staticmethod
    def compareImages(a, b):
        return 1./meanSquareError(a, b)
