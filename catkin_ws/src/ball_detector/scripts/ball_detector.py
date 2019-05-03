import cv2 as cv
import imutils
import time
import rospy

class BallDetection(object):
    def __init__(self, lower_th, higher_th):
        self.lower_th = lower_th
        self.higher_th = higher_th
        self.bbox_x_min = 200
        self.bbox_x_max = 440

    def convert_to_hsv(self, image):
        blur_img = cv.GaussianBlur(image, (11, 11), 0)
        return cv.cvtColor(blur_img, cv.COLOR_BGR2HSV)

    def generate_mask(self, image):
        hsv_img = self.convert_to_hsv(image)

        mask = cv.inRange(hsv_img, self.lower_th, self.higher_th)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)

        return mask

    def detect_ball(self, image):
        image = image[200:340,:,:]
        image_mask = self.generate_mask(image)
        centers = cv.findContours(image_mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        centers = imutils.grab_contours(centers)

        radius = 0.0

        if len(centers) > 0:
            c = max(centers, key=cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(c)
            M = cv.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            
            if radius > 10:
                cv.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv.circle(image, center, 5, (0, 0, 255), -1)
                cv.line(image, (self.bbox_x_min, 0), (self.bbox_x_min, 480), (255, 255, 0), 2)
                cv.line(image, (self.bbox_x_max, 0), (self.bbox_x_max, 480), (255, 255, 0), 2)
                cv.line(image, (320, 0), (320, 480), (255, 0, 255), 2)
                cv.imwrite("/home/rock64/outputs/{}.jpg".format(time.time()), image)
            
                return True, center, radius

        return False, (), radius
